#!/usr/bin/env python3
"""Genera con acados el NMPC de empuje colectivo + velocidades angulares (CTBR).

Esto es el "MPC moderno": a diferencia del ciclo clasico (triple integrador
cinematico que publica ACELERACION y deja a PX4 cerrar actitud y tasas), aqui el
modelo incluye la ACTITUD y la salida son las TASAS ANGULARES y el empuje, que es
lo que publica la literatura agil (HPA-MPC arXiv 2411.11982, Dual-Channel
Adaptive NMPC arXiv 2507.15261).

    estado  x = [px,py,pz, vx,vy,vz, qw,qx,qy,qz]      (10)   NED + cuaternion
    control u = [T, wx, wy, wz]                         (4)
        T  = empuje ESPECIFICO (empuje/masa, m/s^2): asi no hace falta la masa
        w  = velocidades angulares en el cuerpo (FRD, rad/s)

    p' = v
    v' = R(q) * [0,0,-T] + [0,0,g]        NED: z hacia abajo, empuje hacia -z_cuerpo
    q' = 0.5 * q (x) [0, w]

En v1.14.3 esto es alcanzable con el firmware de fabrica, verificado en el
fuente del tag: OffboardControlMode tiene el campo body_rate,
control_mode.cpp:144 deja SOLO flag_control_rates_enabled (posicion, velocidad,
aceleracion Y ACTITUD apagadas), y /fmu/in/vehicle_rates_setpoint esta como
suscripcion en dds_topics.yaml.

El coste es la TASA DEL LAZO: al comandar tasas, el lazo de actitud pasa a ser
nuestro y hay que cerrarlo a >=100 Hz (HPA-MPC va a 150 Hz con N=10). Por eso
este generador acepta -N y --dt y mide el tiempo de solver: el numero decide si
cabe en la Pi 4 o hace falta otro ordenador.

    python3 scripts/mpc/generate_ctbr_mpc.py [--out-dir DIR] [-N 20] [--dt 0.05]
"""
import argparse
import os
import shutil
import sys
import time

import numpy as np
from casadi import SX, vertcat, horzcat

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

NX, NU = 10, 4
G = 9.80665


def crear_modelo():
    p = SX.sym("p", 3)
    v = SX.sym("v", 3)
    q = SX.sym("q", 4)          # [w, x, y, z], cuerpo FRD -> NED
    T = SX.sym("T")             # empuje especifico (m/s^2), siempre >= 0
    w = SX.sym("w", 3)          # tasas en el cuerpo (rad/s)

    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    # Tercera columna de R(q): hacia donde apunta el eje z del cuerpo en NED.
    # El empuje va por -z_cuerpo, asi que v' = -T * Rz + g.
    rz = vertcat(
        2.0 * (qw * qy + qx * qz),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    v_dot = -T * rz + vertcat(0.0, 0.0, G)

    wx, wy, wz = w[0], w[1], w[2]
    q_dot = 0.5 * vertcat(
        -qx * wx - qy * wy - qz * wz,
        qw * wx + qy * wz - qz * wy,
        qw * wy - qx * wz + qz * wx,
        qw * wz + qx * wy - qy * wx,
    )

    model = AcadosModel()
    model.name = "px4_ctbr_mpc"
    model.x = vertcat(p, v, q)
    model.u = vertcat(T, w)
    model.xdot = SX.sym("xdot", NX)
    model.f_expl_expr = vertcat(v, v_dot, q_dot)
    model.f_impl_expr = model.xdot - model.f_expl_expr
    return model


def crear_ocp(N, dt):
    ocp = AcadosOcp()
    ocp.model = crear_modelo()
    ocp.name = "px4_ctbr_mpc"
    if hasattr(ocp, "code_gen_options"):
        ocp.code_gen_options.code_export_directory = "c_generated_code"
    else:
        ocp.code_export_directory = "c_generated_code"
    ocp.dims.N = N

    # --- coste lineal sobre [p, v, q, u]
    ny, ny_e = NX + NU, NX
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.Vx = np.vstack([np.eye(NX), np.zeros((NU, NX))])
    ocp.cost.Vu = np.vstack([np.zeros((NX, NU)), np.eye(NU)])
    ocp.cost.Vx_e = np.eye(NX)

    q_pos = np.array([30.0, 30.0, 60.0])        # la altura pesa mas, como en el clasico
    q_vel = np.array([8.0, 8.0, 12.0])
    # El cuaternion se lleva a la identidad con poco peso: no queremos imponer la
    # actitud, queremos que salga la que produzca la aceleracion pedida, pero sin
    # dejar que el yaw y la inclinacion vaguen. Coste lineal sobre el cuaternion:
    # vale cerca de nivelado, que es el caso de vuelo interior. Para vuelo agresivo
    # habria que usar error de cuaternion de verdad (doble cobertura).
    q_att = np.array([5.0, 20.0, 20.0, 5.0])
    r_u = np.array([0.5, 2.0, 2.0, 2.0])        # empuje y tasas: suavidad
    ocp.cost.W = np.diag(np.concatenate([q_pos, q_vel, q_att, r_u]))
    ocp.cost.W_e = 3.0 * np.diag(np.concatenate([q_pos, q_vel, q_att]))
    yref = np.zeros(ny)
    yref[6] = 1.0        # cuaternion identidad
    yref[NX] = G         # empuje de hover: el dron sostiene su peso
    ocp.cost.yref = yref
    ocp.cost.yref_e = yref[:NX]

    # --- restricciones de estado: pz (suelo) y velocidades, suaves
    ocp.constraints.x0 = np.array([0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0])
    ocp.constraints.idxbx = np.array([2, 3, 4, 5])
    ocp.constraints.lbx = np.array([-50.0, -1.0, -1.0, -1.05])
    ocp.constraints.ubx = np.array([0.0, 1.0, 1.0, 0.80])
    ocp.constraints.idxbx_e = ocp.constraints.idxbx
    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx
    ocp.constraints.idxsbx = np.arange(4)
    ocp.constraints.idxsbx_e = np.arange(4)
    pen = np.array([2e3, 1e2, 1e2, 5e2])
    ocp.cost.zl, ocp.cost.zu = pen, pen
    ocp.cost.Zl, ocp.cost.Zu = pen, pen
    ocp.cost.zl_e, ocp.cost.zu_e = pen, pen
    ocp.cost.Zl_e, ocp.cost.Zu_e = pen, pen

    # --- restricciones de control: DURAS. El empuje nunca negativo (un
    # multirrotor no tira hacia abajo) y las tasas acotadas para interior.
    ocp.constraints.idxbu = np.arange(NU)
    ocp.constraints.lbu = np.array([0.5 * G, -1.5, -1.5, -1.0])
    ocp.constraints.ubu = np.array([1.6 * G, 1.5, 1.5, 1.0])

    ocp.solver_options.tf = N * dt
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.print_level = 0
    return ocp


def banco_de_pruebas(solver, N, dt, repeticiones):
    def x(pz, vz=0.0, px_=0.0, vx=0.0):
        return np.array([px_, 0, pz, vx, 0, vz, 1.0, 0, 0, 0])

    casos = {
        "hover a 0.8 m, en la referencia": (x(-0.8), np.array([0.0, 0.0, -0.8])),
        "20 cm bajo la referencia": (x(-0.6), np.array([0.0, 0.0, -0.8])),
        "30 cm de error horizontal": (x(-0.8, px_=0.3), np.array([0.0, 0.0, -0.8])),
        "subiendo desde el suelo": (x(-0.05), np.array([0.0, 0.0, -0.8])),
        "cayendo a 0.5 m/s": (x(-0.8, vz=0.5), np.array([0.0, 0.0, -0.8])),
    }
    print(f"\n{'caso':<34} {'est':>4} {'t_med':>8} {'t_max':>8}   T [m/s2]  w [rad/s]")
    ok = True
    tiempos_todos = []
    for nombre, (x0, p_ref) in casos.items():
        yref = np.zeros(NX + NU)
        yref[:3] = p_ref
        yref[6] = 1.0
        yref[NX] = G
        for k in range(N):
            solver.set(k, "yref", yref)
        solver.set(N, "yref", yref[:NX])
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)
        ts = []
        for _ in range(repeticiones):
            t0 = time.perf_counter()
            status = solver.solve()
            ts.append((time.perf_counter() - t0) * 1e3)
        tiempos_todos += ts
        u0 = solver.get(0, "u")
        print(f"{nombre:<34} {status:>4} {np.mean(ts):6.2f}ms {max(ts):6.2f}ms   "
              f"{u0[0]:6.2f}   [{u0[1]:+.2f} {u0[2]:+.2f} {u0[3]:+.2f}]")
        ok &= status in (0, 2)
    t = np.array(tiempos_todos)
    print(f"\nTIEMPO DE SOLVER sobre {len(t)} resoluciones: "
          f"media {t.mean():.2f} ms, p95 {np.percentile(t, 95):.2f} ms, max {t.max():.2f} ms")
    for hz in (50, 100, 150):
        presupuesto = 1000.0 / hz
        veredicto = "CABE" if t.max() < 0.5 * presupuesto else (
            "JUSTO" if t.max() < presupuesto else "NO CABE")
        print(f"  a {hz:3d} Hz el ciclo son {presupuesto:5.2f} ms  ->  "
              f"{veredicto} (peor caso usa el {100 * t.max() / presupuesto:.0f}%)")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "generated_ctbr"))
    ap.add_argument("-N", type=int, default=20)
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--repeticiones", type=int, default=50)
    ap.add_argument("--no-test", action="store_true")
    a = ap.parse_args()

    print(f"Horizonte: {a.N} pasos x {a.dt} s = {a.N * a.dt:.2f} s   "
          f"(NX={NX}, NU={NU}, modelo no lineal con cuaternion)")
    if os.path.exists(a.out_dir):
        shutil.rmtree(a.out_dir)
    os.makedirs(a.out_dir)

    cwd = os.getcwd()
    os.chdir(a.out_dir)
    try:
        solver = AcadosOcpSolver(crear_ocp(a.N, a.dt), json_file="acados_ocp_px4_ctbr_mpc.json")
    except Exception as e:
        os.chdir(cwd)
        sys.exit(f"ERROR generando el solver: {e}")

    ok = True if a.no_test else banco_de_pruebas(solver, a.N, a.dt, a.repeticiones)
    os.chdir(cwd)
    print(f"\nCodigo C generado en: {a.out_dir}/c_generated_code")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
