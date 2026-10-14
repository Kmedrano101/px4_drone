#!/usr/bin/env python3
"""Genera con acados el solver MPC del ciclo arm-takeoff-hold-land-disarm.

Modelo: TRIPLE integrador por eje, en NED.
    estado  x = [px, py, pz, vx, vy, vz, ax, ay, az]   (9)
    control u = [jx, jy, jz]                            (3)   jerk
    dinamica: p' = v,  v' = a,  a' = j

Por que el jerk como control y no la aceleracion (que es lo que hace
SUPER/px4_mpc_controller): en Offboard PX4 1.14.3 NO suaviza la trayectoria --
no existe FlightTask de offboard, el setpoint entra crudo al controlador. Un
escalon de setpoint pide aceleracion maxima de golpe, que es lo que inclino el
dron en el patron en cruz del 2026-09-17. Con el jerk como control, el limite de
suavidad es una restriccion dura del problema y la rampa sale por diseno.

Salida al FC: la ACELERACION optima (primer tramo), publicada con posicion y
velocidad en NaN. Asi PX4 desactiva sus lazos de posicion y velocidad
(control_mode.cpp: offboard_control_mode.acceleration -> solo acceleration +
attitude + rates) y el MPC es de verdad el controlador exterior, comparable
contra el PID en pie de igualdad.

Restricciones (todas ajustables en caliente desde el nodo):
  - suelo: pz <= z_suelo (NED, z negativa hacia arriba) -- SUAVE, para que una
    mala estimacion no deje el QP infactible en pleno vuelo
  - velocidad: asimetrica en vertical (MPC_Z_VEL_MAX_UP / _DN de PX4) -- SUAVE
  - aceleracion: bajo el limite de PX4, que ademas NO satura el setpoint de
    aceleracion (issues 18033 / 18855): el nodo tambien lo recorta
  - jerk: dura, es la que da la suavidad

    python3 scripts/mpc/generate_cycle_mpc.py [--out-dir DIR] [-N 30] [--dt 0.05]
"""
import argparse
import os
import shutil
import sys

import numpy as np
from casadi import SX, vertcat

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

NX, NU = 9, 3


def crear_modelo():
    p = SX.sym("p", 3)
    v = SX.sym("v", 3)
    a = SX.sym("a", 3)
    j = SX.sym("j", 3)
    x = vertcat(p, v, a)
    xdot = SX.sym("xdot", NX)

    model = AcadosModel()
    model.name = "px4_cycle_mpc"
    model.x = x
    model.u = j
    model.xdot = xdot
    model.f_expl_expr = vertcat(v, a, j)
    model.f_impl_expr = xdot - model.f_expl_expr
    return model


def crear_ocp(N, dt):
    ocp = AcadosOcp()
    ocp.model = crear_modelo()
    ocp.dims.N = N

    # --- coste: seguimiento de [p, v, a] + penalizacion del jerk
    ny, ny_e = NX + NU, NX
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.Vx = np.vstack([np.eye(NX), np.zeros((NU, NX))])
    ocp.cost.Vu = np.vstack([np.zeros((NX, NU)), np.eye(NU)])
    ocp.cost.Vx_e = np.eye(NX)

    # Pesos iniciales; el nodo los reescribe segun la fase (subir, mantener,
    # aterrizar): mantener quiere mucho peso en posicion, aterrizar lo quiere
    # en velocidad vertical para acotar el contacto.
    q_pos = np.array([30.0, 30.0, 60.0])     # la altura pesa mas: es la fase 1
    q_vel = np.array([8.0, 8.0, 12.0])
    q_acc = np.array([1.0, 1.0, 1.0])        # suaviza y evita pedir empuje a saltos
    r_jerk = np.array([0.5, 0.5, 0.5])
    ocp.cost.W = np.diag(np.concatenate([q_pos, q_vel, q_acc, r_jerk]))
    ocp.cost.W_e = 3.0 * np.diag(np.concatenate([q_pos, q_vel, q_acc]))
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # --- restricciones de estado: pz (suelo), velocidad y aceleracion
    ocp.constraints.x0 = np.zeros(NX)
    idx_pz, idx_v, idx_a = [2], [3, 4, 5], [6, 7, 8]
    ocp.constraints.idxbx = np.array(idx_pz + idx_v + idx_a)
    # valores por defecto; el nodo los fija en cada ciclo
    ocp.constraints.lbx = np.array([-50.0, -1.0, -1.0, -1.05, -3.0, -3.0, -3.0])
    ocp.constraints.ubx = np.array([0.0, 1.0, 1.0, 0.80, 3.0, 3.0, 3.0])
    ocp.constraints.idxbx_e = ocp.constraints.idxbx
    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx

    # Suaves las de suelo y velocidad: con el estimador real (SLAM a 1-2 Hz,
    # rango con ruido) el estado puede entrar violando una restriccion, y un QP
    # infactible en vuelo es peor que una violacion penalizada.
    n_soft = 4  # pz + las tres velocidades
    ocp.constraints.idxsbx = np.arange(n_soft)
    ocp.constraints.idxsbx_e = np.arange(n_soft)
    pen_l1 = np.array([2e3, 1e2, 1e2, 5e2])   # el suelo, lo mas caro de violar
    pen_l2 = np.array([2e3, 1e2, 1e2, 5e2])
    ocp.cost.zl, ocp.cost.zu = pen_l1, pen_l1
    ocp.cost.Zl, ocp.cost.Zu = pen_l2, pen_l2
    ocp.cost.zl_e, ocp.cost.zu_e = pen_l1, pen_l1
    ocp.cost.Zl_e, ocp.cost.Zu_e = pen_l2, pen_l2

    # --- restriccion de control: jerk (dura, es la que da la suavidad)
    ocp.constraints.idxbu = np.arange(NU)
    ocp.constraints.lbu = np.array([-6.0, -6.0, -6.0])
    ocp.constraints.ubu = np.array([6.0, 6.0, 6.0])

    # --- solver
    ocp.solver_options.tf = N * dt
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"   # una iteracion por ciclo
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.print_level = 0
    return ocp


def banco_de_pruebas(solver, N, dt):
    """Casos representativos del ciclo, con tiempos de resolucion."""
    import time

    casos = {
        "despegue a 1 m desde el suelo": (np.zeros(NX), np.array([0.0, 0.0, -1.0])),
        "hold con 20 cm de error": (
            np.array([0.2, -0.1, -0.95, 0.1, 0.0, 0.05, 0, 0, 0]), np.array([0.0, 0.0, -1.0])),
        "aterrizaje desde 1 m": (
            np.array([0.0, 0.0, -1.0, 0, 0, 0, 0, 0, 0]), np.array([0.0, 0.0, -0.17])),
        "estado violando el suelo (estimador malo)": (
            np.array([0.0, 0.0, 0.15, 0, 0, 0.4, 0, 0, 0]), np.array([0.0, 0.0, -1.0])),
    }
    print(f"\n{'caso':<42} {'estado':>7} {'t_solve':>9}   primera aceleracion [m/s2]")
    ok = True
    for nombre, (x0, p_ref) in casos.items():
        yref = np.concatenate([p_ref, np.zeros(6), np.zeros(NU)])
        for k in range(N):
            solver.set(k, "yref", yref)
        solver.set(N, "yref", yref[:NX])
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)
        t0 = time.perf_counter()
        status = solver.solve()
        t_ms = (time.perf_counter() - t0) * 1e3
        u0 = solver.get(0, "u")
        x1 = solver.get(1, "x")
        acc = x1[6:9]
        print(f"{nombre:<42} {status:>7} {t_ms:7.2f} ms   "
              f"[{acc[0]:+.2f} {acc[1]:+.2f} {acc[2]:+.2f}]  jerk0=[{u0[0]:+.1f} {u0[1]:+.1f} {u0[2]:+.1f}]")
        ok &= status in (0, 2)  # 0 ok, 2 maxiter (aceptable en RTI)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "generated"))
    ap.add_argument("-N", type=int, default=30, help="pasos del horizonte")
    ap.add_argument("--dt", type=float, default=0.05, help="paso (s)")
    ap.add_argument("--no-test", action="store_true")
    a = ap.parse_args()

    print(f"Horizonte: {a.N} pasos x {a.dt} s = {a.N * a.dt:.2f} s")
    if os.path.exists(a.out_dir):
        shutil.rmtree(a.out_dir)
    os.makedirs(a.out_dir)

    cwd = os.getcwd()
    os.chdir(a.out_dir)
    try:
        solver = AcadosOcpSolver(crear_ocp(a.N, a.dt), json_file="acados_ocp_px4_cycle_mpc.json")
    except Exception as e:
        os.chdir(cwd)
        sys.exit(f"ERROR generando el solver: {e}")

    ok = True if a.no_test else banco_de_pruebas(solver, a.N, a.dt)
    os.chdir(cwd)
    print(f"\nCodigo C generado en: {a.out_dir}/c_generated_code")
    print("Para la Pi: se copia ese C y se compila alli (no hace falta casadi ni python).")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
