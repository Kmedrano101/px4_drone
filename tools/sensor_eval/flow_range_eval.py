#!/usr/bin/env python3
"""Evalua el flujo optico + LiDAR 1D (MTF-01P) contra una verdad de terreno medida
con cinta. Solo lee: no arma, no cambia parametros.

Procedimiento (dron en la mano, motores apagados, sensor apuntando al suelo):
  1. Mide con cinta la altura de la lente del sensor al suelo -> --gt-height.
  2. Pon una cinta en el suelo en direccion del morro, de --gt-distance metros.
  3. Fase QUIETO (--still s): el dron inmovil a esa altura -> ruido y sesgo del
     LiDAR, deriva y ruido del flujo.
  4. Fase MOVER: lleva el dron HACIA ADELANTE (morro) --gt-distance metros a lo
     largo de la cinta, a velocidad constante, nivelado y a la misma altura.
     Quedate quieto al final hasta que termine.
  Con --gt-distance 0 todo es fase quieto.

Repite a varias alturas (0.5 / 1.0 / 1.5 / 2.0 m): cada corrida se agrega a
sensor_eval_history.csv y, con 2 o mas alturas, se ajusta la curva de error
del LiDAR (escala + offset).

Convencion del flujo = la del EKF2 1.14.3 (optflow_fusion.cpp:66-67):
    vx_body = -flujo_y_compensado * h     vy_body = +flujo_x_compensado * h
    flujo_compensado = integrated - integrated_gyro      (por muestra)

    python3 flow_range_eval.py --gt-height 1.00 --gt-distance 1.0 --duration 25
"""
import argparse, csv, glob, json, math, os, sys, time
from pymavlink import mavutil

M = mavutil.mavlink
PARAMS = ["SENS_FLOW_ROT", "SENS_FLOW_MINHGT", "SENS_FLOW_MAXHGT", "EKF2_OF_QMIN",
          "EKF2_OF_N_MIN", "EKF2_OF_N_MAX", "EKF2_OF_POS_X", "EKF2_OF_POS_Z",
          "EKF2_MIN_RNG", "EKF2_RNG_NOISE", "EKF2_RNG_SFE", "EKF2_RNG_POS_Z",
          "EKF2_RNG_A_HMAX"]
PLAUSIBLE_PCT = 25.0   # un error del LiDAR mayor que esto es casi seguro una altura mal introducida
MIN_MOVE_FRAC = 0.2    # por debajo de este % del recorrido esperado, no hubo movimiento
MIN_CURVE_SPAN_M = 0.3 # separacion minima entre alturas para ajustar la recta del LiDAR
HIST_COLS = ["run_id", "gt_height_m", "lidar_mean_m", "lidar_std_m", "lidar_err_pct",
             "flow_q_still", "flow_drift_m_s", "gt_distance_m", "flow_scale_err_pct_lidar_h",
             "flow_scale_err_pct_gt_h", "flow_dir_deg", "usable_for_curve"]
STREAMS = {M.MAVLINK_MSG_ID_DISTANCE_SENSOR: 50, M.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: 50,
           M.MAVLINK_MSG_ID_ATTITUDE: 50}


def log(msg=""):
    print(msg, flush=True)


def find_port(explicit, wait_s=60):
    t0 = time.time()
    while True:
        if explicit:
            if os.path.exists(explicit):
                return explicit
        else:
            cands = sorted(glob.glob("/dev/serial/by-id/*PX4*") + glob.glob("/dev/ttyACM*"))
            if cands:
                return cands[0]
        if time.time() - t0 > wait_s:
            raise SystemExit("ERROR: no aparece el FC por USB. Conecta el cable USB-C del FC a la Pi.")
        time.sleep(1)


def connect(dev, timeout=30):
    """El USB del PX4 solo transmite cuando recibe MAVLink: heartbeats hasta oir el suyo."""
    m = mavutil.mavlink_connection(dev, baud=115200, source_system=250, source_component=191)
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb is not None and hb.get_srcSystem() != 250:
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return m, hb
    raise SystemExit("ERROR: sin heartbeat del FC")


def read_params(m):
    out = {}
    for name in PARAMS:
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 2:
            p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if p and p.param_id.strip("\x00") == name:
                v = p.param_value
                if p.param_type != M.MAV_PARAM_TYPE_REAL32:  # int32 metido bit a bit en el float
                    import struct
                    v = struct.unpack("<i", struct.pack("<f", v))[0]
                out[name] = v
                break
    return out


def set_streams(m, on):
    for mid, hz in STREAMS.items():
        m.mav.command_long_send(m.target_system, m.target_component,
                                M.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                mid, (1e6 / hz) if on else 0, 0, 0, 0, 0, 0)


def stats(v):
    v = [x for x in v if x is not None and math.isfinite(x)]
    if not v:
        return None
    n = len(v)
    mu = sum(v) / n
    sd = math.sqrt(sum((x - mu) ** 2 for x in v) / max(n - 1, 1))
    return {"n": n, "mean": mu, "std": sd, "min": min(v), "max": max(v)}


def fit_line(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return a, my - a * mx


def migrate_history(hist):
    """Historial de antes de usable_for_curve: se reescribe con la columna, marcando como no usables
    las corridas cuyo error del LiDAR no es verosimil (tipicamente una altura mal introducida)."""
    if not os.path.exists(hist):
        return
    with open(hist) as f:
        r = csv.DictReader(f)
        if r.fieldnames == HIST_COLS:
            return
        rows = list(r)
    for row in rows:
        try:
            ok = abs(100 * (float(row["lidar_mean_m"]) / float(row["gt_height_m"]) - 1)) <= PLAUSIBLE_PCT
        except (TypeError, ValueError, KeyError, ZeroDivisionError):
            ok = False
        row["usable_for_curve"] = int(ok)
    os.replace(hist, hist + ".bak")
    with open(hist, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=HIST_COLS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    log(f"(historial migrado al formato nuevo; copia del anterior en {hist}.bak)")


def report_curve(hist):
    """Ajusta lectura = k*real + b con las corridas utilizables del historial."""
    pts = []
    with open(hist) as f:
        for r in csv.DictReader(f):
            try:
                gt, mean = float(r["gt_height_m"]), float(r["lidar_mean_m"])
            except (TypeError, ValueError):
                continue
            use = r.get("usable_for_curve")
            if use in (None, ""):  # historial viejo sin la columna
                use = abs(100 * (mean / gt - 1)) <= PLAUSIBLE_PCT
            if str(use) in ("1", "True"):
                pts.append((gt, mean))
    span = (max(p[0] for p in pts) - min(p[0] for p in pts)) if pts else 0.0
    if len(pts) >= 2 and span < MIN_CURVE_SPAN_M:
        log(f"\nCURVA DEL LIDAR: las alturas validas van de {min(p[0] for p in pts):.2f} a "
            f"{max(p[0] for p in pts):.2f} m; hacen falta alturas separadas al menos {MIN_CURVE_SPAN_M:.1f} m "
            "(el sensor mide en pasos de 1 cm: con alturas casi iguales la recta sale de ruido)")
    elif len({round(p[0], 2) for p in pts}) >= 2:
        fit = fit_line([p[0] for p in pts], [p[1] for p in pts])
        if fit:
            k, b = fit
            log(f"\nCURVA DEL LIDAR con {len(pts)} corridas validas a {len({round(p[0], 2) for p in pts})} alturas:")
            log(f"  lectura = {k:.4f} x real {b:+.4f} m   (ideal: 1.0000 x real +0.0000)")
            for hh in (0.5, 1.0, 1.5, 2.0, 3.0):
                log(f"    a {hh:.1f} m real el LiDAR diria {k * hh + b:.3f} m ({100 * ((k * hh + b) / hh - 1):+.1f}%)")
    else:
        log(f"\nCURVA DEL LIDAR: {len(pts)} corrida(s) valida(s); hacen falta al menos 2 alturas distintas")



def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt-height", type=float, required=True,
                    help="altura medida con cinta de la lente del sensor al suelo (m)")
    ap.add_argument("--gt-distance", type=float, default=1.0,
                    help="distancia a recorrer hacia adelante a mano (m); 0 = solo quieto")
    ap.add_argument("--duration", type=float, default=25.0, help="duracion total (s)")
    ap.add_argument("--still", type=float, default=6.0, help="fase quieto inicial (s)")
    ap.add_argument("--device", default=os.environ.get("FC_DEV", ""), help="puerto (auto si vacio)")
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
    a = ap.parse_args()

    if not (0.05 <= a.gt_height <= 12):
        raise SystemExit("ERROR: --gt-height fuera de rango (0.05-12 m)")
    if a.still >= a.duration:
        raise SystemExit("ERROR: --still debe ser menor que --duration")
    os.makedirs(a.out_dir, exist_ok=True)
    run_id = time.strftime("sensor_eval_%Y%m%d_%H%M%S")

    log("=" * 64)
    log(f"EVALUACION FLUJO OPTICO + LIDAR 1D   ({run_id})")
    log(f"Verdad de terreno: altura sensor-suelo {a.gt_height:.3f} m, recorrido {a.gt_distance:.2f} m")
    log("Solo lectura: no arma ni cambia parametros. MOTORES APAGADOS.")
    log("=" * 64)

    dev = find_port(a.device)
    log(f"FC en {dev}, conectando...")
    m, hb = connect(dev)
    if hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit("ERROR: el FC esta ARMADO. Esta prueba es con el dron desarmado.")
    prm = read_params(m)
    log("Parametros actuales: " + ", ".join(f"{k}={v:g}" for k, v in prm.items()))
    set_streams(m, True)

    rows = []
    rng_min_dev = rng_max_dev = None
    roll = pitch = 0.0
    last_rng = float("nan")
    t0 = time.time()
    phase = "QUIETO"
    log("")
    log(f">>> FASE QUIETO: manten el dron INMOVIL a {a.gt_height:.2f} m durante {a.still:.0f} s")
    next_print = t0 + 1.0
    dx = dy = 0.0
    try:
        while True:
            now = time.time()
            t = now - t0
            if t >= a.duration:
                break
            if phase == "QUIETO" and t >= a.still and a.gt_distance > 0:
                phase = "MOVER"
                log("")
                log(f">>> FASE MOVER: lleva el dron HACIA ADELANTE (morro) {a.gt_distance:.2f} m por la cinta,")
                log("    despacio y constante, nivelado, misma altura. Al llegar, quieto hasta el final.")
            msg = m.recv_match(type=["DISTANCE_SENSOR", "OPTICAL_FLOW_RAD", "ATTITUDE"],
                               blocking=True, timeout=0.5)
            if msg is None:
                continue
            typ = msg.get_type()
            if typ == "ATTITUDE":
                roll, pitch = msg.roll, msg.pitch
                continue
            if typ == "DISTANCE_SENSOR":
                rng_min_dev, rng_max_dev = msg.min_distance / 100.0, msg.max_distance / 100.0
                last_rng = msg.current_distance / 100.0
                rows.append({"t": t, "phase": phase, "type": "rng", "rng_m": last_rng,
                             "roll": roll, "pitch": pitch})
                continue
            # OPTICAL_FLOW_RAD
            dt = msg.integration_time_us / 1e6
            if dt <= 0:
                continue
            h_lidar = (msg.distance if msg.distance > 0 else last_rng) * math.cos(roll) * math.cos(pitch)
            h_gt = a.gt_height * math.cos(roll) * math.cos(pitch)
            fx = msg.integrated_x - msg.integrated_xgyro
            fy = msg.integrated_y - msg.integrated_ygyro
            ddx, ddy = -fy * h_lidar, fx * h_lidar
            if msg.quality > 0:
                dx += ddx
                dy += ddy
            rows.append({"t": t, "phase": phase, "type": "flow", "q": msg.quality, "dt": dt,
                         "fx": fx, "fy": fy, "rate_x": fx / dt, "rate_y": fy / dt,
                         "h_lidar": h_lidar, "h_gt": h_gt,
                         "dx_lidar": ddx, "dy_lidar": ddy, "dx_gt": -fy * h_gt, "dy_gt": fx * h_gt,
                         "roll": roll, "pitch": pitch})
            if now >= next_print:
                next_print += 1.0
                log(f"  t={t:5.1f}s [{phase:6s}] lidar={last_rng:5.2f} m  calidad={msg.quality:3d}  "
                    f"desplazamiento x={dx:+.2f} y={dy:+.2f} m  tilt={math.degrees(max(abs(roll), abs(pitch))):4.1f}deg")
    except KeyboardInterrupt:
        log("\n(parado desde la webui: se analiza lo grabado hasta ahora)")
    finally:
        set_streams(m, False)

    # ---------------------------------------------------------------- analisis
    rng = [r for r in rows if r["type"] == "rng"]
    flo = [r for r in rows if r["type"] == "flow"]
    if not rows:
        raise SystemExit("ERROR: no llego ninguna muestra del FC (revisa el cable USB)")
    dur = rows[-1]["t"] - rows[0]["t"]
    res = {"run_id": run_id, "gt_height_m": a.gt_height, "gt_distance_m": a.gt_distance,
           "duration_s": dur, "params": prm}

    log("")
    log("=" * 64)
    log("RESULTADOS")
    log("=" * 64)

    # --- LiDAR 1D (fase quieto: la altura es la verdad de terreno)
    still_rng = [r["rng_m"] for r in rng if r["phase"] == "QUIETO"]
    invalid = [x for x in still_rng if rng_min_dev is not None and (x <= rng_min_dev or x >= rng_max_dev)]
    s = stats([x for x in still_rng if x not in invalid])
    log(f"\nLiDAR 1D  ({len(rng) / max(dur, 1e-6):.0f} Hz, fase quieto {len(still_rng)} muestras)")
    if s:
        err = s["mean"] - a.gt_height
        pct = 100 * err / a.gt_height
        res["lidar"] = {**s, "error_m": err, "error_pct": pct,
                        "invalid_pct": 100 * len(invalid) / max(len(still_rng), 1)}
        log(f"  media {s['mean']:.3f} m  (verdad {a.gt_height:.3f})  ->  error {err:+.3f} m ({pct:+.1f}%)")
        log(f"  ruido (desv. tipica) {s['std']:.3f} m   rango {s['min']:.2f}-{s['max']:.2f} m   "
            f"invalidas {res['lidar']['invalid_pct']:.0f}%")
        log(f"  resolucion del sensor por MAVLink: 1 cm")
        if abs(pct) > PLAUSIBLE_PCT:
            verdict = (f"NO SE USA PARA LA CURVA: {pct:+.0f}% es demasiado para un error del sensor. "
                       "¿La altura introducida es la real? (prueba en banco = altura en patas, ~0.17 m)")
        elif abs(err) <= max(0.03, 0.03 * a.gt_height):
            verdict = "OK"
        else:
            verdict = "SESGO: revisar montaje/haz o calibracion del sensor"
        res["lidar"]["usable_for_curve"] = abs(pct) <= PLAUSIBLE_PCT
        log(f"  veredicto: {verdict}")
        if prm.get("EKF2_RNG_NOISE") is not None:
            log(f"  EKF2_RNG_NOISE={prm['EKF2_RNG_NOISE']:.3f} m frente al ruido medido {s['std']:.3f} m"
                + ("  (el parametro cubre el ruido)" if prm['EKF2_RNG_NOISE'] >= s['std'] else "  <- SUBIRLO"))
    else:
        log("  sin lecturas validas del LiDAR en la fase quieto")

    # --- Flujo: fase quieto
    fq = [r for r in flo if r["phase"] == "QUIETO"]
    qmin = prm.get("EKF2_OF_QMIN", 30)
    log(f"\nFLUJO OPTICO  ({len(flo) / max(dur, 1e-6):.0f} Hz)")
    if fq:
        q = stats([r["q"] for r in fq])
        low = 100 * sum(1 for r in fq if r["q"] < qmin) / len(fq)
        rate = stats([math.hypot(r["rate_x"], r["rate_y"]) for r in fq])
        drift = math.hypot(sum(r["dx_lidar"] for r in fq), sum(r["dy_lidar"] for r in fq))
        tq = fq[-1]["t"] - fq[0]["t"]
        res["flow_still"] = {"quality_mean": q["mean"], "quality_min": q["min"], "below_qmin_pct": low,
                             "rate_noise_rad_s": rate["mean"], "drift_m": drift,
                             "drift_speed_m_s": drift / max(tq, 1e-6)}
        log(f"  QUIETO: calidad media {q['mean']:.0f} (min {q['min']:.0f}), bajo EKF2_OF_QMIN={qmin:g}: {low:.0f}%")
        log(f"  ruido de flujo compensado {rate['mean']:.3f} rad/s  (EKF2_OF_N_MIN={prm.get('EKF2_OF_N_MIN', float('nan')):.2f})")
        log(f"  deriva estando quieto: {drift:.3f} m en {tq:.0f} s = {drift / max(tq, 1e-6):.3f} m/s")

    # --- Flujo: fase mover
    fm = [r for r in flo if r["phase"] == "MOVER" and r["q"] > 0]
    if a.gt_distance > 0 and fm:
        mx, my = sum(r["dx_lidar"] for r in fm), sum(r["dy_lidar"] for r in fm)
        gx, gy = sum(r["dx_gt"] for r in fm), sum(r["dy_gt"] for r in fm)
        d_l, d_g = math.hypot(mx, my), math.hypot(gx, gy)
        ang = math.degrees(math.atan2(my, mx))
        tilt = max(math.degrees(max(abs(r["roll"]), abs(r["pitch"]))) for r in fm)
        qm = stats([r["q"] for r in fm])
        res["flow_move"] = {"measured_m_lidar_h": d_l, "measured_m_gt_h": d_g,
                            "scale_err_pct_lidar_h": 100 * (d_l / a.gt_distance - 1),
                            "scale_err_pct_gt_h": 100 * (d_g / a.gt_distance - 1),
                            "direction_deg": ang, "x_m": mx, "y_m": my,
                            "max_tilt_deg": tilt, "quality_mean": qm["mean"], "quality_min": qm["min"]}
        log(f"\n  MOVER: recorrido medido {d_l:.3f} m con altura del LiDAR, {d_g:.3f} m con la altura real"
            f"  (verdad {a.gt_distance:.2f} m)")
        res["flow_move"]["moved"] = max(d_l, d_g) >= MIN_MOVE_FRAC * a.gt_distance
        if not res["flow_move"]["moved"]:
            log(f"  NO SE DETECTO MOVIMIENTO (menos del {100 * MIN_MOVE_FRAC:.0f}% de lo esperado): ¿se movio el dron?")
            log("  -> escala y direccion no son validas en esta corrida")
        if res["flow_move"]["moved"]:
            log(f"  error de escala: {res['flow_move']['scale_err_pct_lidar_h']:+.1f}% (altura LiDAR), "
                f"{res['flow_move']['scale_err_pct_gt_h']:+.1f}% (altura real)")
            log("    -> si solo falla con la altura del LiDAR, el error viene del LiDAR; si falla con las dos, del flujo")
            log(f"  direccion en cuerpo: {ang:+.0f} deg (esperado ~0 = hacia el morro)   x={mx:+.2f} y={my:+.2f} m")
        if res["flow_move"]["moved"] and abs(ang) > 30:
            guess = {90: "rotado +90", -90: "rotado -90", 180: "rotado 180"}
            k = min(guess, key=lambda g: abs(((ang - g + 180) % 360) - 180))
            log(f"  !! DIRECCION INCORRECTA: el sensor parece {guess[k]} grados -> revisar SENS_FLOW_ROT "
                f"(ahora {prm.get('SENS_FLOW_ROT', '?')})")
        log(f"  calidad en movimiento: media {qm['mean']:.0f}, min {qm['min']:.0f}   inclinacion max {tilt:.1f} deg"
            + ("  (alta: repetir mas nivelado)" if tilt > 10 else ""))
        if res["flow_move"]["moved"]:
            log("  nota: la 1.14.3 no tiene SENS_FLOW_SCALE; un error de escala no se corrige por parametro de PX4")
    elif a.gt_distance > 0:
        log("\n  MOVER: sin muestras de flujo validas en la fase de movimiento")

    # --- guardar
    base = os.path.join(a.out_dir, run_id)
    keys = sorted({k for r in rows for k in r})
    with open(base + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with open(base + ".json", "w") as f:
        json.dump(res, f, indent=2)

    hist = os.path.join(a.out_dir, "sensor_eval_history.csv")
    migrate_history(hist)
    new = not os.path.exists(hist)
    with open(hist, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(HIST_COLS)
        L, FS, FM = res.get("lidar", {}), res.get("flow_still", {}), res.get("flow_move", {})
        moved = FM.get("moved", False)
        w.writerow([run_id, a.gt_height, L.get("mean"), L.get("std"), L.get("error_pct"),
                    FS.get("quality_mean"), FS.get("drift_speed_m_s"), a.gt_distance,
                    FM.get("scale_err_pct_lidar_h") if moved else None,
                    FM.get("scale_err_pct_gt_h") if moved else None,
                    FM.get("direction_deg") if moved else None,
                    int(bool(L.get("usable_for_curve", False)))])

    report_curve(hist)

    log(f"\nGuardado: {base}.csv / .json  e historial {hist}")
    log("FIN")


if __name__ == "__main__":
    main()
