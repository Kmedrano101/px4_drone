#!/usr/bin/env python3
"""Registra la carga del ordenador de a bordo durante una prueba, para decidir si
la Raspberry Pi 4 se queda corta y hace falta otro ordenador compacto.

La webui lo arranca junto a cada prueba (como el rosbag) y lo para al terminar con
SIGINT. Escribe un CSV (una fila por periodo) y, al parar, un resumen que tambien
se agrega al final del log de la prueba.

Que mide y por que:
- CPU total y POR NUCLEO: slam_toolbox procesa los barridos en un solo hilo; una CPU
  total del 30% puede esconder un nucleo al 100%, que es lo que limita al SLAM.
- Temperatura, frecuencia y los bits de `vcgencmd get_throttled` (subtension,
  frecuencia recortada, throttling, limite termico): en un dron alimentado por BEC
  la subtension es tan probable como el calor.
- RAM, swap, escritura a disco (el rosbag) y red.
- CPU y RAM de los procesos del stack (SLAM, LiDAR, agente DDS, puente, nodos, bag).
- Con --ros-rates: frecuencia real de /scan, /pose (barridos que el SLAM llega a
  procesar), la salida del puente EV y lo que llega del FC. En el vuelo del
  2026-09-17 el SLAM proceso 1.2 de 9.9 barridos/s: esa es la cifra que decide.

    python3 sysmon.py --out run_sysmon.csv [--period 1.0] [--append-to run.log] [--ros-rates]
"""
import argparse, csv, os, re, signal, subprocess, sys, threading, time

import psutil

# Procesos que interesan: etiqueta -> patron en la linea de comandos.
PROCS = {
    "slam": r"slam_toolbox",
    "ldlidar": r"ldlidar",
    "xrce_agent": r"MicroXRCEAgent",
    "ev_bridge": r"ev_odometry_bridge",
    "nodo_vuelo": r"takeoff_position_hold|ev_offboard_handshake|offboard_position_handshake|offboard_control",
    "rosbag": r"ros2 bag record|rosbag2",
    "sensor_eval": r"flow_range_eval",
    "webui": r"webui/app\.py",
}
THROTTLE_BITS = {  # vcgencmd get_throttled (Raspberry Pi)
    0: "subtension", 1: "frecuencia_recortada", 2: "throttling", 3: "limite_termico",
}
ROS_TOPICS = [  # (topic, tipo, etiqueta)
    ("/scan", "sensor_msgs/msg/LaserScan", "scan_hz"),
    ("/pose", "geometry_msgs/msg/PoseWithCovarianceStamped", "slam_pose_hz"),
    ("/fmu/in/vehicle_visual_odometry", "px4_msgs/msg/VehicleOdometry", "ev_out_hz"),
    ("/fmu/out/vehicle_local_position", "px4_msgs/msg/VehicleLocalPosition", "fc_lpos_hz"),
]

stop = threading.Event()


def read_throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=1).stdout
        return int(out.strip().split("=")[1], 16)
    except Exception:
        return None


def read_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read()) / 1000.0
    except Exception:
        return None


def read_freq_mhz():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
            return int(f.read()) / 1000.0
    except Exception:
        return None


def max_freq_mhz():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq") as f:
            return int(f.read()) / 1000.0
    except Exception:
        return None


class RosRates:
    """Cuenta mensajes por topic con rclpy en un hilo. Si ROS no esta, no hace nada."""

    def __init__(self):
        self.counts = {lbl: 0 for _, _, lbl in ROS_TOPICS}
        self.ok = False
        self.lock = threading.Lock()
        try:
            import rclpy
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            from rosidl_runtime_py.utilities import get_message
            # Sin los manejadores de senal de rclpy: si no, SIGINT cierra ROS por
            # su cuenta, pisa el nuestro y no se llega a escribir el resumen.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self.rclpy = rclpy
            self.node = rclpy.create_node("sysmon_rates")
            for topic, typ, lbl in ROS_TOPICS:
                try:
                    msg_t = get_message(typ)
                except Exception:
                    continue
                self.node.create_subscription(msg_t, topic, self._cb(lbl), qos_profile_sensor_data)
            self.thread = threading.Thread(target=self._spin, daemon=True)
            self.thread.start()
            self.ok = True
        except Exception as e:  # sin ROS en el entorno: seguir sin tasas
            print(f"sysmon: sin tasas ROS ({e})", file=sys.stderr, flush=True)

    def _cb(self, lbl):
        def f(_msg):
            with self.lock:
                self.counts[lbl] += 1
        return f

    def _spin(self):
        try:
            while not stop.is_set() and self.rclpy.ok():
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception:
            pass  # ROS cerrado: las tasas simplemente dejan de contar

    def close(self):
        stop.set()
        self.thread.join(timeout=1.0)
        try:
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass

    def take(self):
        with self.lock:
            c = dict(self.counts)
            for k in self.counts:
                self.counts[k] = 0
        return c


def find_procs():
    found = {k: [] for k in PROCS}
    for p in psutil.process_iter(["pid", "cmdline"]):
        cmd = " ".join(p.info["cmdline"] or [])
        if not cmd or "sysmon.py" in cmd:
            continue
        for k, pat in PROCS.items():
            if re.search(pat, cmd):
                found[k].append(p)
                break
    return found


def pct(v, q):
    v = sorted(x for x in v if x is not None)
    if not v:
        return None
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]


def summarize(rows, ncores, fmax, period):
    out = []
    add = out.append
    if not rows:
        return "sysmon: sin muestras"
    dur = rows[-1]["t_rel"]
    add("=" * 64)
    add(f"CARGA DEL ORDENADOR DE A BORDO ({dur:.0f} s, {len(rows)} muestras, {ncores} nucleos, "
        f"max {fmax:.0f} MHz)" if fmax else f"CARGA DEL ORDENADOR ({dur:.0f} s, {len(rows)} muestras)")
    add("=" * 64)
    tot = [r["cpu_total"] for r in rows]
    top = [r["cpu_core_max"] for r in rows]
    sat = 100.0 * sum(1 for x in top if x >= 90) / len(top)
    add(f"CPU total:        media {sum(tot) / len(tot):5.1f}%  p95 {pct(tot, .95):5.1f}%  max {max(tot):5.1f}%")
    add(f"Nucleo mas cargado: media {sum(top) / len(top):5.1f}%  p95 {pct(top, .95):5.1f}%  "
        f"-> algun nucleo >= 90% el {sat:.0f}% del tiempo")
    temps = [r["temp_c"] for r in rows if r["temp_c"] is not None]
    if temps:
        hot = 100.0 * sum(1 for x in temps if x >= 80) / len(temps)
        add(f"Temperatura:      min {min(temps):.1f}  media {sum(temps) / len(temps):.1f}  max {max(temps):.1f} C"
            f"  (>= 80 C el {hot:.0f}% del tiempo; la Pi 4 recorta a ~80-85 C)")
    freqs = [r["freq_mhz"] for r in rows if r["freq_mhz"] is not None]
    if freqs:
        add(f"Frecuencia CPU:   min {min(freqs):.0f}  media {sum(freqs) / len(freqs):.0f} MHz")
    seen = set()
    for r in rows:
        th = r.get("throttled")
        if th is not None:
            for bit, name in THROTTLE_BITS.items():
                if th & (1 << bit):
                    seen.add(name)
    last_th = next((r["throttled"] for r in reversed(rows) if r.get("throttled") is not None), None)
    if last_th is not None:
        hist = {name for bit, name in THROTTLE_BITS.items() if last_th & (1 << (bit + 16))}
        add("Throttling (vcgencmd): " + ("NINGUNO" if not seen and not hist else
            f"DURANTE LA PRUEBA: {', '.join(sorted(seen)) or '-'}; desde el arranque: {', '.join(sorted(hist)) or '-'}"))
    mem = [r["mem_used_mb"] for r in rows]
    avail = [r["mem_avail_mb"] for r in rows]
    add(f"RAM:              usada max {max(mem):.0f} MB, disponible min {min(avail):.0f} MB, "
        f"swap max {max(r['swap_used_mb'] for r in rows):.0f} MB")
    add(f"Disco (escritura): media {sum(r['disk_w_mb_s'] for r in rows) / len(rows):.2f} MB/s, "
        f"max {max(r['disk_w_mb_s'] for r in rows):.2f} MB/s")
    add("Procesos (CPU en % de UN nucleo; 100 = un nucleo entero):")
    for k in PROCS:
        c = [r[f"{k}_cpu"] for r in rows if r.get(f"{k}_cpu") is not None]
        m = [r[f"{k}_rss_mb"] for r in rows if r.get(f"{k}_rss_mb") is not None]
        if c:
            add(f"  {k:12s} CPU media {sum(c) / len(c):6.1f}%  p95 {pct(c, .95):6.1f}%  max {max(c):6.1f}%   "
                f"RAM max {max(m):6.0f} MB")
    rates = {lbl: [r[lbl] for r in rows if r.get(lbl) is not None] for _, _, lbl in ROS_TOPICS}
    if any(rates.values()):
        add("Tasas ROS (Hz, media / min):")
        for lbl, v in rates.items():
            if v and max(v) > 0:
                add(f"  {lbl:12s} {sum(v) / len(v):6.1f} / {min(v):5.1f}")
        sc = [r["scan_hz"] for r in rows if r.get("scan_hz")]
        po = [r["slam_pose_hz"] for r in rows if r.get("scan_hz")]
        slam_running = any(r.get("slam_cpu") is not None for r in rows)
        if sc and sum(sc) > 0 and slam_running:
            add(f"  -> el SLAM procesa el {100.0 * sum(po) / sum(sc):.0f}% de los barridos que llegan")
    # Lectura para la decision de hardware
    add("Lectura:")
    slam_c = [r["slam_cpu"] for r in rows if r.get("slam_cpu") is not None]
    if slam_c and pct(slam_c, .95) >= 85:
        add("  - slam_toolbox pasa del 85% de un nucleo (p95): esta limitado por CPU en un solo hilo;"
            " un procesador con mas rendimiento POR NUCLEO subiria los barridos procesados.")
    if sat >= 25:
        add(f"  - algun nucleo va saturado el {sat:.0f}% del tiempo: poco margen para mas carga (planificador, 3D).")
    if temps and max(temps) >= 80:
        add("  - temperatura de recorte alcanzada: probar disipador/ventilador antes de concluir que falta CPU.")
    if "subtension" in seen:
        add("  - SUBTENSION durante la prueba: revisar el BEC/cable de 5 V antes que el ordenador.")
    if len(out) and out[-1] == "Lectura:":
        add("  - sin cuellos de botella claros en esta prueba.")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--period", type=float, default=1.0)
    ap.add_argument("--append-to", default="")
    ap.add_argument("--ros-rates", action="store_true")
    a = ap.parse_args()

    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stop.set())

    ncores = psutil.cpu_count()
    fmax = max_freq_mhz()
    ros = RosRates() if a.ros_rates else None
    psutil.cpu_percent(percpu=True)
    procs = {}
    disk0 = psutil.disk_io_counters()
    net0 = psutil.net_io_counters()
    t0 = time.time()
    last = t0
    rows = []

    cols = (["t", "t_rel", "cpu_total", "cpu_core_max"] + [f"cpu{i}" for i in range(ncores)] +
            ["load1", "freq_mhz", "temp_c", "throttled", "mem_used_mb", "mem_avail_mb", "swap_used_mb",
             "disk_w_mb_s", "net_tx_kb_s", "net_rx_kb_s"] +
            [f"{k}_{m}" for k in PROCS for m in ("cpu", "rss_mb")] + [lbl for _, _, lbl in ROS_TOPICS])
    f = open(a.out, "w", newline="")
    w = csv.DictWriter(f, fieldnames=cols)
    w.writeheader()

    while not stop.wait(a.period):
        now = time.time()
        dt = max(now - last, 1e-3)
        last = now
        per = psutil.cpu_percent(percpu=True)
        vm, sw = psutil.virtual_memory(), psutil.swap_memory()
        disk, net = psutil.disk_io_counters(), psutil.net_io_counters()
        r = {"t": round(now, 3), "t_rel": round(now - t0, 2), "cpu_total": round(sum(per) / len(per), 1),
             "cpu_core_max": max(per), "load1": os.getloadavg()[0], "freq_mhz": read_freq_mhz(),
             "temp_c": read_temp(), "throttled": read_throttled(),
             "mem_used_mb": round((vm.total - vm.available) / 2**20), "mem_avail_mb": round(vm.available / 2**20),
             "swap_used_mb": round(sw.used / 2**20),
             "disk_w_mb_s": round((disk.write_bytes - disk0.write_bytes) / 2**20 / dt, 3),
             "net_tx_kb_s": round((net.bytes_sent - net0.bytes_sent) / 1024 / dt, 1),
             "net_rx_kb_s": round((net.bytes_recv - net0.bytes_recv) / 1024 / dt, 1)}
        disk0, net0 = disk, net
        for i, v in enumerate(per):
            r[f"cpu{i}"] = v
        # Procesos: se buscan cada periodo (los del test arrancan despues que sysmon).
        for k, plist in find_procs().items():
            cpu = rss = 0.0
            measured = False
            for p in plist:
                if p.pid not in procs:
                    procs[p.pid] = p
                    try:
                        p.cpu_percent()  # la primera llamada solo ceba: no se registra un 0 falso
                    except psutil.Error:
                        pass
                    continue
                try:
                    cpu += procs[p.pid].cpu_percent()
                    rss += procs[p.pid].memory_info().rss / 2**20
                    measured = True
                except psutil.Error:
                    pass
            if measured:
                r[f"{k}_cpu"], r[f"{k}_rss_mb"] = round(cpu, 1), round(rss, 1)
        if ros and ros.ok:
            for lbl, c in ros.take().items():
                r[lbl] = round(c / dt, 2)
        rows.append(r)
        w.writerow(r)
        f.flush()

    f.close()
    summary = summarize(rows, ncores, fmax, a.period)
    with open(os.path.splitext(a.out)[0] + "_resumen.txt", "w") as g:
        g.write(summary + "\n")
    if a.append_to:
        try:
            with open(a.append_to, "a") as g:
                g.write("\n" + summary + "\n")
        except OSError:
            pass
    print(summary, flush=True)
    if ros and ros.ok:
        ros.close()


if __name__ == "__main__":
    main()
