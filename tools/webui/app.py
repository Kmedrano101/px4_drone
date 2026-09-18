#!/usr/bin/env python3
"""Webapp basica para lanzar los tests de px4_drone desde el celular en
campo (via el AP de wlan0, ver tools/field-ap/). Un solo test corre a la
vez -- dos MicroXRCEAgent peleando por /dev/ttyAMA0 es un modo de fallo ya
documentado en este proyecto.

topic_version_suffix queda fijo en "" (firmware v1.14, el que esta
compilado ahora en px4_msgs). Cambiar de firmware requiere recompilar
px4_msgs con la otra branch a mano -- no es un toggle seguro de exponer
en runtime, ver docs/offboard_control.md seccion 11.

Cada test lanzado graba un ros2 bag (-a, todos los topics visibles) en
paralelo, guardado en logs/<test_id>_<timestamp>_bag/. Sirve para analizar
despues que recibio/publico exactamente la Pi (complementa el .ulog del FC,
que no muestra ese lado -- ver docs/offboard_control.md seccion 10 para el
historial de bugs de comunicacion de este proyecto).
"""
import os
import shlex
import signal
import subprocess
import threading
import time

from flask import Flask, jsonify, request, render_template

WORKSPACE = "/home/kevin/drone_ws"
EXTRA_WS = "/home/kevin/evarobot_ws"
TOPIC_VERSION_SUFFIX = ""  # firmware v1.14, ver docstring arriba

# Pruebas que no son un ros2 launch sino un script suelto (hoy: la evaluacion
# de sensores por MAVLink). Corren con el venv que tiene pymavlink, sin sourcear
# ROS y sin grabar ros2 bag. Leen el FC por su USB-C conectado a la Pi.
SCRIPT_PY = "/home/kevin/.venvs/mav/bin/python"
TOOLS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

app = Flask(__name__)

TESTS = {
    "failsafe": {
        "label": "Failsafe (sin despegue)",
        "desc": "Arma en actitud, empuje cero, simula perdida de heartbeat y verifica que el FC reacciona (failsafe/desarme). Nunca despega.",
        "launch_file": "offboard_control_cpp.launch.py",
        "needs_confirm_takeoff": False,
        "params": [],
    },
    "handshake": {
        "label": "Handshake / kill switch",
        "desc": "Arma en modo posicion (hold fijo, sin despegue) y se queda armado la ventana indicada -- usalo para probar el kill switch del RC. Si nadie lo corta, se desarma solo al final.",
        "launch_file": "offboard_position_handshake.launch.py",
        "needs_confirm_takeoff": False,
        "params": [
            {"key": "hold_seconds", "label": "Segundos armado", "default": 30.0, "min": 3, "max": 300, "step": 1},
        ],
    },
    "indoor": {
        "label": "Despegue + hold indoor",
        "desc": "Despega, mantiene posicion y aterriza solo. Requiere flujo optico/lidar valido. VUELO REAL.",
        "launch_file": "takeoff_position_hold_indoor.launch.py",
        "needs_confirm_takeoff": True,
        "params": [
            {"key": "takeoff_height_m", "label": "Altura (m)", "default": 1.0, "min": 0.3, "max": 5.0, "step": 0.1},
            {"key": "hold_seconds", "label": "Hold (s)", "default": 8.0, "min": 2, "max": 120, "step": 1},
        ],
    },
    "outdoor": {
        "label": "Despegue + hold outdoor",
        "desc": "Despega, mantiene posicion y aterriza solo. Requiere fix GPS 3D. VUELO REAL.",
        "launch_file": "takeoff_position_hold_outdoor.launch.py",
        "needs_confirm_takeoff": True,
        "params": [
            {"key": "takeoff_height_m", "label": "Altura (m)", "default": 2.0, "min": 0.5, "max": 10.0, "step": 0.1},
            {"key": "hold_seconds", "label": "Hold (s)", "default": 10.0, "min": 2, "max": 120, "step": 1},
        ],
    },
    "cross_pattern": {
        "label": "Patrón cruz LiDAR 2D (4 direcciones)",
        "desc": "Lanza LD19 + slam_toolbox + puente EV. Despega a 1 m con LiDAR 1D + baro, recorre adelante/atras/izquierda/derecha volviendo al centro en cada tramo con SLAM 2D, y aterriza solo. Techo 1.2 m. VUELO REAL (espacio libre minimo 3x3 m).",
        "launch_file": "cross_pattern_ev.launch.py",
        "needs_confirm_takeoff": True,
        "params": [
            {"key": "takeoff_height_m", "label": "Altura (m)", "default": 1.0, "min": 0.3, "max": 1.2, "step": 0.1},
            {"key": "hold_seconds", "label": "Hold antes del patron (s)", "default": 5.0, "min": 2, "max": 60, "step": 1},
            {"key": "pattern_distance_m", "label": "Distancia por tramo (m)", "default": 1.0, "min": 0.2, "max": 2.0, "step": 0.1},
            {"key": "pattern_settle_seconds", "label": "Pausa en cada punto (s)", "default": 3.0, "min": 1, "max": 15, "step": 0.5},
        ],
    },
    "ev_handshake": {
        "label": "EV handshake (LiDAR 2D, sin despegue)",
        "desc": "Lanza LD19 + slam_toolbox + el puente EV, y arma en OFFBOARD exigiendo el LiDAR 2D (cs_ev_pos/cs_ev_yaw) sano -- mantiene el origen en el piso, sin despegar. Necesita EKF2_EV_CTRL=9 en el FC.",
        "launch_file": "ev_offboard_handshake.launch.py",
        "needs_confirm_takeoff": False,
        "params": [
            {"key": "hold_seconds", "label": "Segundos armado", "default": 10.0, "min": 3, "max": 300, "step": 1},
        ],
    },
    "sensor_eval": {
        "label": "Evaluar flujo optico + LiDAR 1D (a mano)",
        "desc": "Dron DESARMADO en la mano, cable USB-C del FC a la Pi. Mide con cinta la altura de la lente del sensor al suelo. Fase quieto: ruido y sesgo del LiDAR, deriva del flujo. Fase mover: lleva el dron hacia el morro la distancia indicada por la cinta, nivelado y a la misma altura. Repite a varias alturas para sacar la curva de error. Solo lectura: no arma ni cambia parametros.",
        "script": os.path.join(TOOLS_DIR, "sensor_eval", "flow_range_eval.py"),
        "record_bag": False,
        "needs_confirm_takeoff": False,
        "params": [
            {"key": "gt_height_m", "cli": "--gt-height", "label": "Altura real sensor-suelo (m, cinta)", "default": 1.0, "min": 0.1, "max": 6.0, "step": 0.01},
            {"key": "gt_distance_m", "cli": "--gt-distance", "label": "Recorrido hacia el morro (m, 0 = solo quieto)", "default": 1.0, "min": 0.0, "max": 5.0, "step": 0.05},
            {"key": "still_s", "cli": "--still", "label": "Fase quieto (s)", "default": 6.0, "min": 3, "max": 30, "step": 1},
            {"key": "duration_s", "cli": "--duration", "label": "Duracion total (s)", "default": 25.0, "min": 8, "max": 120, "step": 1},
        ],
    },
    "ev_takeoff": {
        "label": "Ciclo completo LiDAR 2D + 1D",
        "desc": "Lanza LD19 + slam_toolbox + el puente EV, despega, mantiene posicion y aterriza usando el LiDAR 2D (SLAM/EV) para posicion/yaw y el LiDAR 1D + baro para altura. Techo de 1.2 m. Necesita EKF2_EV_CTRL=9 en el FC. VUELO REAL.",
        "launch_file": "takeoff_position_hold_ev.launch.py",
        "needs_confirm_takeoff": True,
        "params": [
            {"key": "takeoff_height_m", "label": "Altura (m)", "default": 1.0, "min": 0.3, "max": 1.2, "step": 0.1},
            {"key": "hold_seconds", "label": "Hold (s)", "default": 5.0, "min": 2, "max": 60, "step": 1},
        ],
    },
}

LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")
os.makedirs(LOG_DIR, exist_ok=True)

_lock = threading.Lock()
_state = {
    "test_id": None,
    "proc": None,
    "log_path": None,
    "started_at": None,
    "bag_proc": None,
    "bag_path": None,
}


def _stop_bag_locked():
    """Para la grabacion de ros2 bag del test actual, si hay una corriendo.
    Se llama tanto desde /api/stop como cuando /api/status detecta que el
    test principal ya termino solo -- grabar despues de eso no sirve de
    nada y deja el proceso de bag colgado."""
    bag_proc = _state["bag_proc"]
    if bag_proc is not None and bag_proc.poll() is None:
        try:
            os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
            bag_proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(bag_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    _state["bag_proc"] = None


def _build_script_command(test):
    args = [SCRIPT_PY, "-u", test["script"]]
    for p in test["params"]:
        value = float(request.json.get(p["key"], p["default"]))  # nunca texto del cliente
        if not (p["min"] <= value <= p["max"]):
            raise ValueError(f"{p['key']} fuera de rango")
        args += [p["cli"], f"{value}"]
    args += ["--out-dir", os.path.join(LOG_DIR, "sensor_eval")]
    return args


def _build_command(test):
    if "script" in test:
        return _build_script_command(test)
    args = ["ros2", "launch", "px4_drone", test["launch_file"]]
    # ros2 launch rechaza "clave:=" con valor vacio (formato invalido) -- el
    # launch file ya trae "" como default, asi que si no hay sufijo
    # simplemente no se pasa el argumento.
    if TOPIC_VERSION_SUFFIX:
        args.append(f"topic_version_suffix:={TOPIC_VERSION_SUFFIX}")
    for p in test["params"]:
        value = request.json.get(p["key"], p["default"])
        value = float(value)  # nunca interpolar texto del cliente sin validar
        args.append(f"{p['key']}:={value}")
    if test["needs_confirm_takeoff"]:
        args.append("confirm_takeoff:=true")
    return args


@app.route("/")
def index():
    return render_template("index.html", tests=TESTS)


@app.route("/api/status")
def status():
    with _lock:
        running = _state["proc"] is not None and _state["proc"].poll() is None
        bag_path = _state["bag_path"]
        if not running and _state["proc"] is not None:
            # el proceso termino solo (nodo llego a kFinished y ros2 launch
            # cerro): parar tambien la grabacion (seguir grabando despues
            # de esto no aporta nada) y liberar el slot para el proximo test
            _stop_bag_locked()
            _state["test_id"] = None
            _state["proc"] = None
            _state["started_at"] = None
            _state["bag_path"] = None

        log_tail = ""
        if _state["log_path"] and os.path.exists(_state["log_path"]):
            with open(_state["log_path"], "r", errors="replace") as f:
                lines = f.readlines()
                log_tail = "".join(lines[-120:])

        return jsonify({
            "running": running,
            "test_id": _state["test_id"],
            "elapsed_s": (time.time() - _state["started_at"]) if _state["started_at"] else 0,
            "log_tail": log_tail,
            "bag_path": bag_path,
        })


def _start_bag(run_id):
    bag_path = os.path.join(LOG_DIR, f"{run_id}_bag")
    bag_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {EXTRA_WS}/install/setup.bash && "
        f"source {WORKSPACE}/install/setup.bash && "
        f"export ROS_DOMAIN_ID=0 && "
        f"ros2 bag record -a -o {shlex.quote(bag_path)}"
    )
    bag_log = open(os.path.join(LOG_DIR, f"{run_id}_bag.log"), "w")
    bag_proc = subprocess.Popen(
        ["bash", "-c", bag_cmd],
        stdout=bag_log, stderr=subprocess.STDOUT,
        cwd=WORKSPACE, preexec_fn=os.setsid,
    )
    return bag_path, bag_proc


@app.route("/api/launch", methods=["POST"])
def launch():
    body = request.json or {}
    test_id = body.get("test_id")
    if test_id not in TESTS:
        return jsonify({"error": "test_id desconocido"}), 400
    test = TESTS[test_id]

    if test["needs_confirm_takeoff"] and not body.get("confirm_takeoff"):
        return jsonify({"error": "Este test despega de verdad: falta confirmar confirm_takeoff"}), 400

    with _lock:
        if _state["proc"] is not None and _state["proc"].poll() is None:
            return jsonify({"error": f"Ya hay un test corriendo: {_state['test_id']}"}), 409

        try:
            cmd_args = _build_command(test)
        except (TypeError, ValueError):
            return jsonify({"error": "Parametro invalido (numerico y dentro del rango permitido)"}), 400

        cmd_str = " ".join(shlex.quote(a) for a in cmd_args)
        if "script" in test:
            full_cmd = cmd_str
        else:
            full_cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {EXTRA_WS}/install/setup.bash && "
                f"source {WORKSPACE}/install/setup.bash && "
                f"export ROS_DOMAIN_ID=0 && "
                f"{cmd_str}"
            )

        run_id = f"{test_id}_{int(time.time())}"
        log_path = os.path.join(LOG_DIR, f"{run_id}.log")
        log_file = open(log_path, "w")
        proc = subprocess.Popen(
            ["bash", "-c", full_cmd],
            stdout=log_file, stderr=subprocess.STDOUT,
            cwd=WORKSPACE, preexec_fn=os.setsid,
        )

        # ros2 bag record -a: graba todos los topics visibles (incluye
        # /fmu/out/*, /fmu/in/*, /rosout, etc.) en paralelo al test, para
        # poder analizar despues que exactamente recibio/publico la Pi --
        # el .ulog del FC no muestra ese lado (ver docs/offboard_control.md
        # seccion 10 para los bugs de comunicacion que ya nos costaron caro
        # en este proyecto).
        bag_path = None
        bag_proc = None
        if test.get("record_bag", True):
            bag_path, bag_proc = _start_bag(run_id)

        _state["test_id"] = test_id
        _state["proc"] = proc
        _state["log_path"] = log_path
        _state["started_at"] = time.time()
        _state["bag_proc"] = bag_proc
        _state["bag_path"] = bag_path

        return jsonify({"ok": True, "test_id": test_id, "cmd": cmd_str, "bag_path": bag_path})


@app.route("/api/stop", methods=["POST"])
def stop():
    with _lock:
        proc = _state["proc"]
        if proc is None or proc.poll() is not None:
            return jsonify({"ok": True, "note": "no habia nada corriendo"})

        pgid = os.getpgid(proc.pid)
        try:
            os.killpg(pgid, signal.SIGINT)
            proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        # por si queda un agente huerfano fuera del process group (ya paso
        # en este proyecto, ver docs/offboard_control.md)
        subprocess.run(["pkill", "-f", "MicroXRCEAgent"], check=False)

        _stop_bag_locked()
        _state["test_id"] = None
        _state["proc"] = None
        _state["started_at"] = None
        _state["bag_path"] = None
        return jsonify({"ok": True})


# --- Red: AP de campo (Drone-UAS) <-> wifi cliente del laboratorio ----------
# Los scripts de tools/field-ap/ ya hacen todo el trabajo (parar o arrancar
# netplan-wpa-wlan0, hostapd y dnsmasq). Aqui solo se invocan. Necesitan root:
# hay una regla de sudoers NOPASSWD limitada a esos dos scripts exactos.
_FIELD_AP_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "field-ap"))
NET_SCRIPTS = {
    "ap": os.path.join(_FIELD_AP_DIR, "enable-field-ap.sh"),
    "wifi": os.path.join(_FIELD_AP_DIR, "disable-field-ap.sh"),
}


def _net_mode():
    """Estado real de wlan0, preguntado a la interfaz y no a una variable
    guardada: si alguien cambia la red por ssh, la pagina se entera igual."""
    try:
        out = subprocess.run(["iw", "dev", "wlan0", "info"],
                             capture_output=True, text=True, timeout=5).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return {"mode": "desconocido", "ssid": None}
    if "type AP" in out:
        mode = "ap"
    elif "type managed" in out:
        mode = "wifi"
    else:
        mode = "desconocido"
    ssid = None
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("ssid "):
            ssid = line[5:].strip()
    return {"mode": mode, "ssid": ssid}


@app.route("/api/network")
def network_status():
    return jsonify(_net_mode())


@app.route("/api/network", methods=["POST"])
def network_switch():
    mode = (request.json or {}).get("mode")
    if mode not in NET_SCRIPTS:
        return jsonify({"ok": False, "error": "modo invalido"}), 400

    with _lock:
        # cambiar de red mata el enlace con el FC y el bag a medias: mismo
        # criterio que el resto de la app, un solo cambio de estado a la vez
        if _state["proc"] is not None and _state["proc"].poll() is None:
            return jsonify({"ok": False,
                            "error": "hay un test corriendo, paralo antes"}), 409

    script = NET_SCRIPTS[mode]

    def _switch():
        subprocess.run(["sudo", "-n", script], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # en segundo plano y con retraso a proposito: el cambio corta la red del
    # cliente, asi que la respuesta HTTP tiene que salir antes del corte
    threading.Timer(1.0, _switch).start()
    return jsonify({"ok": True, "mode": mode})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
