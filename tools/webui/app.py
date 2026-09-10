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
TOPIC_VERSION_SUFFIX = ""  # firmware v1.14, ver docstring arriba

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
        "label": "Patron cruz (4 direcciones)",
        "desc": "Despega, hold, recorre adelante/atras/izquierda/derecha (cuerpo del dron) volviendo al centro entre cada tramo, aterriza. VUELO REAL -- empezar con 0.5 m, ver espacio libre necesario en el launch file.",
        "launch_file": "cross_pattern_indoor.launch.py",
        "needs_confirm_takeoff": True,
        "params": [
            {"key": "takeoff_height_m", "label": "Altura (m)", "default": 1.0, "min": 0.3, "max": 5.0, "step": 0.1},
            {"key": "hold_seconds", "label": "Hold antes del patron (s)", "default": 8.0, "min": 2, "max": 120, "step": 1},
            {"key": "pattern_distance_m", "label": "Distancia por tramo (m)", "default": 0.5, "min": 0.0, "max": 3.0, "step": 0.1},
            {"key": "pattern_settle_seconds", "label": "Pausa en cada punto (s)", "default": 4.0, "min": 1, "max": 30, "step": 0.5},
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


def _build_command(test):
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
            return jsonify({"error": "Parametro invalido (debe ser numerico)"}), 400

        cmd_str = " ".join(shlex.quote(a) for a in cmd_args)
        full_cmd = (
            f"source /opt/ros/jazzy/setup.bash && "
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
        bag_path = os.path.join(LOG_DIR, f"{run_id}_bag")
        bag_cmd = (
            f"source /opt/ros/jazzy/setup.bash && "
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


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
