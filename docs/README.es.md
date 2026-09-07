**Idiomas:** [English](../README.md) | [Español](README.es.md)

# px4_drone

🛸 Paquete ROS 2 (C++/Python) para **control Offboard de PX4** desde una
Raspberry Pi como companion computer — secuenciado de armado/modo,
verificación de failsafe, y vuelo despegue→hold→aterrizaje, hablando con el
controlador de vuelo por **uXRCE-DDS** sobre un enlace serial. Pensado para
funcionar con **múltiples versiones de firmware PX4** (v1.14 y v1.17) sin
tocar código C++.

> 📓 Diario de ingeniería complementario (reportes de vuelo, diagnóstico de
> `.ulog`, backups de firmware):
> [px4_drone_dev](https://github.com/Kmedrano101/px4_drone_dev)

## Contenido

- [Resumen](#resumen)
- [Instalación](#instalación)
- [Uso](#uso)
- [Recursos del proyecto](#recursos-del-proyecto)
- [Contribuir](#contribuir)
- [Licencia](#licencia)
- [Contacto](#contacto)

## Resumen

**Stack**
- 🚁 **FC:** PX4 — probado en v1.14 (`release/1.14`) y v1.17 (HKUST_NXT_DUAL)
- 💻 **Companion:** Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy
- 🔗 **Enlace:** Micro XRCE-DDS Agent, serial `/dev/ttyAMA0` @ 921600 baud
- 📷 **Cámara:** cámara oficial de Raspberry Pi vía `camera_ros`/`libcamera`

**Nodos**
| Nodo | Propósito |
|---|---|
| `offboard_control` | Arma en OFFBOARD (actitud, empuje cero, nunca despega), luego simula pérdida de heartbeat para verificar la reacción de failsafe del FC. |
| `takeoff_position_hold_indoor` | Espera un estimador de posición válido (flujo óptico/lidar), arma, despega, mantiene posición, aterriza — todo automático. |
| `takeoff_position_hold_outdoor` | Misma máquina de estados, condicionada a un fix GPS 3D en vez de flujo óptico. |
| `offboard_streamer.py` | Streamer mínimo en Python para pruebas de puesta a punto del enlace. |

**Hitos**
- ✅ Enlace Offboard RPi ⇄ FC establecido y diagnosticado de punta a punta (alineación de `px4_msgs`, versión de Fast-DDS/Fast-CDR de `MicroXRCEAgent`, dominio DDS)
- ✅ Test de failsafe (armar → hold → simular pérdida de heartbeat → desarme confirmado) verificado en **dos FC distintos**
- ✅ Soporte multi-firmware: cambiar de FC con un branch de `px4_msgs` + un parámetro de launch, sin tocar código
- ✅ Puesta a punto de la cámara oficial de Raspberry Pi (`camera_ros`, se arregló un bug de `libcamera` para el sensor OV5647)
- 🔄 Hold de posición indoor — bloqueado por tuning de EKF2/sensores, seguimiento en [px4_drone_dev](https://github.com/Kmedrano101/px4_drone_dev)
- ⏳ Primer test de vuelo con hold de posición GPS outdoor

## Instalación

`px4_msgs` viene vendorizado como **submodule de git, fijado al commit
exacto** con el que se compiló este paquete (`px4_msgs/`, actualmente el
commit del firmware v1.14 — ver [Uso](#uso) para cambiar de FC). Clonar con
`--recurse-submodules`, y symlinkearlo a nivel `src/` del workspace para que
`colcon` (que no recorre dentro de un paquete ya encontrado) lo descubra
junto a `px4_drone`:

```bash
cd ~/drone_ws/src
git clone --recurse-submodules https://github.com/Kmedrano101/px4_drone.git
ln -s px4_drone/px4_msgs px4_msgs

cd ~/drone_ws
colcon build --packages-select px4_msgs px4_drone --symlink-install
source install/setup.bash
```

También hace falta un [`MicroXRCEAgent`](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
compilado en la Raspberry Pi (ver [Recursos del proyecto](#recursos-del-proyecto)
para la versión/flags exactas que realmente interoperan con ROS 2 Jazzy — el
build default de `main` **no** funciona).

## Uso

```bash
# Verificacion de failsafe (seguro: nunca despega, empuje cero)
ros2 launch px4_drone offboard_control_cpp.launch.py

# Despegue + hold + aterrizaje completo — REQUIERE confirm_takeoff:=true (motores giran de verdad)
ros2 launch px4_drone takeoff_position_hold_indoor.launch.py confirm_takeoff:=true

# Camara de Raspberry Pi
ros2 launch px4_drone camera.launch.py
```

⚠️ **Seguridad:** cualquier nodo que arma el FC hace girar los motores,
aunque sea a empuje cero — asegurar o retirar las hélices antes de correr
`offboard_control`, y verificar el espacio físico antes de usar
`confirm_takeoff:=true`.

**Apuntar a otro firmware de FC** (default: v1.14, sin sufijo de versión de mensaje):

```bash
cd ~/drone_ws/src/px4_msgs && git checkout fc-v17-82e3322e   # o fc-v14-ffb6e80
cd ~/drone_ws && colcon build --packages-select px4_msgs px4_drone
ros2 launch px4_drone offboard_control_cpp.launch.py topic_version_suffix:=_v1
```

## Recursos del proyecto

**Documentación** (`docs/`)
- [`offboard_control` — comunicación RPi ⇄ FC](offboard_control.md) — arquitectura, flujo de mensajes, comportamiento de failsafe, y el log completo de troubleshooting (alineación de `px4_msgs`, build de `MicroXRCEAgent`, dominio DDS, setup multi-FC)

**Relacionado**
- [px4_drone_dev](https://github.com/Kmedrano101/px4_drone_dev) — reportes de vuelo, herramientas de análisis de `.ulog`, backups de firmware
- [PX4/px4_msgs](https://github.com/PX4/px4_msgs) — definiciones de mensajes (este repo depende de branches específicas según el firmware del FC, ver arriba)

## Contribuir

Proyecto personal de I+D — issues y sugerencias bienvenidas. Ver
[offboard_control.md](offboard_control.md) para la arquitectura actual y
las notas de troubleshooting abiertas antes de proponer cambios.

## Licencia

Distribuido bajo la [Apache License 2.0](../LICENSE).

## Contacto

Kevin Medrano — [kevin.ejem18@gmail.com](mailto:kevin.ejem18@gmail.com) · [@Kmedrano101](https://github.com/Kmedrano101)
