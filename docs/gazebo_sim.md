# Rama `gazebo-sim` — el mismo control, contra Gazebo Sim

Esta rama replica lo que hacen los nodos en el dron real, pero apuntando al
simulador. **El código C++ y Python es el mismo que en `main`**: solo cambian el
transporte del enlace, el bring-up y algún valor por defecto.

## Qué cambia respecto a `main`

| | `main` (dron real) | `gazebo-sim` |
|---|---|---|
| Agente uXRCE-DDS | `serial --dev /dev/ttyAMA0 -b 921600` | `udp4 -p 8888` |
| Firmware del FC | PX4 1.14.3 (FC#2) | PX4 `main` (SITL) |
| `px4_msgs` | `release/1.14` | `release/1.17` |
| `topic_version_suffix` | `""` | **`"_v1"`** |
| Bring-up | agente en la RPi | `sitl.launch.py` levanta Gazebo + PX4 + agente |
| Cámara | Raspberry Pi (`camera_ros`) | no aplica |

**No hay cambios en `src/` ni en `include/`.** Se comprobó que los 6 mensajes que
usa el paquete (`VehicleStatus`, `VehicleLocalPosition`, `TrajectorySetpoint`,
`OffboardControlMode`, `VehicleCommand`, `VehicleCommandAck`) tienen **campos
idénticos** entre PX4 `main` y `px4_msgs release/1.17`, y que las **23 constantes**
que el código referencia valen lo mismo en ambos. Solo difieren constantes que el
paquete no usa (`ARM_DISARM_REASON_*`), y esas no viajan por el cable.

## Uso

Una vez por instalación, dejar el modelo y el airframe en PX4-Autopilot:

```bash
~/src/drone_tunning/gazebo/sync_px4.sh restore
cd ~/PX4-Autopilot && make px4_sitl_default
```

Luego, en dos terminales:

```bash
# 1) simulador + agente
ros2 launch px4_drone sitl.launch.py

# 2) el nodo de control (sin levantar un segundo agente)
ros2 launch px4_drone takeoff_position_hold_indoor.launch.py \
     confirm_takeoff:=true run_agent:=false

# PX4 publica en BEST_EFFORT: para inspeccionar a mano hay que pedirlo
ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_local_position_v1
```

Argumentos de `sitl.launch.py`:

| Argumento | Por defecto | Para qué |
|---|---|---|
| `world` | `indoor_zone` | La zona de pruebas escaneada |
| `model` | `rjx_f450_indoor` | RJX F450 + MTF-01P + LDROBOT D500 |
| `headless` | `false` | Ver el aviso de abajo |
| `camera_follow` | `false` | `true` fija la cámara y **anula el zoom** |
| `px4_dir` | `~/PX4-Autopilot` | Ruta al clon de PX4 |

## Cuatro trampas del simulador

**1. No usar `headless:=true` en máquinas NVIDIA.** El servidor de Gazebo falla al
crear el contexto EGL y **los sensores GPU —los dos LiDAR y el flujo óptico— dejan
de publicar sin dar ningún error**. El modelo aparece y los motores giran, pero el
EKF no recibe nada.

**2. Los topics del SITL llevan sufijo `_v1`, y `dds_topics.yaml` engaña.** Ese
fichero lista los nombres **sin** sufijo, pero el cliente uXRCE-DDS lo añade en
runtime según el `MESSAGE_VERSION` del mensaje. En este SITL son
`/fmu/out/vehicle_status_v1` y `/fmu/out/vehicle_local_position_v1`; los de entrada
(`offboard_control_mode`, `trajectory_setpoint`, `vehicle_command`) y
`vehicle_command_ack` van sin sufijo. Por eso el default de esta rama es `_v1`.
Un nombre equivocado **no da error, da silencio**. Comprobar siempre con:

```bash
ros2 topic list | grep -E "vehicle_local_position|vehicle_status"
```

**3. Sin GPS no se puede usar Takeoff ni ningún modo AUTO_*.** Exigen posición
global. El vehículo arranca en `AUTO_LOITER` y el armado se rechaza con *"Resolve
system health failures first"*, sin decir por qué. La vía indoor es **Offboard**,
que solo pide posición local — la que el EKF saca del flujo óptico y el láser. Es
justo lo que hacen `offboard_control` y `takeoff_position_hold_indoor`.

**4. No hay RC.** PX4 exige mando salvo que se cambie `COM_RC_IN_MODE`. El airframe
`4023_gz_rjx_f450_indoor` no lo toca; si el armado se queja de
`manual_control_signal_lost`, poner en la consola de PX4:

```
param set COM_RC_IN_MODE 4
```

## Diferencias que la simulación NO reproduce

Importante antes de sacar conclusiones de un vuelo simulado:

- **En la sim hay magnetómetro y es perfecto; el dron real no tiene ninguno.** El
  bloqueo real del hold de posición indoor es la falta de yaw. Para reproducirlo:
  `param set SYS_HAS_MAG 0` y `param set EKF2_MAG_TYPE 5`.
- **El láser del MTF-01P es lineal hasta 12 m en la sim.** El real satura por
  encima de ~1.5 m (a 3 m lee 2.14) y nunca reporta baja confianza.

Detalle completo en
[`drone_tunning/docs/SIMULACION_GAZEBO_INDOOR.md`](https://github.com/Kmedrano101/px4_drone_dev).
