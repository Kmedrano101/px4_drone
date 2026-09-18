# Parada por obstáculo con el LiDAR 2D

`takeoff_position_hold_ev` (despegue/hold/aterrizaje y patrón en cruz) vigila `/scan` en 360°
con `obstacle_stop_distance_m` (defecto **1.0 m**, `0` = desactivada):

- **Antes de armar:** si hay algo más cerca que esa distancia, **no arma**.
- **En vuelo:** si un obstáculo se acerca tanto que hay que empezar a frenar para quedar parado a
  esa distancia, **frena** (velocidad horizontal 0, posición horizontal en NaN, altura mantenida)
  `obstacle_brake_seconds` (2 s) y **aterriza**. Si el piloto ya tomó el control, no toca nada.
- **Sin `/scan`** durante más de 0.5 s, o con menos del 10% de rayos válidos: se trata como
  obstáculo, porque no se puede garantizar la distancia.

Usa **solo los barridos crudos**: ni el SLAM ni la posición del EKF. En el vuelo del 2026-09-17 el
SLAM perdió el tracking y dijo que el dron estaba quieto mientras recorría ~3 m, y el EKF se lo creyó;
lo único que vio el obstáculo acercarse fue `/scan`. Detalle del algoritmo en
`include/px4_drone/obstacle_guard.hpp`.

Umbral de frenada por sector (36 sectores de 10°):
`stop_distance + v·0.35 s + v²/(2·2.0 m/s²)`, con `v` = velocidad de acercamiento medida en los
barridos. A 1.2 m/s dispara a ~1.8 m para quedar a ~1.0 m.

## Parámetros del nodo

| Parámetro | Defecto | |
|---|---|---|
| `obstacle_stop_distance_m` | 1.0 | 0 = desactivada; si no, ≥ 0.3 |
| `obstacle_brake_seconds` | 2.0 | frenado antes de pedir LAND |
| `obstacle_brake_decel_m_s2` | 2.0 | frenada supuesta (`MPC_ACC_HOR` = 3) |
| `obstacle_min_valid_range_m` | 0.15 | por debajo: el propio dron o ruido |
| `obstacle_scan_timeout_s` | 0.5 | sin `/scan` más tiempo = parar |
| `laser_yaw_offset_deg` | 0.0 | giro del LiDAR respecto al morro; debe coincidir con la TF `base_link → base_laser` |
| `laser_height_above_range_m` | 0.134 | plano del LiDAR sobre el LiDAR 1D (0.114 FC→LiDAR 2D + 0.020 LiDAR 1D bajo el FC), para descartar el suelo al inclinarse |

## Validación 1: barridos reales grabados

```bash
pip install mcap mcap-ros2-support
python3 bag_to_scans.py <bag_0.mcap> [t0_epoch] > scans.csv   # sirve con bags truncados
~/drone_ws/install/px4_drone/lib/px4_drone/obstacle_guard_replay 1.0 < scans.csv
```

Con el bag del accidente (`cross_pattern_1789649199_bag`, t0 = inicio del tramo "adelante"):
**ninguna parada en el despegue ni en el hover**, y **parada a t = 3.16 s** con el obstáculo a
1.65 m, acercándose a 1.27 m/s. El piloto intervino a los 3.54 s, y el golpe fue a los ~4 s.

Una primera versión daba paradas falsas en hover: cuando el obstáculo más cercano de un sector
cambiaba de una pared a otra, el salto contaba como velocidad. Ahora un cambio más rápido de lo que
el dron puede moverse reinicia el sector, y la velocidad sale de una regresión sobre la ventana.

## Validación 2: el nodo completo contra un FC falso

```bash
./run_scenario.sh preflight   # obstáculo a 0.6 m: NO arma
./run_scenario.sh approach    # pared a 1.2 m/s: frena a ~1.56 m y aterriza
./run_scenario.sh scanloss    # se corta /scan: frena a los 0.5 s y aterriza
./run_scenario.sh slamlost    # la sigma del SLAM salta a 0.45 m en vuelo: aterriza
./run_scenario.sh userhold    # HOLD en pleno tramo de la cruz: frena, se queda quieto; ATERRIZAR: aterriza
./run_scenario.sh cancel      # HOLD antes de armar: cancela sin armar
```

## HOLD y ATERRIZAR desde la webui

En las pruebas con despegue, la webui muestra **HOLD** y **ATERRIZAR**. Mandan SIGUSR1 / SIGUSR2
al ejecutable del nodo, buscado dentro del grupo de procesos de la prueba. No se envían a
`ros2 launch`: matar el envoltorio dejaría el nodo volando solo.

- **HOLD:** frena 1 s (velocidad horizontal 0) y después mantiene la posición donde quedó, con todas
  las protecciones activas (obstáculo, SLAM, control perdido). Aterriza solo pasados
  `user_hold_timeout_s` (120 s).
- **ATERRIZAR:** LAND, solo si el nodo aún tiene el control.
- **Antes de armar:** cualquiera de los dos cancela la prueba sin armar.
- **MATAR procesos** (el STOP de antes) corta todo, incluido el enlace con el FC. En pruebas de vuelo
  pide confirmación, porque en el aire dispara el failsafe de Offboard.

## Salud del SLAM y varianza del EV (mismo día)

- `takeoff_position_hold_ev` tampoco despega, y en vuelo aterriza, si `/pose` de slam_toolbox declara
  **sigma > `slam_max_sigma_m` (0.3 m)** o no llega en **`slam_pose_timeout_s` (2.5 s)**.
  En el bag del accidente: hover 0.06-0.10 m, perdido 0.41-0.48 m (desde t = 2.1 s). En vuelo hubo
  huecos entre poses de hasta 1.2 s, y en tierra de 2.4 s: por eso el plazo es 2.5 s.
- `ev_odometry_bridge` sigue a 20 Hz (el EKF deja el EV si pasan >200 ms entre muestras,
  `EV_MAX_INTERVAL`), pero ahora cada muestra lleva **varianza = la del SLAM + (antigüedad de la pose
  × `ev_max_speed_m_s`)²**: una pose recién calculada pesa (sigma ~0.13 m) y sus repeticiones cada
  vez menos (1.3 m a los 0.9 s), así que entre poses manda el flujo óptico. z, roll y pitch van con
  1e4, finitos: EKF2 solo usa las varianzas si las tres son finitas. Comprobación:
  `../ev_bridge/check_variance.sh`. Si se baja `MPC_XY_VEL_MAX`, conviene bajar `ev_max_speed_m_s`
  a algo por encima (defecto 1.5 m/s).

`ROS_DOMAIN_ID=77` y solo localhost: el nodo publica ARM de verdad, y así no puede llegar al dron.

## Limitaciones

- Frena y aterriza; **no rodea** el obstáculo ni retrocede.
- Solo ve lo que corta el **plano** del LiDAR: un obstáculo por encima o por debajo de ese plano
  (una barandilla baja, un cable) no existe para él.
- El rechazo del suelo necesita `dist_bottom_valid`, que solo es true en vuelo. En tierra, con el
  dron nivelado, no hace falta.
