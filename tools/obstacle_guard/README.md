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
```

`ROS_DOMAIN_ID=77` y solo localhost: el nodo publica ARM de verdad, y así no puede llegar al dron.

## Limitaciones

- Frena y aterriza; **no rodea** el obstáculo ni retrocede.
- Solo ve lo que corta el **plano** del LiDAR: un obstáculo por encima o por debajo de ese plano
  (una barandilla baja, un cable) no existe para él.
- El rechazo del suelo necesita `dist_bottom_valid`, que solo es true en vuelo. En tierra, con el
  dron nivelado, no hace falta.
