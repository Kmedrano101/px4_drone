# Ciclos PID/MPC: zona ciega y salud SLAM (2026-10-20)

Zona ciega circular de 70 cm de diametro centrada en el LiDAR: se ignoran
retornos r <= 0.35 m en los 360 grados. Sustituye la mascara angular trasera.
Parametro: obstacle_blind_radius_m=0.35; 0 desactiva la mascara.
La distancia de parada sigue midiendose desde el LiDAR.

Los launches takeoff_position_hold_ev y takeoff_hold_land_mpc usan
slam_health_enabled=false por defecto para las pruebas de ciclo.
No bloquean ni abortan por sigma/antiguedad/ausencia de /pose; mantienen
las comprobaciones de posicion del FC y el funcionamiento del SLAM/puente EV.
slam_health_enabled:=true restaura esa guarda. La clase conserva true por
defecto para otros nodos.

Actualizacion posterior solicitada por el usuario: guarda de obstaculos desde
TAKEOFF y chequeo previo al armado restaurados. Exige /scan utilizable y espacio
libre antes de ARM; durante el ascenso frena y pide LAND si dispara. La zona
ciega circular y slam_health_enabled=false se mantienen.

Verificacion: 288 casos C++ del filtro circular y evaluacion de ambos launches
con guarda SLAM false/true, sin ejecutar nodos ni iniciar vuelos.
Compilacion: ver /tmp/pid-mpc-review-final-build.log.
Cambios locales, sin commit ni parametros escritos al FC.

Compilacion de esta actualizacion: /tmp/obstacle-takeoff-build.log.

## Ampliacion: la zona ciega sola no bastaba (mismo dia)

Analisis de los rosbags del 2026-10-16 con obstacle_guard_replay:

- Con hélices paradas (corrida rechazada antes de armar, 655 barridos / 66 s) el
  LD19 no da **ningun** retorno por debajo de 2 m. Nada del dron esta dentro del
  plano de barrido en reposo.
- Con hélices girando aparece una rafaga de retornos espurios de 0.6 s (MPC) a
  1.6 s (PID), repartida por muchos rumbos a la vez y con distancias que saltan
  entre barridos. Es lo que disparo las dos paradas; no habia obstaculo (los
  unicos son 4 pilares a 3 m, con el dron en el centro).
- Zona ciega 0.35 m sola: salva el MPC, pero el PID habria abortado igual con un
  retorno a 0.58 m.

Por eso se anade **obstacle_debounce_scans**, con 3 por defecto en el nodo y en los
dos launches (antes 2, fijo en el codigo). La clase C++ conserva 2 por defecto.

| Zona ciega | Debounce | MPC 16-oct | PID 16-oct | Accidente 18-sep |
|---|---|---|---|---|
| 0.00 m | 2 | PARA 0.95 m | PARA 0.16 m | PARA 1.65 m |
| 0.35 m | 2 | libre | PARA 0.58 m | PARA 1.65 m |
| 0.35 m | 3 | libre | libre | PARA 1.51 m |

Descartados tras probarlos: filtro por rayos pegados a range_min (el bag del
accidente tiene un 26 % de barridos asi, con obstaculo real) y debounce por
sector (mismo resultado a debounce 3 y menos sensible al guiñar).

No se activa la guarda solo despues del despegue: el chequeo previo al armado se
hace con las helices paradas, que es cuando el barrido esta limpio, y apagarla en
el ascenso dejaria el dron sin proteccion en la fase mas baja para arreglar algo
que ya esta corregido en su origen.

Analisis completo: px4_drone_dev/reports/2026-10-20_falsos-obstaculos-al-despegar.md
