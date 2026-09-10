# Herramientas de campo

Setup para probar los nodos de `px4_drone` en campo desde el celular, sin
laptop: la Pi genera su propio AP wifi y sirve una webapp con botones para
cada test.

## 1. AP de campo (`field-ap/`)

Wifi propio de la Pi (`wlan0`), independiente del wifi del lab. **No** se
activa solo -- es un toggle manual, para no perder la conexión normal del
lab sin querer.

```bash
sudo apt install -y hostapd dnsmasq iw
sudo systemctl unmask hostapd
sudo systemctl disable hostapd dnsmasq   # el toggle los prende/apaga a mano

sudo tools/field-ap/install.sh           # copia las configs a /etc, una sola vez
```

Uso:
```bash
sudo tools/field-ap/enable-field-ap.sh   # SSID Drone-UAS / adminadmin, IP 192.168.4.1
sudo tools/field-ap/disable-field-ap.sh  # vuelve al wifi cliente normal
```

⚠️ **Si estás conectado por SSH a través de `wlan0`, `enable-field-ap.sh` te
corta.** Conectate por `eth0` (cable) o desde teclado/monitor local antes de
correrlo.

## 2. Webapp de control (`webui/`)

Flask, corre como servicio systemd (arranca solo, sobrevive reinicios,
funciona tanto en el wifi del lab como en el AP de campo).

```bash
sudo apt install -y python3-flask
sudo cp tools/webui/px4-webui.service /etc/systemd/system/px4-webui.service
sudo systemctl daemon-reload
sudo systemctl enable --now px4-webui.service
```

Página: `http://<ip-de-la-pi>:5000` (en el AP de campo: `http://192.168.4.1:5000`).

Botones para los 5 tests de `px4_drone` (failsafe, handshake/kill switch,
indoor, outdoor, patrón cruz), con parámetros editables, checkbox de
confirmación obligatoria en los que despegan de verdad, log en vivo, y botón
STOP que corta todo limpio (agente + nodo).

`topic_version_suffix` queda fijo en `""` (firmware v1.14, lo que está
compilado ahora en `px4_msgs`) -- no es un toggle seguro de exponer en
runtime, ver `docs/offboard_control.md` sección 11.

**Cada test lanzado graba un `ros2 bag` en paralelo** (`-a`, todos los
topics visibles), guardado en `webui/logs/<test>_<timestamp>_bag/` --
complementa el `.ulog` del FC mostrando qué recibió/publicó realmente la Pi.
Útil para diagnosticar post-vuelo si algo no salió bien. `webui/logs/` no se
trackea en git (son artefactos de cada corrida, no código).

**Verificado en hardware (2026-09-10):** el flujo completo -- lanzar,
armar, ver log en vivo, grabar bag, STOP -- funciona **sin QGroundControl
conectado**, solo con el RC encendido (PX4 exige QGC *o* RC para armar, no
los dos; ver `docs/offboard_control.md` sección 6, punto 2).

## Checklist para ir a campo

1. `sudo tools/field-ap/enable-field-ap.sh` (necesita estar en `eth0` o local)
2. Conectar el celular a `Drone-UAS` / `adminadmin`
3. Abrir `http://192.168.4.1:5000`
4. RC encendido, switch de modo en Offboard, kill switch a mano
5. Al terminar: `sudo tools/field-ap/disable-field-ap.sh` para volver al wifi normal
