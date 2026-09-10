# Herramientas de campo

Setup para probar los nodos de `px4_drone` en campo desde el celular, sin
laptop: la Pi genera su propio AP wifi y sirve una webapp con botones para
cada test.

## 1. AP de campo (`field-ap/`)

Wifi propio de la Pi (`wlan0`), independiente del wifi del lab.
**Activo por defecto en todo boot** (`px4-field-ap.service`) -- a propósito,
para que el dron se recupere solo tras un corte de batería en campo, sin
depender de acceso previo por SSH. `wlan0` **no** se conecta sola al wifi
del lab mientras este servicio esté habilitado.

```bash
sudo apt install -y hostapd dnsmasq iw
sudo tools/field-ap/install.sh   # copia configs a /etc, habilita el AP en cada boot
```

⚠️ **`install.sh` deja el AP activo de inmediato.** Si estás conectado por
SSH a través de `wlan0`, esto te corta. Conectate por `eth0` (cable) o desde
teclado/monitor local antes de correrlo.

Uso manual (toggle temporal, hasta el próximo reinicio -- el servicio
vuelve a poner el AP en el boot siguiente):
```bash
sudo tools/field-ap/disable-field-ap.sh  # vuelve a wlan0 cliente (lab)
sudo tools/field-ap/enable-field-ap.sh   # vuelve al AP sin esperar un reboot
```

Para volver el AP a manual-solamente (no arrancar solo en boot):
```bash
sudo systemctl disable px4-field-ap.service
```

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

El AP ya está activo por defecto (arranca solo con la Pi) -- no hace falta
prenderlo a mano salvo que lo hayas desactivado antes en el lab.

1. Conectar el celular a `Drone-UAS` / `adminadmin`
2. Abrir `http://192.168.4.1:5000`
3. RC encendido, switch de modo en Offboard, kill switch a mano
4. De vuelta en el lab: `sudo tools/field-ap/disable-field-ap.sh` si querés
   `wlan0` cliente hasta el próximo reinicio (o `eth0` para acceso directo)
