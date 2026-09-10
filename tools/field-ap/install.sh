#!/bin/bash
# Instalacion de una sola vez: copia las configs de hostapd/dnsmasq a /etc y
# deja el AP de campo activo por defecto en TODO boot (px4-field-ap.service),
# para que el dron se recupere solo tras un corte de bateria en campo sin
# depender de acceso previo. hostapd/dnsmasq en si quedan deshabilitados del
# arranque directo (no compiten en boot); es px4-field-ap.service el que los
# arranca via enable-field-ap.sh.
#
# En el lab: usar eth0 para acceso normal, o disable-field-ap.sh para volver
# a wlan0 cliente hasta el proximo reinicio (el servicio vuelve a poner el AP
# en el boot siguiente -- ver tools/README.md).
#
# Requiere: apt install -y hostapd dnsmasq iw (ya instalado si llegaste aca
# siguiendo docs/field-ap.md).
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Este script necesita sudo." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

install -m 600 "$SCRIPT_DIR/hostapd.conf" /etc/hostapd/hostapd.conf
install -m 644 "$SCRIPT_DIR/dnsmasq-field-ap.conf" /etc/dnsmasq.d/field-ap.conf

sed -i 's|^#DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd 2>/dev/null || \
  echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' >> /etc/default/hostapd

systemctl unmask hostapd
systemctl disable hostapd dnsmasq

install -m 644 "$SCRIPT_DIR/px4-field-ap.service" /etc/systemd/system/px4-field-ap.service
systemctl daemon-reload
systemctl enable px4-field-ap.service

echo "Instalado. AP de campo (Drone-UAS) activo por defecto en cada boot."
echo "disable-field-ap.sh vuelve a wlan0 cliente hasta el proximo reinicio."
