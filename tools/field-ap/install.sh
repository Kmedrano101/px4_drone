#!/bin/bash
# Instalacion de una sola vez: copia las configs de hostapd/dnsmasq a /etc y
# se asegura de que ninguno de los dos arranque solo al bootear (el toggle
# los prende/apaga a mano via enable-field-ap.sh / disable-field-ap.sh).
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

echo "Instalado. hostapd/dnsmasq configurados pero apagados por defecto."
echo "Usar enable-field-ap.sh / disable-field-ap.sh para el toggle."
