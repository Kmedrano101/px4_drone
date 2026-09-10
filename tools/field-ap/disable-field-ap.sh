#!/bin/bash
# Apaga el AP de campo y restaura wlan0 como cliente wifi normal (vuelve a
# conectarse a las redes configuradas en /etc/netplan, ej. Tidop-Robotica).
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Este script necesita sudo." >&2
  exit 1
fi

echo "Deteniendo hostapd y dnsmasq..."
systemctl stop hostapd || true
systemctl stop dnsmasq || true

echo "Restaurando wifi cliente en wlan0..."
ip addr flush dev wlan0
systemctl start netplan-wpa-wlan0.service
netplan apply

echo "Wifi cliente restaurado (puede tardar unos segundos en reconectar)."
