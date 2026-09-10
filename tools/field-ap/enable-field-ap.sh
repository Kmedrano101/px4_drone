#!/bin/bash
# Activa el AP de campo en wlan0: SSID Drone-UAS, IP 192.168.4.1.
#
# OJO: si estas conectado por SSH a traves de wlan0 (wifi), esto te corta.
# Conectate por eth0 (cable) o desde el teclado/monitor local antes de correr
# esto. Ver docs/field-ap.md.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Este script necesita sudo." >&2
  exit 1
fi

echo "Deteniendo wifi cliente en wlan0..."
systemctl stop netplan-wpa-wlan0.service || true

echo "Asignando IP estatica 192.168.4.1/24 a wlan0..."
ip addr flush dev wlan0
ip addr add 192.168.4.1/24 dev wlan0
ip link set wlan0 up

echo "Arrancando hostapd (AP) y dnsmasq (DHCP)..."
systemctl start hostapd
systemctl start dnsmasq

echo ""
echo "AP de campo activo:"
echo "  SSID:     Drone-UAS"
echo "  Password: adminadmin"
echo "  Pagina:   http://192.168.4.1:5000"
