#!/usr/bin/env bash
# Chequeo previo al vuelo, con el FC por USB (consola NSH via MAVLink).
# Cada linea es algo que ya ha hecho perder una salida al campo o una prueba:
#   - dominio DDS distinto en la Pi y en el FC          (2026-10-20, cinco corridas sin arrancar)
#   - MTF-01P apagado o mudo: sin LiDAR 1D ni flujo     (2026-10-21, banco; en campo = altura a ciegas)
# Uso:  tools/check_preflight.sh            (FC_DEV por defecto /dev/ttyACM0)
set -u
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
tools="${MAVLINK_TOOLS:-$HOME/mavlink_tools}"
py="${MAV_PY:-$HOME/.venvs/mav/bin/python}"
dev="${FC_DEV:-/dev/ttyACM0}"
fallos=0

ok()   { printf "  OK    %s\n" "$1"; }
mal()  { printf "  FALLO %s\n" "$1"; fallos=$((fallos+1)); }

echo "1) Dominio DDS (ficheros + FC)"
if "$repo/tools/check_dds_domain.sh" >/dev/null 2>&1; then ok "todos en el mismo dominio"; else mal "dominio DDS desalineado: ejecutar tools/check_dds_domain.sh"; fi

if [ ! -e "$dev" ] || [ ! -f "$tools/shell.py" ] || [ ! -x "$py" ]; then
  echo "2) FC por USB: no disponible ($dev / $tools): el resto no se puede comprobar"
  exit 1
fi
nsh() { (cd "$tools" && FC_DEV="$dev" FC_TIMEOUT=30 timeout 50 "$py" shell.py "$1" 2>/dev/null); }

echo "2) MTF-01P por TEL4 (LiDAR 1D + flujo optico)"
ds=$(nsh "listener distance_sensor 1")
if echo "$ds" | grep -q "never published"; then
  mal "distance_sensor NUNCA publicado: MTF-01P apagado, desconectado o en modo MSP"
else
  d=$(echo "$ds" | grep -oE "current_distance: [0-9.]+" | head -1 | awk '{print $2}')
  if [ -n "$d" ]; then ok "LiDAR 1D publica (${d} m)"; else mal "distance_sensor sin lectura"; fi
fi
of=$(nsh "listener sensor_optical_flow 1")
if echo "$of" | grep -q "never published"; then
  mal "sensor_optical_flow NUNCA publicado"
else
  q=$(echo "$of" | grep -oE "quality: [0-9]+" | head -1 | awk '{print $2}')
  if [ -n "$q" ] && [ "$q" -gt 0 ]; then ok "flujo optico publica (quality $q)"; else mal "flujo optico con quality ${q:-?}"; fi
fi

echo "3) Enlace DDS del FC (cliente uXRCE)"
cl=$(nsh "uxrce_dds_client status")
if echo "$cl" | grep -q "Running"; then
  if echo "$cl" | grep -q ", connected"; then ok "cliente DDS corriendo y conectado al agente"; else ok "cliente DDS corriendo (sin agente ahora: normal si no hay prueba lanzada)"; fi
else
  mal "cliente uXRCE-DDS no corre en el FC: revisar /fs/microsd/etc/extras.txt"
fi

echo
if [ "$fallos" -eq 0 ]; then echo "PREVUELO OK"; else echo "PREVUELO: $fallos fallo(s). No salir al campo asi."; exit 1; fi
