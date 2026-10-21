#!/usr/bin/env bash
# Comprueba que el dominio DDS es el MISMO en todos los sitios que tienen que
# coincidir. Si uno se cambia solo, el nodo de vuelo deja de ver al FC y el
# ciclo espera posicion para siempre (asi se perdio la salida del 2026-10-20).
#
#   tools/check_dds_domain.sh            # ficheros, y el FC si esta por USB
#   FC_DEV=/dev/ttyACM0 tools/check_dds_domain.sh
#
# El dominio lo fija el FC (UXRCE_DDS_DOM_ID). Para cambiarlo, ver
# docs/CONFIGURACION_OFFBOARD_RPI4.md en px4_drone_dev, apartado ROS_DOMAIN_ID.
set -u
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fallos=0
ref=""

mostrar() {   # nombre valor
  printf "  %-46s %s\n" "$1" "${2:-(no encontrado)}"
  if [ -z "${2:-}" ]; then fallos=$((fallos+1)); return; fi
  if [ -z "$ref" ]; then ref="$2"; elif [ "$2" != "$ref" ]; then fallos=$((fallos+1)); fi
}

echo "Dominio DDS en cada sitio que debe coincidir:"
mostrar "tools/webui/app.py (ROS_DOMAIN_ID = )" \
  "$(grep -E '^ROS_DOMAIN_ID = [0-9]+' "$repo/tools/webui/app.py" | grep -oE '[0-9]+$')"
mostrar "scripts/rviz_lidar_desktop.sh" \
  "$(grep -oE 'ROS_DOMAIN_ID=[0-9]+' "$repo/scripts/rviz_lidar_desktop.sh" | head -1 | cut -d= -f2)"
if [ -f "$HOME/.bashrc" ]; then
  mostrar "~/.bashrc (shell interactivo de la Pi)" \
    "$(grep -oE '^export ROS_DOMAIN_ID=[0-9]+' "$HOME/.bashrc" | head -1 | cut -d= -f2)"
fi

dev="${FC_DEV:-}"
[ -z "$dev" ] && [ -e /dev/ttyACM0 ] && dev=/dev/ttyACM0
tools="${MAVLINK_TOOLS:-$HOME/mavlink_tools}"
py="${MAV_PY:-$HOME/.venvs/mav/bin/python}"
if [ -n "$dev" ] && [ -f "$tools/fc.py" ] && [ -x "$py" ]; then
  fc=$(cd "$tools" && FC_DEV="$dev" FC_TIMEOUT=20 "$py" - 2>/dev/null <<'PY'
import fc
m = fc.connect()
msg = fc.param_get(m, "UXRCE_DDS_DOM_ID")   # devuelve el PARAM_VALUE entero
print(int(fc.decode(msg)) if msg is not None else "")   # PX4 mete el int32 bit a bit en el float
PY
)
  mostrar "FC UXRCE_DDS_DOM_ID (por $dev)" "$fc"
else
  echo "  FC: sin USB (/dev/ttyACM0) o sin ~/mavlink_tools: no se comprueba el FC."
fi

echo
if [ "$fallos" -eq 0 ]; then
  echo "OK: todos en el dominio $ref."
else
  echo "FALLO: $fallos sitio(s) no coinciden. El dominio lo fija el FC; todo lo demas se adapta."
  exit 1
fi
