#!/bin/bash
# Genera el solver MPC con la version de acados de ESTA maquina y compila el
# paquete. En un paso, porque hacerlo por separado se olvida: si falta el codigo
# generado, CMake se salta el nodo MPC con un warning y el test aparece en la
# webui como no disponible.
#
#   ./scripts/mpc/build_on_this_machine.sh [ruta_del_workspace]
set -e
PKG_DIR=$(cd "$(dirname "$0")/../.." && pwd)
WS=${1:-$(cd "$PKG_DIR/../.." && pwd)}
PY=${PY:-$([ -x "$HOME/.venvs/mav/bin/python" ] && echo "$HOME/.venvs/mav/bin/python" || echo python3)}
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH

echo "== acados: $ACADOS_SOURCE_DIR ($(cd $ACADOS_SOURCE_DIR && git describe --tags 2>/dev/null || echo '?'))"
echo "== generando el solver con $PY"
"$PY" "$PKG_DIR/scripts/mpc/generate_cycle_mpc.py"

echo "== compilando en $WS"
source /opt/ros/jazzy/setup.bash
cd "$WS"
# -j1 en la Pi 4: con mas trabajos se queda sin memoria.
MAKEFLAGS=${MAKEFLAGS:--j1} colcon build --packages-select px4_drone --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
sync   # que el codigo generado y el install lleguen al disco antes de un reinicio
test -x "$WS/install/px4_drone/lib/px4_drone/takeoff_hold_land_mpc" \
  && echo "OK: takeoff_hold_land_mpc instalado" \
  || { echo "ERROR: el nodo MPC no se instalo; mira el warning de CMake arriba"; exit 1; }
