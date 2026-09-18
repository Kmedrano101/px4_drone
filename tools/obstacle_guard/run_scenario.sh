#!/bin/bash
# $1 = preflight | approach | scanloss | slamlost | userhold | cancel. Aislado: ROS_DOMAIN_ID 77 y solo localhost,
# porque el nodo publica ARM de verdad: nada de esto puede llegar al dron.
D=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HOME/drone_ws}
export PATH=/usr/bin:/bin ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
python3 $D/mock_fc.py $1 > /tmp/mock_$1.log 2>&1 &
MOCK=$!
NODE=$WS/install/px4_drone/lib/px4_drone/takeoff_position_hold_ev
case $1 in
  userhold)
    # Patron en cruz; HOLD en pleno tramo (el tramo 1 empieza ~2 s tras armar) y ATERRIZAR 5 s despues.
    timeout 40 $NODE --ros-args -p confirm_takeoff:=true -p hold_seconds:=1.0 -p pattern_distance_m:=1.0 \
      -p obstacle_stop_distance_m:=1.0 > /tmp/node_$1.log 2>&1 &
    N=$!
    # La senal va al NODO por su nombre exacto, no a $N (que es `timeout`): mandarsela al
    # envoltorio lo mata y deja el nodo volando solo. Mismo cuidado en la webui.
    sleep 12.5; pkill -USR1 -x takeoff_positio; echo "[runner] HOLD (SIGUSR1) enviado"
    sleep 5; pkill -USR2 -x takeoff_positio; echo "[runner] ATERRIZAR (SIGUSR2) enviado"
    wait $N ;;
  cancel)
    timeout 40 $NODE --ros-args -p confirm_takeoff:=true -p hold_seconds:=20.0 > /tmp/node_$1.log 2>&1 &
    N=$!
    sleep 3; pkill -USR1 -x takeoff_positio; echo "[runner] HOLD (SIGUSR1) enviado antes de armar"
    wait $N; kill $MOCK 2>/dev/null ;;
  *)
    timeout 40 $NODE --ros-args -p confirm_takeoff:=true -p hold_seconds:=20.0 -p obstacle_stop_distance_m:=1.0 \
      > /tmp/node_$1.log 2>&1 ;;
esac
wait $MOCK
echo "--- nodo (/tmp/node_$1.log):"; grep -E "NO SE ARMA|OBSTACULO|Aterrizando|ARMADO|SLAM no|HOLD|Tramo|cancelada" /tmp/node_$1.log
echo "--- FC falso (/tmp/mock_$1.log):"; grep "mock t" /tmp/mock_$1.log
