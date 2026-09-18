#!/bin/bash
# $1 = preflight | approach | scanloss | slamlost. Aislado: ROS_DOMAIN_ID 77 y solo localhost,
# porque el nodo publica ARM de verdad: nada de esto puede llegar al dron.
D=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HOME/drone_ws}
export PATH=/usr/bin:/bin ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
python3 $D/mock_fc.py $1 > /tmp/mock_$1.log 2>&1 &
MOCK=$!
timeout 40 $WS/install/px4_drone/lib/px4_drone/takeoff_position_hold_ev --ros-args -p confirm_takeoff:=true -p hold_seconds:=20.0 -p obstacle_stop_distance_m:=1.0 > /tmp/node_$1.log 2>&1
wait $MOCK
echo "--- nodo (/tmp/node_$1.log):"; grep -E "NO SE ARMA|OBSTACULO|Aterrizando|ARMADO|SLAM" /tmp/node_$1.log
echo "--- FC falso (/tmp/mock_$1.log):"; grep "mock t" /tmp/mock_$1.log
