#!/bin/bash
# Lanza ev_odometry_bridge aislado (dominio 77, solo localhost) y comprueba sus varianzas.
D=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HOME/drone_ws}
export PATH=/usr/bin:/bin ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
$WS/install/px4_drone/lib/px4_drone/ev_odometry_bridge > /tmp/bridge_check.log 2>&1 &
B=$!
sleep 1
python3 $D/check_variance.py
kill $B; wait $B 2>/dev/null
grep -E "Primera /pose" /tmp/bridge_check.log
