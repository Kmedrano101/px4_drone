#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0  # = UXRCE_DDS_DOM_ID del FC y = la webui
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${repo_root}/config/cyclonedds-desktop.xml"
export ROS_LOCALHOST_ONLY=0

exec rviz2 -d "${repo_root}/config/lidar_360.rviz"
