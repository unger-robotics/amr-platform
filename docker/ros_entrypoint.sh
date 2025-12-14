#!/bin/bash
set -e

# ROS 2 Umgebung laden
source "/opt/ros/humble/setup.bash"

# Falls der Workspace schon gebaut wurde, lade ihn auch
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
  source "/root/ros2_ws/install/setup.bash"
fi

exec "$@"
