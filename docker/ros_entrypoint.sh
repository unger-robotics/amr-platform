#!/bin/bash
set -e

# Setup ROS 2 Umgebung
source /opt/ros/humble/setup.bash

# Falls Workspace gebaut wurde, auch sourcen
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

exec "$@"
