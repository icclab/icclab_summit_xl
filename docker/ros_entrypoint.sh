#!/bin/bash
set -e

export QT_GRAPHICSSYSTEM=native
# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.bash"
exec "$@"
