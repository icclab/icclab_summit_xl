#!/bin/bash
#
# bringup grasping in sim

set -e

. /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash
roslaunch icclab_summit_xl irlab_sim_summit_xls_grasping.launch launch_rviz_grasping:=false gazebo_gui:=false

exit 0
