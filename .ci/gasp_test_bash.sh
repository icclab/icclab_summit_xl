#!/bin/bash
#
# bringup grasping in sim

set -e

# save PID of roslaunch
ROSLAUNCH_PID=$!


. /opt/ros/noetic/setup.bash

. ~/catkin_ws/devel/setup.bash
RES=`timeout -k 5m 5m roslaunch icclab_summit_xl irlab_sim_summit_xls_grasping.launch launch_rviz_grasping:=false gazebo_gui:=false | grep -c "You can start planning now!"`


# kill roslaunch
kill -15 $ROSLAUNCH_PID

exit $RES
