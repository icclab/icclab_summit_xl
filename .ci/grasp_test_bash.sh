#!/bin/bash
#
# bringup grasping in sim

#set -e

. /opt/ros/noetic/setup.bash

. ~/catkin_ws/devel/setup.bash

roslaunch icclab_summit_xl irlab_sim_summit_xls_grasping.launch launch_rviz_grasping:=false gazebo_gui:=false&

# save PID of roslaunch
ROSLAUNCH_PID=$!

# let it start completely
sleep 30

RES=`timeout -k 180s 180s rostopic echo -n 1 "/summit_xl/move_group/status" | grep -c "header"`


# kill roslaunch
kill -15 $ROSLAUNCH_PID

exit $RES
