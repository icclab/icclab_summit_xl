#!/bin/bash
#
# test navigation stack in sim

. /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash
roslaunch icclab_summit_xl irlab_summit_xl_amcl.launch launch_rviz_nav:=false gazebo_gui:=false nav_test:=true

exit 0
