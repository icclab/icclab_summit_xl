#!/bin/bash
#
# test navigation stack in sim

cd ~/catkin_ws/src/icclab_summit_xl/
git pull
. /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash
roslaunch icclab_summit_xl irlab_sim_summit_xls_amcl.launch launch_rviz_nav:=false gazebo_gui:=false nav_test:=true
