#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash #NOTE: this should be set to catkin_ws dir
exec roslaunch icclab_turtlebot rplidar.launch
