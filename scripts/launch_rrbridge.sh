#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/turtlebot/catkin_ws/devel/setup.bash
# create cgroup
#cgcreate -a turtlebot -t turtlebot -g memory:salt/rrbridge
# execute with cgexec
cgexec -g memory:salt/rrbridge --sticky roslaunch rr_cloud_bridge rr_cloud_bridge.launch
