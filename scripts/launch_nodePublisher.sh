#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/turtlebot/catkin_ws/devel/setup.bash
# create cgroup
#cgcreate -a root -t turtlebot -g memory:salt/nodepublisher
# execute with cgexec
cgexec -g memory:salt/nodepublisher --sticky rosrun rosnodeinfo nodes.py 
