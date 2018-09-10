#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/turtlebot/catkin_ws/devel/setup.bash
# execute with cgexec
cgexec -g memory:salt/minimal --sticky roslaunch icclab_turtlebot minimal_amcl.launch map_file:=/home/turtlebot/catkin_ws/src/icclab_turtlebot/icclab_latest_map.yaml initial_pose_x:=-5.97576435259 initial_pose_y:=2.07389271192 initial_pose_a:=3.14
