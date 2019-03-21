#!/bin/bash
#
# test pick and place in sim

. /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/icclab_summit_xl/scripts/
python pick_and_place_summit_simulation.py 
