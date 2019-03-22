#!/bin/bash
#
# test pick and place in sim

set -e

. /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/icclab_summit_xl/scripts/
printf 'y\ny\ny\n' | python pick_and_place_summit_simulation.py # enters 'y' when prompted

exit 0