#!/bin/bash
export IMAGE_NAME=robopaas/rosdocked-kinetic-workspace-included:latest

set -e

docker run -d -it $IMAGE_NAME
export CONTAINER_NAME=$(docker ps --latest --format "{{.Names}}")
echo $CONTAINER_NAME
docker exec $CONTAINER_NAME bash -c "cd ~/catkin_ws/src/icclab_summit_xl/; git pull; . /opt/ros/kinetic/setup.bash; . ~/catkin_ws/devel/setup.bash; env | grep ROS; roslaunch icclab_summit_xl irlab_sim_summit_xls_amcl.launch launch_rviz_nav:=false gazebo_gui:=false"

exit 0
