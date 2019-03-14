#!/usr/bin/env bash
export IMAGE_NAME=robopaas/rosdocked-kinetic-workspace-included:latest

set -e

docker run\
  -h `hostname`\
  --privileged\
  --net=host\
  -e SHELL\
  -e DISPLAY=:0\
  -e DOCKER=1\
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"\
  -it -d $IMAGE_NAME $SHELL

export CONTAINER_NAME=$(docker ps --latest --format "{{.Names}}")
echo $CONTAINER_NAME

# bringup sim complete with amcl and without rviz or gazebo gui
docker exec -d $CONTAINER_NAME bash -c "cd ~/catkin_ws/src/icclab_summit_xl/; git pull; . /opt/ros/kinetic/setup.bash; . ~/catkin_ws/devel/setup.bash; roslaunch icclab_summit_xl irlab_sim_summit_xls_amcl.launch launch_rviz_nav:=false gazebo_gui:=false"

# launch move base action client
docker exec $CONTAINER_NAME bash -c ". /opt/ros/kinetic/setup.bash; . ~/catkin_ws/devel/setup.bash; ROS_NAMESPACE=summit_xl rosrun icclab_summit_xl move_base_client.py"

exit 0
