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

# TEST #1: testing navigation stack
# bringup sim complete with amcl and move_base action client
echo "Testing navigation stack..."
echo "Executing 'roslaunch icclab_summit_xl irlab_sim_summit_xls_amcl.launch launch_rviz_nav:=false gazebo_gui:=false nav_test:=true' inside container $CONTAINER_NAME..."
docker exec -it $CONTAINER_NAME bash -c "cd ~/catkin_ws/src/icclab_summit_xl/; git pull; . /opt/ros/kinetic/setup.bash; . ~/catkin_ws/devel/setup.bash; roslaunch icclab_summit_xl irlab_sim_summit_xls_amcl.launch launch_rviz_nav:=false gazebo_gui:=false nav_test:=true"

# extract output of movebase_client_py node
docker exec -it $CONTAINER_NAME bash -c ". /opt/ros/kinetic/setup.bash; roscd log; cat $(ls | grep movebase_client)" >> movebase_client_py_log.txt
nav_test_output=$(cat movebase_client_py_log.txt)

if [[ "$nav_test_output" == *"fail"* ]] ; then
  echo "Navigation test failed. Check output."
  echo $nav_test_output
  exit 1
elif [[ "$nav_test_output" == *"success"* ]]; then
  echo "Navigation test succeeded. No issues found"
  exit 0
fi

exit 0
