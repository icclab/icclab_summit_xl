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
# TODO: output log of this process
docker exec -d $CONTAINER_NAME bash -c "cd ~/catkin_ws/src/icclab_summit_xl/; git pull; . /opt/ros/kinetic/setup.bash; . ~/catkin_ws/devel/setup.bash; roslaunch icclab_summit_xl irlab_sim_summit_xls_amcl.launch launch_rviz_nav:=false gazebo_gui:=false"

# wait for repo to pulled
sleep 5

# TODO: assign output to a variable
# launch move base action client
docker exec -it $CONTAINER_NAME bash -c ". /opt/ros/kinetic/setup.bash; . ~/catkin_ws/devel/setup.bash; ROS_NAMESPACE=summit_xl rosrun icclab_summit_xl move_base_client.py"

# kill gzserver process
ps -ef | grep 'gzserver' | grep -v grep | awk '{print $2}' | xargs -r kill -9

#echo $nav_test_output
#if [[ "$nav_test_output" == *"fail"* ]] ; then
#  echo "Navigation test failed. Check output."
#  echo $nav_test_output
#  exit 1
#elif [[ "$nav_test_output" == *"success"* ]]; then
#  echo "Navigation test succeeded. No issues found"
#  exit 0
#fi

exit 0
