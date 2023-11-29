#!/bin/bash

#Start a background screen name the session "summit":
screen -d -m -S summit

#(Re)Start VPN, enable multicast
screen -S summit -X exec bash -c "sudo wg-quick down wg0; sudo wg-quick up wg0; sudo ip link set dev wg0 multicast on; sudo wg show"

#Start another window, name it "robot" and run robot startup in it
screen -S summit -X screen -t robot
screen -S summit -p robot -X exec roslaunch icclab_summit_xl summit_xl_base_bringup.launch

#In another window we have the ros1_bridge
screen -S summit -X screen -t ros1_bridge
screen -S summit -p ros1_bridge -X exec docker run -it --privileged --network=host --ipc=host --pid=host -v /home/summit:/home/summit ros:galactic-ros1-bridge bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/cyclonedds-mc-galactic.xml; ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"

#In another window we have the ros2 container that will run nav2
screen -S summit -X screen -t ros2
screen -S summit -p ros2 -X exec docker run -it --privileged --network=host --ipc=host --pid=host --env UID=$(id -u) --env GID=$(id -g) -v /home/summit:/home/summit -v /dev/lidar_front:/dev/lidar_front -v /dev/lidar_rear:/dev/lidar_rear -v /dev/astra_s:/dev/astra_s robopaas/rosdocked-humble-cpu bash -c "export ROS_DISTRO=humble; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:////home/summit/cyclonedds-mc.xml; source /home/ros/colcon_ws/install/setup.bash; ros2 launch icclab_summit_xl summit_xl_real.launch.py"




