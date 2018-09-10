#!/bin/bash
source /opt/ros/kinetic/setup.bash 
source /home/turtlebot/catkin_ws/devel/setup.bash

# kill all running nodes from minimal + amcl
PIDS=$(cat /sys/fs/cgroup/memory/salt/minimal/cgroup.procs)
for pid in $PIDS; do
    [ -d /proc/$pid ] && kill -INT $pid
done
sleep 1
# Make second loop and hard kill any remaining
for pid in $PIDS; do
    [ -d /proc/$pid ] && kill -KILL $pid
done

#stop rplidar motor
exec roslaunch icclab_turtlebot stop_rplidar_motor.launch
