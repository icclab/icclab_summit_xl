#!/bin/bash

ldconfig /opt/ros/kinetic/lib/

ttyACM0_id=$(/opt/ros/kinetic/lib/urg_node/getID /dev/ttyACM0 q)
ttyACM1_id=$(/opt/ros/kinetic/lib/urg_node/getID /dev/ttyACM1 q)

lidar_front_id="H1814665"
lidar_rear_id="H1814667"

rm /dev/lidar_front
rm /dev/lidar_rear

if [ "$ttyACM0_id" == "$lidar_front_id" ]; then
	ln -s /dev/ttyACM0 /dev/lidar_front
fi
if [ "$ttyACM0_id" == "$lidar_rear_id" ]; then	
	ln -s /dev/ttyACM0 /dev/lidar_rear
fi
if [ "$ttyACM1_id" == "$lidar_front_id" ]; then	
	ln -s /dev/ttyACM1 /dev/lidar_front
fi
if [ "$ttyACM1_id" == "$lidar_rear_id" ]; then	
	ln -s /dev/ttyACM1 /dev/lidar_rear
fi




