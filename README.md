# icclab_turtlebot
Base scripts for the turtlebots at ICCLab.

NOTE DEPENDENCIES: 
- the rplidar model uses a mesh from the "hector_sensors_description" package, so you should have that installed
- the actual rplidar ROS node we're using allows us to stop the rotation motor, and it's here: https://github.com/negre/rplidar_ros.git

# TL;DR

In the launch directory you will find:
- minimal_with_rplidar.launch : launches the "minimal" turtlebot_bringup script + the rplidar node
- gmapping_icclab.launch : gmapping using the rplidar input on /scan topic instead of the kinect
- amcl_icclab.launch : amcl using the rplidar

In the base directory, the file 10-local.rules makes sure that when the rplidar is connected with 
USB it's given a consistent name through a symlink (/dev/rplidar) you should save the file in
/etc/udev/rules.d/10-local.rules on your turtlebot

# How to try things out

## Simulation of SLAM using Gazebo + rviz

On laptop:
  
  roslaunch icclab_turtlebot gmapping_icclab_simulation.launch GAZEBO_GUI:=true

## SLAM with actual Turtlebot

On Turtlebot launch:
  
  roslaunch icclab_turtlebot minimal_with_rplidar.launch
  
  roslaunch icclab_turtlebot amcl_icclab.launch map_file:=/home/turtlebot/catkin_ws/src/icclab_turtlebot/icclab_latest_map.yaml
  roslaunch icclab_turtlebot amcl_icclab.launch map_file:=/home/turtlebot/catkin_ws/src/icclab_turtlebot/icclab_latest_map.yaml initial_pose_x:=-6.2 initial_pose_y:=2 initial_pose_a:=3.50

On laptop:

  roslaunch turtlebot_rviz_launchers view_navigation.launch

