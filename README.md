# icclab_summit_xl
[![Build Status](https://travis-ci.com/icclab/icclab_summit_xl.svg?branch=noetic)](https://travis-ci.com/icclab/icclab_summit_xl)

Base scripts for robots at ICCLab

For documentation [see the wiki](https://github.com/icclab/icclab_summit_xl/wiki)

Quick Start:

1. Launch the simulation:

        ros2 launch icclab_summit_xl summit_xl_simulation.launch.py

2. Launch Nav2:

        ros2 launch icclab_summit_xl summit_xl_nav2.launch.py

3. Launch arm control:

        ros2 launch icclab_summit_xl summit_xl_move_it.launch.py

For Mapping + Nav2:

        ros2 launch summit_xl_navigation nav2_bringup_launch.py slam:=True


Visualizing Nav2 Rviz config launched manually (notice the namespacing):

	ros2 launch icclab_summit_xl rviz.launch.py rviz_config:=navigation.rviz
