For using ar_track_alvar with noetic you need to clone the following repository selecting the kinetic-devel branch:

https://github.com/ros-perception/ar_track_alvar

To use the aruco tag models in gazebo you need to set the right path:

export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/icclab_summit_xl/worlds/models:$GAZEBO_MODEL_PATH


# icclab_summit_xl
[![Build Status](https://travis-ci.com/icclab/icclab_summit_xl.svg?branch=master)](https://travis-ci.com/icclab/icclab_summit_xl)

Base scripts for robots at ICCLab/ICRLab

For documentation [see the wiki](https://github.com/icclab/icclab_summit_xl/wiki)
