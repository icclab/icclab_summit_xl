# ICCLab Summit-XL Quick-Start

This short guide will show you how to set up the environment we use for Summit-XL simulation at ICCLab.

## Prerequisites

- A linux machine able to run docker

## Steps

1. Install docker: https://docs.docker.com/install/linux/docker-ce/ubuntu/

2. Clone our scripts to use our docker environment:

	`git clone https://github.com/icclab/rosdocked-irlab.git`

3. Enter the directory and build your personalized image for the container:

	`cd rosdocked-irlab/personalized_image/`
	`./build.sh`
	
4. Launch the docker container from the same directory:

	`./run-with-dev.sh`
	
5. The container will start and mount your home directory as home.

6. Create a catkin workspace https://wiki.ros.org/catkin/Tutorials/create_a_workspace :

	`source /opt/ros/kinetic/setup.bash`
	`mkdir -p ~/catkin_ws/src`
	`cd ~/catkin_ws/`
	`catkin_make`
	
7. Clone our needed projects in ~/catkin_ws/src:
	`cd ~/catkin_ws/src`
	`git clone https://github.com/icclab/icclab_summit_xl`
	`git clone https://github.com/JenniferBuehler/gazebo-pkgs.git`

8. Install all project dependencies
	`rosdep update`
	`rosdep install -ry --from-paths .`
	
9. Setup environmental variables
	`source ~/catkin_ws/devel/setup.bash`

10. Take the simulator for a spin


	