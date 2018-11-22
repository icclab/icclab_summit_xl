# icclab_turtlebot

Base scripts for robots at ICCLab/ICRLab

## Summit XL with arm simulation

Run:

	roslaunch icclab_turtlebot irlab_sim_summit_xls_complete.launch	
	
This will enable navigation and gazebo simulation

Run:

	ROS_NAMESPACE=summit_xl roslaunch robotnik_ur5_moveit_config demo.launch
	
This will start the move_group and allow you to control the arm
