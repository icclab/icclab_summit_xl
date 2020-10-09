#!/bin/sh
#
# Send a value to change the opening of an effort controlled gripper
#

if [ "$#" -ne 1 ]
  then echo "usage: send_effort_gripper.sh <effort_float_value>  # where positive values close the gripper, negative values open it"
else
  rostopic pub -1 /summit_xl/gripper_left_controller/command std_msgs/Float64 -- $1 &
  #rostopic pub -1 /summit_xl/gripper_right_controller/command std_msgs/Float64 -- $1
fi
