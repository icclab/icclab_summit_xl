#!/usr/bin/python
# Send a value to change the opening of the Robotiq gripper using an action


import rospy
# brings the SimpleActionClient
import actionlib
import argparse
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

def gripper_client(value):
    if value > 0.14:
        print ("value over the limits value becomes 0.14")
        value = 0.14
    if value < 0.0:
        print ("value negative it becomes 0")
        value = 0.0

    robotiq_client = actionlib.SimpleActionClient('summit_xl/command_robotiq_action', CommandRobotiqGripperAction)

    # Wait until the action server has been started and is listening for goals
    robotiq_client.wait_for_server()

    # Create a goal to send (to the action server)
    goal = CommandRobotiqGripperGoal()
    print (value)
    goal.position = value   # From 0.0 to 0.14
    goal.speed= 0.1  
    goal.force= 5.0  
    robotiq_client.send_goal(goal)

    # w8 for action to be executed
    robotiq_client.wait_for_result()
    return robotiq_client.get_result()


if __name__ == '__main__':
    try:
        # Get the angle from the command line
        parser = argparse.ArgumentParser()
        parser.add_argument("--value", type=float, default="0.2",
                            help="Value betwewen 0.0 (open) and 0.14 (closed)")
        args = parser.parse_args()
        gripper_value = args.value

        # Start the ROS node
        rospy.init_node('gripper_command')
        # Set the value to the gripper
        result = gripper_client(gripper_value)

    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
