#!/usr/bin/python
#
# Send a value to change the opening of the Robotiq gripper using an action
#

import argparse

import rospy
import actionlib
import control_msgs.msg


def gripper_client(value):

    # Create an action client
    client_l = actionlib.SimpleActionClient(
        '/summit_xl/gripper_left_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    client_r = actionlib.SimpleActionClient(
        '/summit_xl/gripper_right_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client_l.wait_for_server()
    client_r.wait_for_server()

    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value   # From 0.0 to 0.8
    goal.command.max_effort = -1.0  # Do not limit the effort
    client_l.send_goal(goal)
    client_r.send_goal(goal)

    client_l.wait_for_result()
    client_r.wait_for_result()
    return client_l.get_result()


if __name__ == '__main__':
    try:
        # Get the angle from the command line
        parser = argparse.ArgumentParser()
        parser.add_argument("--value", type=float, default="0.2",
                            help="Value betwewen 0.2 (open) 0 (closed)")
        args = parser.parse_args()
        gripper_value = args.value
        # Start the ROS node
        rospy.init_node('gripper_command')
        # Set the value to the gripper
        result = gripper_client(gripper_value)
        
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
