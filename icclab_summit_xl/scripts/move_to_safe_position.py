#!/usr/bin/env python3
"""
Move arm to a safe starting position before using servo
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
import time


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('move_to_safe_position')

    node.get_logger().info('Initializing MoveItPy...')
    moveit = MoveItPy(node_name="moveit_py_planning")

    arm_group = moveit.get_planning_component("arm")

    # Move to 'up' position which is away from singularities
    node.get_logger().info('Moving to safe "up" position...')
    arm_group.set_goal_state(configuration_name="up")

    plan_result = arm_group.plan()
    if plan_result:
        node.get_logger().info('Planning successful, executing...')
        arm_group.execute(blocking=True)
        node.get_logger().info('✓ Arm in safe position. You can now use servo control.')
    else:
        node.get_logger().error('Failed to plan to safe position')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
