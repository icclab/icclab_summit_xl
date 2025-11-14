#!/usr/bin/env python3
"""
Setup servo for use - moves arm to safe position and sets command type
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit_msgs.srv import ServoCommandType
import time


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('setup_servo')

    # Step 1: Move to safe position
    node.get_logger().info('Step 1: Moving arm to safe position...')
    moveit = MoveItPy(node_name="moveit_py_planning")
    arm_group = moveit.get_planning_component("arm")

    arm_group.set_goal_state(configuration_name="up")
    plan_result = arm_group.plan()

    if plan_result:
        node.get_logger().info('Planning successful, executing...')
        arm_group.execute(blocking=True)
        node.get_logger().info('✓ Arm in safe position')
    else:
        node.get_logger().error('Failed to plan to safe position')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Step 2: Set servo command type to TWIST (Cartesian control)
    node.get_logger().info('Step 2: Setting servo command type to TWIST...')

    # Create service client
    cli = node.create_client(ServoCommandType, '/servo_node/switch_command_type')

    # Wait for service
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Servo command type service not available')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Call service
    req = ServoCommandType.Request()
    req.command_type = 1  # TWIST for Cartesian control

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if future.result() is not None:
        if future.result().success:
            node.get_logger().info('✓ Servo command type set to TWIST (Cartesian control)')
            node.get_logger().info('')
            node.get_logger().info('==============================================')
            node.get_logger().info('Servo is ready! You can now use:')
            node.get_logger().info('  ros2 run icclab_summit_xl servo_keyboard_control.py')
            node.get_logger().info('==============================================')
        else:
            node.get_logger().warn('Failed to set command type')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
