#!/usr/bin/env python3
"""
Move arm to a safe starting position before using servo
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
import time


class MoveToSafePosition(Node):
    def __init__(self):
        super().__init__('move_to_safe_position')

        # Store joint state
        self.current_joint_state = None

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Create MoveGroup action client
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

    def joint_state_callback(self, msg):
        """Store current joint state"""
        if self.current_joint_state is None:
            self.current_joint_state = msg

    def move_to_safe(self):
        """Move arm to 'look_forward' configuration"""
        self.get_logger().info('Moving arm to safe position...')

        # Wait for joint state
        self.get_logger().info('Waiting for joint state...')
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Wait for action server
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        # Define safe position
        arm_joint_names = [
            'arm_shoulder_pan_joint',
            'arm_shoulder_lift_joint',
            'arm_elbow_joint',
            'arm_wrist_1_joint',
            'arm_wrist_2_joint',
            'arm_wrist_3_joint'
        ]

        # Safe position values (verified to work without singularity warnings)
        safe_position = [
            -1.336421520695716,    # arm_shoulder_pan_joint
            -1.0173433430859413,   # arm_shoulder_lift_joint
            -1.7652695616252208,   # arm_elbow_joint
            -0.79047386389632,     # arm_wrist_1_joint
            1.5523559894182737,    # arm_wrist_2_joint
            -1.416207801480389     # arm_wrist_3_joint
        ]

        # Create motion plan request
        goal_msg = MoveGroup.Goal()

        goal_msg.request.workspace_parameters.header.frame_id = "arm_base_link"
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()

        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # Set goal constraints
        constraints = Constraints()
        for i, joint_name in enumerate(arm_joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = safe_position[i]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

        # Send goal
        self.get_logger().info('Planning and executing motion...')
        future = self.move_group_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('Failed to send goal')
            return False

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Executing motion...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        if result_future.result() is None:
            self.get_logger().error('Failed to get result')
            return False

        result = result_future.result().result

        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✓ Arm in safe position. You can now use servo control.')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = MoveToSafePosition()

    success = node.move_to_safe()

    node.destroy_node()
    rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
