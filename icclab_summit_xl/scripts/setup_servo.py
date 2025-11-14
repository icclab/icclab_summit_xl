#!/usr/bin/env python3
"""
Setup servo for use - moves arm to safe position and sets command type
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import JointState
import time


class SetupServo(Node):
    def __init__(self):
        super().__init__('setup_servo_node')

        # Store joint names and current state
        self.joint_names = None
        self.current_joint_state = None

        # Subscribe to joint states to get current configuration
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

        # Create servo service client
        self.servo_client = self.create_client(
            ServoCommandType,
            '/servo_node/switch_command_type'
        )

    def joint_state_callback(self, msg):
        """Store current joint state"""
        if self.current_joint_state is None:
            self.current_joint_state = msg
            self.joint_names = msg.name

    def move_to_safe_position(self):
        """Move arm to 'up' configuration"""
        self.get_logger().info('Step 2: Moving arm to safe position...')

        # Wait for joint state
        self.get_logger().info('Waiting for joint state...')
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Wait for action server
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        # Define safe position joint values
        # These values are verified to avoid singularity warnings
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

        # Set request parameters
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

        goal_msg.planning_options.plan_only = False  # Plan and execute
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

        # Send goal
        self.get_logger().info('Sending goal to MoveGroup...')
        future = self.move_group_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('Failed to send goal to MoveGroup')
            return False

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveGroup')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        if result_future.result() is None:
            self.get_logger().error('Failed to get result from MoveGroup')
            return False

        result = result_future.result().result

        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✓ Arm moved to safe position')
            return True
        else:
            self.get_logger().error(f'Motion planning failed with error code: {result.error_code.val}')
            return False

    def set_servo_command_type(self, cmd_type=0): #JOINT_JOG
        """Set servo command type to cmd_type"""
        self.get_logger().info('Step 1: Setting servo command type to (0: JOINT_JOG, 1:TWIST): ' + str(cmd_type))

        # Wait for service
        if not self.servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servo command type service not available')
            return False

        # Call service
        req = ServoCommandType.Request()
        req.command_type = cmd_type  # TWIST for Cartesian control

        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('✓ Servo command type set')
                self.get_logger().info('')
                if (cmd_type == 1):
                    self.get_logger().info('==============================================')
                    self.get_logger().info('Servo is ready! You can now use:')
                    self.get_logger().info('  ros2 run icclab_summit_xl servo_keyboard_control.py')
                    self.get_logger().info('==============================================')
                return True
            else:
                self.get_logger().warn('Failed to set command type')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = SetupServo()

    # Step 1: Set command type to JOINT so we can move the arm
    if not node.set_servo_command_type(0):
        node.get_logger().error('Failed to set servo command type')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Small delay to let servo process command type change
    time.sleep(0.5)

    # Step 2: Move to safe position (now that servo is in JOINT_JOG mode)
    if not node.move_to_safe_position():
        node.get_logger().error('Failed to move to safe position')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Step 3: Set command type (must be done before servo can accept commands)
    if not node.set_servo_command_type(1):
        node.get_logger().error('Failed to set servo command type')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Small delay to let servo process command type change
    time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
