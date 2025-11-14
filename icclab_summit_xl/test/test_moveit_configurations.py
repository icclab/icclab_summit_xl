#!/usr/bin/env python3
"""
Test to verify that MoveIt can reach all pre-set configurations.
This checks that:
1. MoveIt move_group is running
2. The arm can reach all pre-set configurations (home, up, docked, look_forward)
3. The gripper can reach all pre-set configurations (open, closed)
4. No planning or execution errors occur
"""

import unittest
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import time


class MoveItConfigurationsTest(unittest.TestCase):

    # Pre-set configurations from summit_xl.srdf
    ARM_CONFIGURATIONS = {
        'home': {
            'arm_elbow_joint': 0.0,
            'arm_shoulder_lift_joint': 0.0,
            'arm_shoulder_pan_joint': 0.0,
            'arm_wrist_1_joint': 0.0,
            'arm_wrist_2_joint': 0.0,
            'arm_wrist_3_joint': 0.0,
        },
        'up': {
            'arm_elbow_joint': 0.0,
            'arm_shoulder_lift_joint': -1.4232,
            'arm_shoulder_pan_joint': 0.0,
            'arm_wrist_1_joint': 0.0,
            'arm_wrist_2_joint': 0.0,
            'arm_wrist_3_joint': 0.0,
        },
        'docked': {
            'arm_elbow_joint': -2.8291,
            'arm_shoulder_lift_joint': 0.0,
            'arm_shoulder_pan_joint': 0.0,
            'arm_wrist_1_joint': 0.0,
            'arm_wrist_2_joint': 0.0,
            'arm_wrist_3_joint': 0.0,
        },
        'look_forward': {
            'arm_elbow_joint': -2.8291,
            'arm_shoulder_lift_joint': -0.243,
            'arm_shoulder_pan_joint': 0.0,
            'arm_wrist_1_joint': -0.7984,
            'arm_wrist_2_joint': 1.5621,
            'arm_wrist_3_joint': 0.0,
        },
    }

    GRIPPER_CONFIGURATIONS = {
        'open': {
            'finger_joint': 0.0,
        },
        'closed': {
            'finger_joint': 0.7,
        },
    }

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_moveit_configurations')
        cls.node.get_logger().info('MoveIt configurations test node started')

        # Initialize MoveItPy
        try:
            cls.node.get_logger().info('Initializing MoveItPy...')
            cls.moveit = MoveItPy(node_name="moveit_py_test")
            cls.node.get_logger().info('MoveItPy initialized successfully')

            # Get planning components
            cls.arm_group = cls.moveit.get_planning_component("arm")
            cls.gripper_group = cls.moveit.get_planning_component("gripper")
            cls.node.get_logger().info('Planning components retrieved')

        except Exception as e:
            cls.node.get_logger().error(f'Failed to initialize MoveItPy: {e}')
            raise

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _plan_and_execute(self, planning_component, goal_state_name, timeout=15.0):
        """Plan and execute motion to a named goal state."""
        self.node.get_logger().info(f'Planning to: {goal_state_name}')

        # Set goal state
        planning_component.set_goal_state(configuration_name=goal_state_name)

        # Plan
        self.node.get_logger().info('Planning...')
        start_time = time.time()
        plan_result = planning_component.plan()
        planning_time = time.time() - start_time

        if not plan_result:
            self.node.get_logger().error(f'Planning failed for {goal_state_name}')
            return False

        self.node.get_logger().info(f'Planning successful (took {planning_time:.2f}s)')

        # Execute
        self.node.get_logger().info('Executing...')
        robot = self.moveit.get_robot_model()
        robot_state = RobotState(robot)

        start_time = time.time()
        execute_success = planning_component.execute(blocking=True)
        execution_time = time.time() - start_time

        if not execute_success:
            self.node.get_logger().error(f'Execution failed for {goal_state_name}')
            return False

        self.node.get_logger().info(
            f'✓ Successfully moved to {goal_state_name} '
            f'(planning: {planning_time:.2f}s, execution: {execution_time:.2f}s)'
        )

        # Wait a bit for the robot to settle
        time.sleep(0.5)

        return True

    def test_moveit_initialization(self):
        """Test that MoveItPy initialized successfully."""
        self.node.get_logger().info('Testing MoveIt initialization...')
        self.assertIsNotNone(self.moveit, "MoveItPy not initialized")
        self.assertIsNotNone(self.arm_group, "Arm planning component not available")
        self.assertIsNotNone(self.gripper_group, "Gripper planning component not available")
        self.node.get_logger().info('✓ MoveIt initialized successfully')

    def test_arm_home_configuration(self):
        """Test moving arm to home configuration."""
        self.node.get_logger().info('Testing arm home configuration...')
        success = self._plan_and_execute(self.arm_group, 'home')
        self.assertTrue(success, "Failed to move arm to home configuration")

    def test_arm_up_configuration(self):
        """Test moving arm to up configuration."""
        self.node.get_logger().info('Testing arm up configuration...')
        success = self._plan_and_execute(self.arm_group, 'up')
        self.assertTrue(success, "Failed to move arm to up configuration")

    def test_arm_docked_configuration(self):
        """Test moving arm to docked configuration."""
        self.node.get_logger().info('Testing arm docked configuration...')
        success = self._plan_and_execute(self.arm_group, 'docked')
        self.assertTrue(success, "Failed to move arm to docked configuration")

    def test_arm_look_forward_configuration(self):
        """Test moving arm to look_forward configuration."""
        self.node.get_logger().info('Testing arm look_forward configuration...')
        success = self._plan_and_execute(self.arm_group, 'look_forward')
        self.assertTrue(success, "Failed to move arm to look_forward configuration")

    def test_gripper_open_configuration(self):
        """Test moving gripper to open configuration."""
        self.node.get_logger().info('Testing gripper open configuration...')
        success = self._plan_and_execute(self.gripper_group, 'open')
        self.assertTrue(success, "Failed to move gripper to open configuration")

    def test_gripper_closed_configuration(self):
        """Test moving gripper to closed configuration."""
        self.node.get_logger().info('Testing gripper closed configuration...')
        success = self._plan_and_execute(self.gripper_group, 'closed')
        self.assertTrue(success, "Failed to move gripper to closed configuration")

    def test_arm_configuration_cycle(self):
        """Test cycling through all arm configurations."""
        self.node.get_logger().info('Testing arm configuration cycle...')

        configurations = ['home', 'up', 'docked', 'look_forward', 'home']

        for config in configurations:
            success = self._plan_and_execute(self.arm_group, config)
            self.assertTrue(
                success,
                f"Failed during arm configuration cycle at: {config}"
            )

        self.node.get_logger().info('✓ Successfully cycled through all arm configurations')

    def test_gripper_configuration_cycle(self):
        """Test cycling through all gripper configurations."""
        self.node.get_logger().info('Testing gripper configuration cycle...')

        # Cycle between open and closed multiple times
        configurations = ['open', 'closed', 'open', 'closed', 'open']

        for config in configurations:
            success = self._plan_and_execute(self.gripper_group, config)
            self.assertTrue(
                success,
                f"Failed during gripper configuration cycle at: {config}"
            )

        self.node.get_logger().info('✓ Successfully cycled through gripper configurations')


if __name__ == '__main__':
    import sys
    # Remove ROS args before running unittest
    filtered_argv = [arg for arg in sys.argv if not arg.startswith('__')]
    unittest.main(argv=filtered_argv)
