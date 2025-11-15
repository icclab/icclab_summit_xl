#!/usr/bin/env python3
"""
Test to verify that the Gazebo simulation is running and ready.
This checks that:
1. The simulation is publishing clock messages
2. Joint states are being published
3. The robot spawned successfully
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
import time


class SimulationReadinessTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_simulation_readiness')
        cls.node.get_logger().info('Simulation readiness test node started')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_clock_published(self):
        """Test that simulation clock is being published."""
        self.node.get_logger().info('Testing if /clock topic is being published...')

        clock_received = False

        def clock_callback(msg):
            nonlocal clock_received
            clock_received = True

        subscription = self.node.create_subscription(
            Clock,
            '/clock',
            clock_callback,
            10
        )

        # Wait up to 10 seconds for clock message
        timeout = 10.0
        start_time = time.time()
        while not clock_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)
        self.assertTrue(clock_received, "Simulation clock not being published on /clock")
        self.node.get_logger().info('✓ Clock topic is being published')

    def test_joint_states_published(self):
        """Test that robot joint states are being published."""
        self.node.get_logger().info('Testing if /joint_states topic is being published...')

        joint_state_received = False
        received_joint_names = []

        def joint_state_callback(msg):
            nonlocal joint_state_received, received_joint_names
            joint_state_received = True
            received_joint_names = msg.name

        subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
            joint_state_callback,
            10
        )

        # Wait up to 15 seconds for joint state message
        timeout = 15.0
        start_time = time.time()
        while not joint_state_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)
        self.assertTrue(joint_state_received, "Joint states not being published on /joint_states")
        self.node.get_logger().info(f'✓ Joint states being published with {len(received_joint_names)} joints')

    def test_required_joints_present(self):
        """Test that all required robot joints are present in joint states."""
        self.node.get_logger().info('Testing if all required joints are present...')

        required_arm_joints = [
            'arm_shoulder_pan_joint',
            'arm_shoulder_lift_joint',
            'arm_elbow_joint',
            'arm_wrist_1_joint',
            'arm_wrist_2_joint',
            'arm_wrist_3_joint',
        ]

        required_gripper_joints = [
            'finger_joint',
        ]

        all_required_joints = required_arm_joints + required_gripper_joints

        received_joint_names = []

        def joint_state_callback(msg):
            nonlocal received_joint_names
            received_joint_names = msg.name

        subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
            joint_state_callback,
            10
        )

        # Wait for joint state message
        timeout = 15.0
        start_time = time.time()
        while not received_joint_names and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)

        # Check that all required joints are present
        missing_joints = [j for j in all_required_joints if j not in received_joint_names]

        self.assertEqual(
            len(missing_joints), 0,
            f"Missing required joints: {missing_joints}"
        )
        self.node.get_logger().info(f'✓ All {len(all_required_joints)} required joints are present')

    def test_simulation_time_advancing(self):
        """Test that simulation time is advancing (not paused)."""
        self.node.get_logger().info('Testing if simulation time is advancing...')

        clock_values = []

        def clock_callback(msg):
            clock_values.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

        subscription = self.node.create_subscription(
            Clock,
            '/clock',
            clock_callback,
            10
        )

        # Collect clock values for 3 seconds
        start_time = time.time()
        while (time.time() - start_time) < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)

        # Check that we received multiple clock messages
        self.assertGreater(len(clock_values), 1, "Not enough clock messages received")

        # Check that time is advancing
        time_delta = clock_values[-1] - clock_values[0]
        self.assertGreater(
            time_delta, 0.5,
            f"Simulation time not advancing properly. Delta: {time_delta}s"
        )
        self.node.get_logger().info(f'✓ Simulation time is advancing (delta: {time_delta:.2f}s)')


if __name__ == '__main__':
    import sys
    # Remove ROS args before running unittest
    filtered_argv = [arg for arg in sys.argv if not arg.startswith('__')]
    unittest.main(argv=filtered_argv)
