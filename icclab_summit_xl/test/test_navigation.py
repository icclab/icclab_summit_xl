#!/usr/bin/env python3
"""
Test to verify that Nav2 navigation is working correctly.
This checks that:
1. Nav2 nodes are running
2. The robot can navigate to a goal position
3. The robot reaches the goal within tolerance
"""

import unittest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import time
import math


class NavigationTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_navigation')
        cls.node.get_logger().info('Navigation test node started')

        # Create action client for navigation
        cls.nav_to_pose_client = ActionClient(
            cls.node,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Publisher for initial pose
        cls.initial_pose_pub = cls.node.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Subscriber for odometry to track robot position
        cls.current_odom = None
        cls.odom_sub = cls.node.create_subscription(
            Odometry,
            '/odom',
            cls._odom_callback,
            10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _odom_callback(cls, msg):
        cls.current_odom = msg

    def _wait_for_odom(self, timeout=10.0):
        """Wait for odometry to be available."""
        start_time = time.time()
        while self.current_odom is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.current_odom is not None

    def _set_initial_pose(self, x=0.0, y=0.0, theta=0.0):
        """Set the initial pose of the robot."""
        self.node.get_logger().info(f'Setting initial pose to x={x}, y={y}, theta={theta}')

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.node.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = math.sin(theta / 2.0)
        initial_pose.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Set covariance (small values indicate high confidence)
        initial_pose.pose.covariance = [0.0] * 36
        initial_pose.pose.covariance[0] = 0.01  # x variance
        initial_pose.pose.covariance[7] = 0.01  # y variance
        initial_pose.pose.covariance[35] = 0.01  # yaw variance

        # Publish initial pose multiple times to ensure it's received
        for _ in range(5):
            self.initial_pose_pub.publish(initial_pose)
            time.sleep(0.1)

    def _calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def _send_nav_goal(self, x, y, theta, timeout=60.0):
        """Send a navigation goal and wait for result."""
        self.node.get_logger().info(f'Sending navigation goal: x={x}, y={y}, theta={theta}')

        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error('Navigation action server not available')
            return False, None

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)

        # Wait for goal acceptance
        start_time = time.time()
        while not send_goal_future.done() and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if not send_goal_future.done():
            self.node.get_logger().error('Goal not accepted in time')
            return False, None

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Goal rejected by action server')
            return False, None

        self.node.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if not result_future.done():
            self.node.get_logger().warn('Navigation timeout, canceling goal')
            goal_handle.cancel_goal_async()
            return False, None

        result = result_future.result()
        return True, result

    def test_nav2_action_server_available(self):
        """Test that Nav2 action server is available."""
        self.node.get_logger().info('Testing if Nav2 action server is available...')

        server_available = self.nav_to_pose_client.wait_for_server(timeout_sec=15.0)
        self.assertTrue(server_available, "Nav2 navigate_to_pose action server not available")
        self.node.get_logger().info('✓ Nav2 action server is available')

    def test_odometry_published(self):
        """Test that odometry is being published."""
        self.node.get_logger().info('Testing if odometry is being published...')

        odom_received = self._wait_for_odom(timeout=15.0)
        self.assertTrue(odom_received, "Odometry not being published")
        self.node.get_logger().info('✓ Odometry is being published')

    def test_navigation_to_goal(self):
        """Test navigation to a specific goal position."""
        self.node.get_logger().info('Testing navigation to goal...')

        # Wait for odometry
        if not self._wait_for_odom(timeout=15.0):
            self.fail("Odometry not available")

        # Set initial pose (center of tugbot_depot map, roughly)
        self._set_initial_pose(x=0.0, y=2.0, theta=0.0)

        # Wait a bit for localization to settle
        time.sleep(2.0)

        # Send navigation goal to a free space
        # For tugbot_depot map, (5.0, 2.0) should be a free space
        goal_x, goal_y, goal_theta = 3.0, 2.0, 0.0
        success, result = self._send_nav_goal(goal_x, goal_y, goal_theta, timeout=90.0)

        self.assertTrue(success, "Navigation did not complete successfully")

        # Wait for final odometry update
        time.sleep(1.0)
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Check if robot reached the goal (within tolerance)
        if self.current_odom is None:
            self.fail("Odometry not available after navigation")

        final_x = self.current_odom.pose.pose.position.x
        final_y = self.current_odom.pose.pose.position.y

        distance_to_goal = self._calculate_distance(final_x, final_y, goal_x, goal_y)

        self.node.get_logger().info(
            f'Final position: x={final_x:.2f}, y={final_y:.2f}, '
            f'distance to goal: {distance_to_goal:.2f}m'
        )

        # Check if within tolerance (0.5m should be reasonable)
        tolerance = 0.5
        self.assertLess(
            distance_to_goal, tolerance,
            f"Robot did not reach goal. Distance: {distance_to_goal:.2f}m > tolerance: {tolerance}m"
        )

        self.node.get_logger().info(
            f'✓ Robot successfully navigated to goal (error: {distance_to_goal:.2f}m)'
        )


if __name__ == '__main__':
    import sys
    # Remove ROS args before running unittest
    filtered_argv = [arg for arg in sys.argv if not arg.startswith('__')]
    unittest.main(argv=filtered_argv)
