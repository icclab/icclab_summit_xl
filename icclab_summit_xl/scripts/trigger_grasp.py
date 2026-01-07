#!/usr/bin/env python3
"""
Simple script to trigger a grasp by publishing to /start_grasp topic.

Usage:
    ros2 run icclab_summit_xl trigger_grasp.py "red cup on table"
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run icclab_summit_xl trigger_grasp.py <object_description>")
        print("Example: ros2 run icclab_summit_xl trigger_grasp.py 'red cup on table'")
        sys.exit(1)

    object_description = ' '.join(sys.argv[1:])

    rclpy.init(args=args)
    node = Node('grasp_trigger')

    # Create publisher
    pub = node.create_publisher(String, '/start_grasp', 10)

    # Wait for subscriber
    print(f"Waiting for visual_servo_grasp node to be ready...")
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Publish grasp request
    msg = String()
    msg.data = object_description

    print(f"Triggering grasp for: '{object_description}'")
    pub.publish(msg)

    # Spin briefly to ensure message is sent
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Grasp triggered successfully!")
    print("Monitor progress with: ros2 topic echo /grasp_status")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
