#!/usr/bin/env python3
"""
Debug script to monitor servo status and see why twist commands don't work
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import ServoStatus

class ServoDebugMonitor(Node):
    def __init__(self):
        super().__init__('servo_debug_monitor')

        self.status_sub = self.create_subscription(
            ServoStatus,
            '/servo_node/status',
            self.status_callback,
            10
        )

        self.get_logger().info('Monitoring servo status...')
        self.get_logger().info('Status codes:')
        self.get_logger().info('  0 = NO_WARNING')
        self.get_logger().info('  1 = DECELERATE_FOR_SINGULARITY')
        self.get_logger().info('  2 = HALT_FOR_SINGULARITY')
        self.get_logger().info('  3 = DECELERATE_FOR_COLLISION')
        self.get_logger().info('  4 = HALT_FOR_COLLISION')
        self.get_logger().info('  5 = JOINT_BOUND')
        self.get_logger().info('  6 = INVALID')

        self.last_status = None

    def status_callback(self, msg):
        if msg.code != self.last_status:
            self.last_status = msg.code
            self.get_logger().info(f'Status changed to: {msg.code} - {msg.message}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoDebugMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
