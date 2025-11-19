#!/usr/bin/env python3
"""
Simple circular motion demo for MoveIt Servo
This script demonstrates MoveIt Servo by commanding the end-effector
to move in a circular pattern in Cartesian space.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math


class ServoCircleDemo(Node):
    def __init__(self):
        super().__init__('servo_circle_demo')

        # Parameters
        # Note: Using speed_units commands - values are in m/s and rad/s
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('radius', 0.05)  # Circle radius in meters
        self.declare_parameter('angular_speed', 0.3)  # Angular speed in rad/s for circular motion
        self.declare_parameter('plane', 'xy')  # 'xy', 'xz', or 'yz'

        self.publish_rate = self.get_parameter('publish_rate').value
        self.radius = self.get_parameter('radius').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.plane = self.get_parameter('plane').value

        # Publisher for Cartesian twist commands
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        # Timer for publishing commands
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.time = 0.0
        self.dt = 1.0 / self.publish_rate

        self.get_logger().info('MoveIt Servo Circle Demo Started')
        self.get_logger().info(f'Moving in a circle with radius {self.radius}m in {self.plane} plane')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')

    def timer_callback(self):
        """Publish circular motion commands."""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'arm_flange'  # EE frame for end-effector relative control

        # Calculate circular velocities based on the selected plane
        # The velocity vector should be perpendicular to the position vector
        # for circular motion: v = omega x r

        if self.plane == 'xy':
            # Circle in XY plane (horizontal)
            # vx = -omega * r * sin(omega * t)
            # vy =  omega * r * cos(omega * t)
            twist_msg.twist.linear.x = -self.angular_speed * self.radius * math.sin(self.angular_speed * self.time)
            twist_msg.twist.linear.y = self.angular_speed * self.radius * math.cos(self.angular_speed * self.time)
            twist_msg.twist.linear.z = 0.0

        elif self.plane == 'xz':
            # Circle in XZ plane (vertical, side view)
            twist_msg.twist.linear.x = -self.angular_speed * self.radius * math.sin(self.angular_speed * self.time)
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = self.angular_speed * self.radius * math.cos(self.angular_speed * self.time)

        elif self.plane == 'yz':
            # Circle in YZ plane (vertical, front view)
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = -self.angular_speed * self.radius * math.sin(self.angular_speed * self.time)
            twist_msg.twist.linear.z = self.angular_speed * self.radius * math.cos(self.angular_speed * self.time)

        else:
            self.get_logger().error(f'Invalid plane: {self.plane}. Use xy, xz, or yz.')
            return

        # No rotation
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        self.twist_pub.publish(twist_msg)

        # Update time
        self.time += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = ServoCircleDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send final zero velocity command
        twist_msg = TwistStamped()
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'arm_flange'
        node.twist_pub.publish(twist_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
