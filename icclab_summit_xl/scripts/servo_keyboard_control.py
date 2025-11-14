#!/usr/bin/env python3
"""
Keyboard control for MoveIt Servo
This script allows you to control the robot arm using keyboard inputs.
The servo node must be running for this script to work.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
import sys
import select
import termios
import tty

msg = """
MoveIt Servo Keyboard Control
---------------------------
Moving in Cartesian space:
        w
   a    s    d
        x

w/x : move forward/backward (X axis)
a/d : move left/right (Y axis)
q/e : move up/down (Z axis)
i/k : rotate around X axis (roll)
j/l : rotate around Y axis (pitch)
u/o : rotate around Z axis (yaw)

CTRL-C to quit
"""

# Movement speed factors
LINEAR_SPEED = 0.5   # m/s
ANGULAR_SPEED = 0.5  # rad/s

moveBindings = {
    'w': (1, 0, 0, 0, 0, 0),     # Forward (X+)
    'x': (-1, 0, 0, 0, 0, 0),    # Backward (X-)
    'a': (0, 1, 0, 0, 0, 0),     # Left (Y+)
    'd': (0, -1, 0, 0, 0, 0),    # Right (Y-)
    'q': (0, 0, 1, 0, 0, 0),     # Up (Z+)
    'e': (0, 0, -1, 0, 0, 0),    # Down (Z-)
    'i': (0, 0, 0, 1, 0, 0),     # Roll+ (rotate around X)
    'k': (0, 0, 0, -1, 0, 0),    # Roll- (rotate around X)
    'j': (0, 0, 0, 0, 1, 0),     # Pitch+ (rotate around Y)
    'l': (0, 0, 0, 0, -1, 0),    # Pitch- (rotate around Y)
    'u': (0, 0, 0, 0, 0, 1),     # Yaw+ (rotate around Z)
    'o': (0, 0, 0, 0, 0, -1),    # Yaw- (rotate around Z)
}


def getKey(settings):
    """Get a single keypress from stdin."""
    tty.setraw(sys.stdin.fileno())
    # Use select to wait for input with a timeout
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ServoKeyboardControl(Node):
    def __init__(self):
        super().__init__('servo_keyboard_control')

        # Publisher for Cartesian twist commands
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/twist_cmds',
            10
        )

        self.get_logger().info('MoveIt Servo Keyboard Control Node Started')
        self.get_logger().info('Publishing twist commands to /servo_node/twist_cmds')

    def publish_twist(self, x, y, z, roll, pitch, yaw):
        """Publish a twist command."""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'arm_base_link'  # Use planning frame

        # Linear velocities
        twist_msg.twist.linear.x = x * LINEAR_SPEED
        twist_msg.twist.linear.y = y * LINEAR_SPEED
        twist_msg.twist.linear.z = z * LINEAR_SPEED

        # Angular velocities
        twist_msg.twist.angular.x = roll * ANGULAR_SPEED
        twist_msg.twist.angular.y = pitch * ANGULAR_SPEED
        twist_msg.twist.angular.z = yaw * ANGULAR_SPEED

        self.twist_pub.publish(twist_msg)


def main(args=None):
    # Store terminal settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = ServoKeyboardControl()

    print(msg)

    try:
        while rclpy.ok():
            key = getKey(settings)

            if key in moveBindings.keys():
                x, y, z, roll, pitch, yaw = moveBindings[key]
                node.publish_twist(x, y, z, roll, pitch, yaw)
            elif key == '\x03':  # CTRL-C
                break
            else:
                # Publish zero velocity when no key is pressed
                if key == '':
                    node.publish_twist(0, 0, 0, 0, 0, 0)

            # Spin once to process callbacks
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(e)

    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        # Send final zero velocity command
        node.publish_twist(0, 0, 0, 0, 0, 0)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
