#!/usr/bin/env python3
"""
Keyboard control for MoveIt Servo
This script allows you to control the robot arm using keyboard inputs.
The servo node must be running for this script to work.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty
import threading

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

+/- : increase/decrease speed
SPACE: stop all motion
CTRL-C to quit
"""

# Movement speed in actual units (m/s and rad/s)
# With servo config: command_in_type="speed_units"
LINEAR_SPEED = 0.02   # Linear velocity in m/s
ANGULAR_SPEED = 0.1   # Angular velocity in rad/s
SPEED_INCREMENT = 0.01  # Speed adjustment step

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


def getKey(settings, timeout=0.1):
    """Get a single keypress from stdin."""
    tty.setraw(sys.stdin.fileno())
    # Use select to wait for input with a timeout
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
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
            '/servo_node/delta_twist_cmds',
            10
        )

        # Current velocity command
        self.current_twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, z, roll, pitch, yaw
        self.linear_speed = LINEAR_SPEED
        self.angular_speed = ANGULAR_SPEED

        # Create a timer to continuously publish twist commands at 50Hz
        # This matches the servo publish_period of 0.02s
        self.timer = self.create_timer(0.02, self.publish_twist)

        self.get_logger().info('MoveIt Servo Keyboard Control Node Started')
        self.get_logger().info('Publishing twist commands to /servo_node/delta_twist_cmds at 50Hz')
        self.get_logger().info(f'Initial velocities: linear={self.linear_speed:.3f} m/s, angular={self.angular_speed:.3f} rad/s')

    def set_twist(self, x, y, z, roll, pitch, yaw):
        """Set the current twist command."""
        self.current_twist = [x, y, z, roll, pitch, yaw]

    def publish_twist(self):
        """Continuously publish the current twist command."""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'arm_flange'  # EE frame for apply_twist_commands_about_ee_frame=true

        # Linear velocities
        twist_msg.twist.linear.x = self.current_twist[0] * self.linear_speed
        twist_msg.twist.linear.y = self.current_twist[1] * self.linear_speed
        twist_msg.twist.linear.z = self.current_twist[2] * self.linear_speed

        # Angular velocities
        twist_msg.twist.angular.x = self.current_twist[3] * self.angular_speed
        twist_msg.twist.angular.y = self.current_twist[4] * self.angular_speed
        twist_msg.twist.angular.z = self.current_twist[5] * self.angular_speed

        self.twist_pub.publish(twist_msg)

    def increase_speed(self):
        """Increase movement speed."""
        self.linear_speed += SPEED_INCREMENT
        self.angular_speed += SPEED_INCREMENT * 2
        self.get_logger().info(f'Velocities increased - linear: {self.linear_speed:.3f} m/s, angular: {self.angular_speed:.3f} rad/s')

    def decrease_speed(self):
        """Decrease movement speed."""
        self.linear_speed = max(0.001, self.linear_speed - SPEED_INCREMENT)
        self.angular_speed = max(0.01, self.angular_speed - SPEED_INCREMENT * 2)
        self.get_logger().info(f'Velocities decreased - linear: {self.linear_speed:.3f} m/s, angular: {self.angular_speed:.3f} rad/s')

    def stop(self):
        """Stop all motion."""
        self.set_twist(0, 0, 0, 0, 0, 0)
        self.get_logger().info('Motion stopped')


def main(args=None):
    # Store terminal settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = ServoKeyboardControl()

    print(msg)

    # Spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            key = getKey(settings, timeout=0.05)

            if key in moveBindings.keys():
                # Set the velocity based on key press
                x, y, z, roll, pitch, yaw = moveBindings[key]
                node.set_twist(x, y, z, roll, pitch, yaw)
            elif key == ' ':  # Space bar - stop
                node.stop()
            elif key == '+' or key == '=':  # Increase speed
                node.increase_speed()
            elif key == '-' or key == '_':  # Decrease speed
                node.decrease_speed()
            elif key == '\x03':  # CTRL-C
                break
            elif key == '':
                # No key pressed - maintain current velocity
                pass
            else:
                # Unknown key - stop motion for safety
                if key != '':
                    node.stop()

    except Exception as e:
        print(e)

    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        # Send final zero velocity command
        node.stop()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
