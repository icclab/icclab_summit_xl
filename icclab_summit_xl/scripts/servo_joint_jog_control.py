#!/usr/bin/env python3
"""
Joint jog keyboard control for MoveIt Servo - for debugging
This uses joint space commands instead of Cartesian twist
"""

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
import sys
import select
import termios
import tty
import threading

msg = """
MoveIt Servo JOINT JOG Control (for debugging)
---------------------------
Control individual joints:

1/2 : shoulder_pan +/-
3/4 : shoulder_lift +/-
5/6 : elbow +/-
7/8 : wrist_1 +/-
9/0 : wrist_2 +/-
-/= : wrist_3 +/-

SPACE: stop all motion
CTRL-C to quit
"""

JOINT_SPEED = 0.1  # rad/s

moveBindings = {
    '1': ('arm_shoulder_pan_joint', 1),
    '2': ('arm_shoulder_pan_joint', -1),
    '3': ('arm_shoulder_lift_joint', 1),
    '4': ('arm_shoulder_lift_joint', -1),
    '5': ('arm_elbow_joint', 1),
    '6': ('arm_elbow_joint', -1),
    '7': ('arm_wrist_1_joint', 1),
    '8': ('arm_wrist_1_joint', -1),
    '9': ('arm_wrist_2_joint', 1),
    '0': ('arm_wrist_2_joint', -1),
    '-': ('arm_wrist_3_joint', -1),
    '=': ('arm_wrist_3_joint', 1),
}


def getKey(settings, timeout=0.05):
    """Get a single keypress from stdin."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ServoJointJogControl(Node):
    def __init__(self):
        super().__init__('servo_joint_jog_control')

        # Publisher for joint jog commands
        self.jog_pub = self.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            10
        )

        # Joint names
        self.joint_names = [
            'arm_shoulder_pan_joint',
            'arm_shoulder_lift_joint',
            'arm_elbow_joint',
            'arm_wrist_1_joint',
            'arm_wrist_2_joint',
            'arm_wrist_3_joint'
        ]

        # Current velocities for each joint
        self.joint_velocities = {name: 0.0 for name in self.joint_names}

        # Create a timer to continuously publish at 50Hz
        self.timer = self.create_timer(0.02, self.publish_jog)

        self.get_logger().info('MoveIt Servo Joint Jog Control Started')
        self.get_logger().info('Publishing to /servo_node/delta_joint_cmds at 50Hz')

    def set_joint_velocity(self, joint_name, velocity):
        """Set velocity for a specific joint."""
        if joint_name in self.joint_velocities:
            self.joint_velocities[joint_name] = velocity
            self.get_logger().info(f'{joint_name}: {velocity:.2f} rad/s')

    def publish_jog(self):
        """Continuously publish the current joint jog command."""
        jog_msg = JointJog()
        jog_msg.header.stamp = self.get_clock().now().to_msg()
        jog_msg.header.frame_id = 'arm_base_link'

        jog_msg.joint_names = self.joint_names
        jog_msg.velocities = [self.joint_velocities[name] for name in self.joint_names]

        self.jog_pub.publish(jog_msg)

    def stop_all(self):
        """Stop all joints."""
        for name in self.joint_names:
            self.joint_velocities[name] = 0.0
        self.get_logger().info('All joints stopped')


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = ServoJointJogControl()

    print(msg)

    # Spin in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            key = getKey(settings, timeout=0.05)

            if key in moveBindings.keys():
                joint_name, direction = moveBindings[key]
                # Set only this joint, stop others
                node.stop_all()
                node.set_joint_velocity(joint_name, direction * JOINT_SPEED)
            elif key == ' ':
                node.stop_all()
            elif key == '\x03':  # CTRL-C
                break
            elif key == '':
                pass  # No key, maintain current velocities
            else:
                if key != '':
                    node.stop_all()

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
