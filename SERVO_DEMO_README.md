# MoveIt Servo Demo for Summit XL

This demo showcases MoveIt Servo functionality with the Summit XL robot equipped with a UR arm.

## Overview

MoveIt Servo allows for real-time control of the robot arm using velocity commands in joint space (joint jog). This is ideal for:
- Teleoperation
- Joystick/gamepad control
- Fine joint-level adjustments
- Compliant manipulation

## Current Status

✅ **Joint Jog Mode**: Fully functional - control individual joints with precise velocity commands
❌ **Twist (Cartesian) Mode**: Not working - IK solver has issues converting Cartesian velocities to joint velocities

**Recommendation**: Use joint jog control (`servo_joint_jog_control.py`) for reliable servo operation.

## Files Added

### Configuration Files
- `icclab_summit_xl_move_it_config/config/moveit_servo.yaml` - Servo parameters and settings

### Launch Files
- `icclab_summit_xl_move_it_config/launch/servo_demo.launch.py` - Main servo demo launcher (starts move_group, servo node, and RViz)
- `icclab_summit_xl/launch/servo_demo_full.launch.py` - Complete demo launcher with control nodes

### Demo Scripts
- `icclab_summit_xl/scripts/servo_joint_jog_control.py` - **✅ WORKING** - Keyboard control for individual joints
- `icclab_summit_xl/scripts/setup_servo.py` - Automated setup: moves arm to safe position and configures servo
- `icclab_summit_xl/scripts/servo_debug_monitor.py` - Monitor servo status for debugging
- `icclab_summit_xl/scripts/servo_keyboard_control.py` - ❌ Twist-based keyboard control (IK issues)
- `icclab_summit_xl/scripts/servo_circle_demo.py` - ❌ Twist-based circular motion (IK issues)
- `icclab_summit_xl/scripts/move_to_safe_position.py` - Standalone script to move arm away from singularities

## Usage

### Quick Start (Recommended - Joint Jog Control)

**Terminal 1** - Start simulation:
```bash
ros2 launch icclab_summit_xl summit_xl_simulation_ign.launch.py
```

**Terminal 2** - Start servo demo:
```bash
ros2 launch icclab_summit_xl_move_it_config servo_demo.launch.py
```

**Terminal 3** - Setup servo (moves arm to safe position and sets command type to JOINT_JOG):
```bash
# Modify setup_servo.py to keep command_type at 0 (JOINT_JOG), or manually:
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 0}"
```

**Terminal 4** - Start joint jog keyboard control:
```bash
ros2 run icclab_summit_xl servo_joint_jog_control.py
```

**Joint Jog Keyboard Controls:**
```
Control individual joints:
1/2 : shoulder_pan +/-
3/4 : shoulder_lift +/-
5/6 : elbow +/-
7/8 : wrist_1 +/-
9/0 : wrist_2 +/-
-/= : wrist_3 +/-

SPACE: stop all motion
CTRL-C to quit
```

### Command Types

Servo supports different command modes:
- `0` = **JOINT_JOG** - ✅ Works perfectly - control individual joints
- `1` = **TWIST** - ❌ IK issues - Cartesian control not functional
- `2` = **POSE** - Not tested

Set command type with:
```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 0}"
```

### Alternative: Cartesian Twist Control (Not Recommended - Has Issues)

**Note**: The twist-based keyboard and circle demos have inverse kinematics issues and do not work correctly. All twist commands result in the same joint motion regardless of input direction.

If you want to try anyway (for debugging):
```bash
# Set command type to TWIST
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"

# Try keyboard control (will not work as expected)
ros2 run icclab_summit_xl servo_keyboard_control.py
```

**Twist Keyboard Controls (non-functional):**
```
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
```

### 3. Circle Demo

**Setup first:**
```bash
ros2 run icclab_summit_xl setup_servo.py
```

To see an automated circular motion demo:

```bash
ros2 launch icclab_summit_xl servo_demo_full.launch.py start_circle_demo:=true
```

You can customize the circle parameters:
```bash
ros2 launch icclab_summit_xl servo_demo_full.launch.py \
    start_circle_demo:=true \
    circle_demo_plane:=xz \
    circle_demo_radius:=0.15
```

Available planes: `xy` (horizontal), `xz` (vertical side), `yz` (vertical front)

## Topics

The servo node listens to the following topics:

- `/servo_node/pose_target_cmds` (geometry_msgs/PoseStamped) - Target pose commands
- `/servo_node/twist_cmds` (geometry_msgs/TwistStamped) - Cartesian velocity commands
- `/servo_node/joint_cmds` (control_msgs/JointJog) - Joint velocity commands

And publishes to:
- `/arm_controller/joint_trajectory` (trajectory_msgs/JointTrajectory) - Trajectory commands to the controller

## Configuration

The main servo configuration is in `icclab_summit_xl_move_it_config/config/moveit_servo.yaml`.

Key parameters you might want to adjust:

- `scale.linear` - Maximum linear velocity (m/s) for Cartesian commands
- `scale.rotational` - Maximum angular velocity (rad/s) for Cartesian commands
- `scale.joint` - Maximum joint velocity for joint commands
- `publish_period` - Control loop rate (seconds)
- `check_collisions` - Enable/disable collision checking
- `collision_check_rate` - Rate for collision checking (Hz)

## Custom Control

You can create your own control nodes by publishing to the servo command topics. Here's a simple example:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class MyServoController(Node):
    def __init__(self):
        super().__init__('my_servo_controller')
        self.pub = self.create_publisher(
            TwistStamped,
            '/servo_node/twist_cmds',
            10
        )

    def send_command(self, vx, vy, vz, wx, wy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arm_tool0'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self.pub.publish(msg)

# Use the controller...
```

## Troubleshooting

### Servo node not responding
- Make sure the robot simulation or real robot is running
- Check that joint states are being published: `ros2 topic echo /joint_states`
- Verify move_group is running: `ros2 node list | grep move_group`

### Arm moves too fast or too slow
- Adjust the scale parameters in `moveit_servo.yaml`
- Modify the speed factors in the keyboard control script

### Collision checking stops motion
- The servo node stops motion when collisions are detected
- Adjust collision thresholds in `moveit_servo.yaml`
- Or disable collision checking (not recommended): set `check_collisions: false`

### Commands not being received
- Check the topic names match: `ros2 topic list | grep servo`
- Verify the message format: `ros2 topic info /servo_node/delta_twist_cmds`

## Technical Details

- **Command Type**: "unitless" - commands are in the range [-1, 1] and scaled by the parameters
- **Planning Frame**: `arm_base_link` - the reference frame for planning
- **Command Frame**: `arm_tool0` - the end-effector frame
- **Move Group**: `arm` - the planning group for the UR arm
- **Smoothing**: Uses Butterworth filter for smooth motion

## Integration with Real Robot

To use with the real Summit XL robot:

1. Start the real robot bringup:
```bash
ros2 launch icclab_summit_xl summit_xl_real.launch.py
```

2. Start the servo demo with `use_sim_time:=false`:
```bash
ros2 launch icclab_summit_xl_move_it_config servo_demo.launch.py use_sim_time:=false
```

3. Run your control node (keyboard or custom)

## Safety Notes

- Always test in simulation first
- Start with low velocity scales
- Keep the emergency stop accessible when using the real robot
- Be aware of the robot's workspace limits
- Collision checking is enabled by default but should not be the only safety measure

## References

- [MoveIt Servo Documentation](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Summit XL Robot](https://github.com/RobotnikAutomation/summit_xl_common)
