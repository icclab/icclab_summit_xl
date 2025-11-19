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
✅ **Twist (Cartesian) Mode**: Fully functional - Cartesian velocity control with proper IK configuration

Both control modes are working correctly with the UR5 arm on the Summit XL robot.

**Key Configuration Requirements:**
- KDL kinematics plugin must be loaded via MoveItConfigsBuilder
- Twist commands should use `arm_flange` frame for end-effector relative control
- Singularity thresholds set appropriately for smooth operation

## Files Added

### Configuration Files
- `icclab_summit_xl_move_it_config/config/moveit_servo.yaml` - Servo parameters and settings

### Launch Files
- `icclab_summit_xl_move_it_config/launch/servo_demo.launch.py` - Main servo demo launcher (starts move_group, servo node, and RViz)
- `icclab_summit_xl/launch/servo_demo_full.launch.py` - Complete demo launcher with control nodes

### Demo Scripts
- `icclab_summit_xl/scripts/servo_joint_jog_control.py` - **✅ WORKING** - Keyboard control for individual joints
- `icclab_summit_xl/scripts/servo_keyboard_control.py` - **✅ WORKING** - Twist-based Cartesian keyboard control
- `icclab_summit_xl/scripts/servo_circle_demo.py` - **✅ WORKING** - Twist-based circular motion demo
- `icclab_summit_xl/scripts/setup_servo.py` - Automated setup: moves arm to safe position and configures servo
- `icclab_summit_xl/scripts/servo_debug_monitor.py` - Monitor servo status for debugging
- `icclab_summit_xl/scripts/move_to_safe_position.py` - Standalone script to move arm away from singularities

## Usage

### Quick Start - Cartesian Twist Control

**Terminal 1** - Start simulation:
```bash
ros2 launch icclab_summit_xl summit_xl_simulation_ign.launch.py
```

**Terminal 2** - Start servo demo:
```bash
ros2 launch icclab_summit_xl_move_it_config servo_demo.launch.py
```

**Terminal 3** - Run setup servo to move arm to safe position and enable TWIST mode:
```bash
ros2 run icclab_summit_xl setup_servo.py
```

**Terminal 4** - Start Cartesian keyboard control:
```bash
ros2 run icclab_summit_xl servo_keyboard_control.py
```

**Twist Keyboard Controls:**
```
Moving in Cartesian space (relative to end-effector):
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
```

### Circle Demo

To see an automated circular motion demo:
```bash
ros2 run icclab_summit_xl servo_circle_demo.py
```

You can customize parameters (radius in unitless scale, plane: xy/xz/yz):
```bash
ros2 run icclab_summit_xl servo_circle_demo.py --ros-args -p radius:=0.5 -p plane:=xz
```

### Command Types

Servo supports different command modes:
- `0` = **JOINT_JOG** - ✅ Works perfectly - control individual joints
- `1` = **TWIST** - ✅ Works perfectly - Cartesian velocity control
- `2` = **POSE** - Not tested

Set command type manually with:
```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
```

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

## Configuration Notes for ROS 2 Jazzy

The following configuration is critical for proper twist (Cartesian) control:

1. **Kinematics Plugin Loading**: The `MoveItConfigsBuilder` must explicitly load the kinematics configuration:
   ```python
   .robot_description_kinematics(file_path="config/kinematics.yaml")
   ```

2. **Frame Configuration**:
   - Twist commands should use `arm_flange` frame_id for end-effector relative control
   - The `apply_twist_commands_about_ee_frame: true` parameter enables EE-relative control
   - The SRDF defines the kinematic chain: `base_link="arm_base_link"` to `tip_link="arm_tool0"`

3. **Singularity Thresholds**: Appropriate values prevent unnecessary emergency stops:
   - `lower_singularity_threshold: 30.0`
   - `hard_stop_singularity_threshold: 90.0`

4. **Kinematics Solver**: Using `kdl_kinematics_plugin/KDLKinematicsPlugin` (not cached version) for better reliability

## References

- [MoveIt Servo Documentation](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Summit XL Robot](https://github.com/RobotnikAutomation/summit_xl_common)
