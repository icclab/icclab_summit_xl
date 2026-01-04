# Visual Servoing System for Summit XL

## Overview

This package implements a **teach-by-demonstration visual servoing system** for grasping with the Summit XL robot. The system allows you to teach the robot how to grasp objects by showing them once, then the robot can autonomously approach and grasp similar objects using visual feedback.

### Key Features

- ✅ **Teach-by-demonstration**: No CAD models or grasp databases required
- ✅ **Feature-based tracking**: Robust to lighting changes and viewpoint variations
- ✅ **Real-time visual servoing**: 30 Hz control loop for reactive manipulation
- ✅ **Multiple feature extractors**: ORB, SIFT (SuperPoint and XFeat ready)
- ✅ **RGBD support**: Uses depth information for accurate 3D pose estimation
- ✅ **MoveIt Servo integration**: Seamless integration with existing servo system

## How It Works

The system implements a classical visual servoing approach:

1. **Teaching Phase**:
   - User presents object to gripper-mounted camera
   - User selects region of interest (ROI) around object
   - System extracts visual features and 3D positions
   - Stores feature descriptors and relative gripper-object pose

2. **Execution Phase**:
   - System detects object in current camera view
   - Tracks learned features frame-to-frame
   - Estimates current object pose using feature correspondences
   - Computes velocity commands to servo gripper to taught pose
   - Maintains grasp approach until convergence

## Architecture

```
┌─────────────────────────────────────────────────┐
│ 1. Feature Extraction (camera_utils.py)        │
│    - Extract keypoints from RGB image           │
│    - Backproject to 3D using depth              │
└─────────────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────┐
│ 2. Feature Matching (feature_tracker.py)       │
│    - Match current features to taught features  │
│    - Use ratio test and RANSAC for robustness   │
└─────────────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────┐
│ 3. Pose Estimation (camera_utils.py)           │
│    - Estimate rigid transform from matches      │
│    - Compute relative pose error                │
└─────────────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────┐
│ 4. Visual Servoing (visual_servo_execute.py)   │
│    - Compute Cartesian velocity commands        │
│    - Send to MoveIt Servo for execution         │
└─────────────────────────────────────────────────┘
```

## Files Added

### Core Modules

- `scripts/camera_utils.py` - Camera utilities for backprojection, pose estimation, and transforms
- `scripts/feature_tracker.py` - Feature extraction and matching (ORB, SIFT, etc.)
- `scripts/visual_servo_teach.py` - ROS 2 node for teaching grasp poses
- `scripts/visual_servo_execute.py` - ROS 2 node for executing visual servoing
- `scripts/visual_servo_demo.py` - Interactive demo script

### Configuration

- `config/visual_servo.yaml` - Visual servoing parameters

### Launch Files

- `launch/visual_servo_demo.launch.py` - Main launch file for visual servoing demo

## Quick Start

### Prerequisites

Ensure you have the following Python packages installed:
```bash
pip install opencv-python scipy numpy
```

### 1. Start Simulation

In Terminal 1, start the robot simulation:
```bash
ros2 launch icclab_summit_xl summit_xl_simulation_ign.launch.py
```

### 2. Start MoveIt and Servo

In Terminal 2, start MoveIt with servo:
```bash
ros2 launch icclab_summit_xl_move_it_config servo_demo.launch.py
```

### 3. Start Visual Servoing Nodes

In Terminal 3, launch the visual servoing system:
```bash
ros2 launch icclab_summit_xl visual_servo_demo.launch.py
```

### 4. Run Demo (Interactive Mode)

In Terminal 4, run the interactive demo:
```bash
ros2 run icclab_summit_xl visual_servo_demo.py
```

Follow the menu prompts:
1. Select "1" to teach a grasp
2. Position an object in front of the camera
3. Select the ROI around the object when prompted
4. Select "2" to start servoing to the taught grasp

### Alternative: Command Line Mode

Teach a grasp:
```bash
ros2 run icclab_summit_xl visual_servo_demo.py teach
```

Execute servoing:
```bash
ros2 run icclab_summit_xl visual_servo_demo.py start
```

Stop servoing:
```bash
ros2 run icclab_summit_xl visual_servo_demo.py stop
```

Full demo (teach + execute):
```bash
ros2 run icclab_summit_xl visual_servo_demo.py demo
```

## Services

The visual servoing system exposes the following ROS 2 services:

### `/visual_servo/teach_grasp` (std_srvs/Trigger)

Captures current camera view and teaches a grasp pose:
```bash
ros2 service call /visual_servo/teach_grasp std_srvs/srv/Trigger
```

### `/visual_servo/start` (std_srvs/Trigger)

Starts visual servoing to the most recently taught grasp:
```bash
ros2 service call /visual_servo/start std_srvs/srv/Trigger
```

### `/visual_servo/stop` (std_srvs/Trigger)

Stops active visual servoing:
```bash
ros2 service call /visual_servo/stop std_srvs/srv/Trigger
```

## Topics

### Subscribed Topics

- `/camera/color/image_raw` (sensor_msgs/Image) - RGB camera feed
- `/camera/depth/image_rect_raw` (sensor_msgs/Image) - Depth image
- `/camera/color/camera_info` (sensor_msgs/CameraInfo) - Camera intrinsics

### Published Topics

- `/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped) - Velocity commands to servo
- `/visual_servo/debug_image` (sensor_msgs/Image) - Debug visualization
- `/visual_servo/teach_viz` (sensor_msgs/Image) - Teaching visualization

## Configuration

Edit `config/visual_servo.yaml` to customize parameters:

### Feature Extraction

```yaml
feature_extractor:
  type: "orb"  # Options: "orb", "sift", "superpoint", "xfeat"
  max_keypoints: 500
  keypoint_threshold: 0.005
```

- **orb**: Fast, binary features (default, works well for textured objects)
- **sift**: More robust but slower
- **superpoint**: Deep learning-based (requires installation)
- **xfeat**: State-of-the-art (requires installation)

### Control Parameters

```yaml
control:
  rate: 30.0  # Control loop frequency (Hz)
  k_position: 0.5  # Position control gain
  k_orientation: 0.3  # Orientation control gain
  max_linear_velocity: 0.1  # m/s
  max_angular_velocity: 0.3  # rad/s
```

Adjust gains for different responsiveness:
- Higher `k_position`/`k_orientation`: Faster, more aggressive
- Lower gains: Slower, smoother

### Camera Configuration

```yaml
camera:
  topic_rgb: "/camera/color/image_raw"
  topic_depth: "/camera/depth/image_rect_raw"
  depth_scale: 0.001  # Conversion factor to meters
  min_depth: 0.1  # Minimum valid depth (m)
  max_depth: 2.0  # Maximum valid depth (m)
```

## Troubleshooting

### Issue: "Too few features detected"

**Solution**:
- Ensure object has visible texture
- Improve lighting
- Adjust `keypoint_threshold` in config (lower = more features)
- Try different `feature_extractor.type`

### Issue: "Lost tracking"

**Possible causes**:
- Object moved out of view
- Lighting changed significantly
- Depth data quality poor

**Solution**:
- Increase `tracking.lost_track_threshold` for more tolerance
- Enable motion smoothing: `tracking.motion_smoothing: true`
- Check depth camera is working: `ros2 topic echo /camera/depth/image_rect_raw`

### Issue: "No taught grasps found"

**Solution**:
- Verify taught grasps directory exists: `~/taught_grasps` by default
- Check if grasps were saved successfully
- Manually specify grasp file location in code

### Issue: Servo moves too fast/slow

**Solution**:
- Adjust velocity limits in `config/visual_servo.yaml`:
  - `control.max_linear_velocity`
  - `control.max_angular_velocity`
- Adjust control gains:
  - `control.k_position`
  - `control.k_orientation`

### Issue: Camera topics not available

**Solution**:
- Verify camera is running: `ros2 topic list | grep camera`
- Check topic names match config
- For simulation, ensure Gazebo is publishing camera data

## Advanced Usage

### Using Different Feature Extractors

#### SuperPoint (Deep Learning-Based)

Install dependencies:
```bash
pip install torch torchvision
# Clone SuperPoint implementation
git clone https://github.com/magicleap/SuperPointPretrainedNetwork.git
```

Enable in config:
```yaml
feature_extractor:
  type: "superpoint"
```

#### XFeat (State-of-the-Art)

Install:
```bash
pip install xfeat
```

Enable in config:
```yaml
feature_extractor:
  type: "xfeat"
```

### Custom Control Strategies

Modify `visual_servo_execute.py` to implement custom control strategies:

```python
# Example: Add damping for smoother motion
velocity_cmd = self.camera_utils.compute_cartesian_velocity(
    current_pose, desired_pose,
    k_p=0.3,  # Lower gain
    k_r=0.2,
    max_linear_vel=0.05,  # Slower max speed
    max_angular_vel=0.2
)
```

### Integrating with Gripper Control

Add gripper commands in `visual_servo_execute.py`:

```python
# When pose is reached
if self.camera_utils.is_pose_reached(current_pose, desired_pose):
    self.get_logger().info('Grasp pose reached, closing gripper')
    # Publish gripper close command
    gripper_msg = # ... create gripper command
    self.gripper_pub.publish(gripper_msg)
```

## Performance Tips

1. **Feature Extraction**: ORB is fastest, SIFT is most robust
2. **Control Rate**: 30 Hz is good balance, 60 Hz for faster objects
3. **Matching Threshold**: Lower = more matches but more outliers
4. **RANSAC Threshold**: Adjust based on depth noise (~0.01m for RealSense)

## Technical Details

### Coordinate Frames

- **camera_color_optical_frame**: Camera frame where features are extracted
- **arm_tool0**: End-effector frame for servo commands
- All transforms use right-handed coordinate system

### Feature Matching Pipeline

1. Extract features from taught image
2. Extract features from current image
3. Match descriptors using ratio test (Lowe's criterion)
4. RANSAC to filter outliers
5. Estimate 3D-3D rigid transform

### Pose Estimation

Uses Kabsch algorithm for optimal rigid transformation:
```
R, t = argmin Σ ||R * p_taught + t - p_current||²
```

Where:
- R: 3×3 rotation matrix
- t: 3×1 translation vector
- p_taught, p_current: 3D point correspondences

## Integration with Real Robot

To use with real Summit XL:

1. Ensure camera drivers are running
2. Update camera topics in `visual_servo.yaml`
3. Set `use_sim_time:=false`:
   ```bash
   ros2 launch icclab_summit_xl visual_servo_demo.launch.py use_sim_time:=false
   ```

## Safety Considerations

⚠️ **Important Safety Notes**:

- Always test in simulation first
- Keep emergency stop accessible
- Start with low velocity limits
- Monitor for collisions (MoveIt collision checking is enabled)
- Ensure adequate lighting for feature tracking
- Verify camera calibration accuracy

## Future Enhancements

Potential improvements (not yet implemented):

- [ ] LangSAM integration for automatic object segmentation
- [ ] ICP fallback for textureless objects
- [ ] Multi-object tracking
- [ ] Grasp quality estimation
- [ ] Adaptive control gains based on tracking confidence
- [ ] TF2 integration for proper coordinate transforms
- [ ] Gripper control integration
- [ ] Trajectory recording and playback

## References

### Visual Servoing Theory

- Chaumette, F., & Hutchinson, S. (2006). "Visual servo control, Part I: Basic approaches"
- Corke, P. (2017). "Robotics, Vision and Control"

### Feature Matching

- Lowe, D. G. (2004). "Distinctive image features from scale-invariant keypoints" (SIFT)
- Rublee, E., et al. (2011). "ORB: An efficient alternative to SIFT or SURF" (ORB)
- DeTone, D., et al. (2018). "SuperPoint: Self-supervised interest point detection"

### Related Work

- [MoveIt Servo Documentation](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [ROS 2 cv_bridge](https://github.com/ros-perception/vision_opencv)

## Support

For issues or questions:
- Check troubleshooting section above
- Review ROS 2 logs: `ros2 topic echo /rosout`
- Enable debug visualization: `visualization.enabled: true`

## License

MIT License (same as package)

## Authors

- Visual Servoing System: Implementation based on discussion from Claude AI conversation
- Integration with Summit XL: ICCLab Team

---

**Last Updated**: 2026-01-04
**ROS 2 Version**: Jazzy
**Package**: icclab_summit_xl
