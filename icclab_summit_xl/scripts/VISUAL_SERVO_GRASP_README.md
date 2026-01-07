# Visual Servoing Grasp System

A ROS2 implementation of visual servoing for grasping flat objects from planar surfaces using a gripper-mounted RGB-D camera.

## Overview

This system implements a robust grasping approach that combines:
1. **Lang-SAM segmentation** for initial object detection
2. **RANSAC plane fitting** to identify the table surface
3. **PCA-based grasp pose estimation** for 2-finger grippers
4. **Lightweight visual tracking** for real-time servo control
5. **PID control** for maintaining XY alignment during vertical descent

## Key Features

- **Handles reflective surfaces**: Fits table plane once at the beginning, then uses the geometric model
- **No contact sensor required**: Uses model-based height control
- **Real-time tracking**: ~30Hz control loop using template matching
- **Robust to motion**: Visual servoing compensates for arm positioning errors and vibrations

## Architecture

```
┌─────────────────┐
│  Segmentation   │  Lang-SAM for initial detection
│     Node        │  Point cloud generation
└────────┬────────┘
         │ /segmentation_mask
         │ /segmented_pointcloud
         ↓
┌─────────────────┐
│ Visual Servo    │  Plane fitting (RANSAC)
│  Grasp Node     │  PCA grasp estimation
│                 │  Template tracker
│                 │  PID controller
└────────┬────────┘
         │ /servo_node/delta_twist_cmds
         ↓
    [MoveIt Servo]
```

## Installation

The scripts are automatically installed when building the workspace:

```bash
cd /home/ros/colcon_ws
colcon build --packages-select icclab_summit_xl
source install/setup.bash
```

## Usage

### 1. Launch the System

```bash
ros2 launch icclab_summit_xl visual_servo_grasp.launch.py
```

This launches:
- Segmentation node (Lang-SAM)
- Visual servo grasp node

### 2. Launch MoveIt with Servo

The visual servo grasp node sends velocity commands to MoveIt Servo. Launch MoveIt with servo enabled:

```bash
# Launch MoveIt with both planning (move_group) and servo enabled
ros2 launch icclab_summit_xl summit_xl_move_it.launch.py use_servo:=true

# Note: Servo node starts after 5 seconds to ensure move_group is ready
# You can use both regular MoveIt planning AND servo control simultaneously
```

### 3. Trigger a Grasp

Send an object description to start the grasp sequence:

```bash
ros2 topic pub --once /start_grasp std_msgs/msg/String "data: 'red cup on table'"
```

The system will:
1. Request segmentation from Lang-SAM
2. Fit the table plane using RANSAC
3. Extract object points above the table
4. Estimate grasp pose using PCA
5. Initialize the visual tracker
6. Open the gripper
7. Approach the object with visual servoing
8. Descend vertically while maintaining XY alignment
9. Close gripper at target height
10. Lift object

### 4. Monitor Status

```bash
# Watch grasp status
ros2 topic echo /grasp_status

# Watch segmentation status
ros2 topic echo /segmentation_status

# Visualize segmentation mask
ros2 run rqt_image_view rqt_image_view /segmentation_mask

# Visualize point cloud
rviz2  # Add PointCloud2 display for /segmented_pointcloud
```

## Topics

### Subscribed Topics
- `/arm_camera/color/image_raw` (sensor_msgs/Image): RGB image from gripper camera
- `/arm_camera/depth/image_raw` (sensor_msgs/Image): Depth image from gripper camera
- `/segmented_pointcloud` (sensor_msgs/PointCloud2): Segmented object point cloud
- `/segmentation_mask` (sensor_msgs/Image): Binary segmentation mask
- `/start_grasp` (std_msgs/String): Trigger to start grasp with object description

### Published Topics
- `/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped): Velocity commands to MoveIt Servo
- `/grasp_status` (std_msgs/String): Current state of grasp execution
- `/gripper_command` (std_msgs/Bool): Gripper open/close commands
- `/segment_text` (std_msgs/String): Segmentation requests to Lang-SAM

## Parameters

Configure in the launch file or via command line:

```bash
ros2 launch icclab_summit_xl visual_servo_grasp.launch.py \
  pre_grasp_height:=0.20 \
  grasp_clearance:=0.003 \
  descent_speed:=0.015
```

### Visual Servo Grasp Node Parameters
- `pre_grasp_height` (default: 0.15): Height above table for initial approach (meters)
- `grasp_clearance` (default: 0.002): Clearance above table when grasping (meters)
- `descent_speed` (default: 0.01): Vertical descent speed (m/s)
- `servo_rate` (default: 30.0): Control loop frequency (Hz)
- `xy_tolerance` (default: 0.005): XY position tolerance (meters)

### Segmentation Node Parameters
- `model_type` (default: 'langsam'): Segmentation model ('sam', 'langsam', 'mobile_sam')
- `voxel_size` (default: 0.002): Point cloud voxel downsampling size (meters)
- `remove_outliers` (default: true): Apply statistical outlier removal

## Algorithm Details

### 1. Plane Fitting (RANSAC)
```python
# Fits plane to point cloud: ax + by + cz + d = 0
plane_model, inliers = fit_plane_ransac(points, distance_threshold=0.01)
```

### 2. Object Extraction
```python
# Extract points above the table plane
object_points = extract_points_above_plane(points, plane_model, min_height=0.002)
```

### 3. PCA-Based Grasp Pose
```python
# Compute principal axes of object
centroid = mean(object_points)
eigenvectors, eigenvalues = PCA(object_points)

# Z-axis: smallest eigenvalue (normal to flat object)
# X-axis: largest eigenvalue (longest object dimension)
# Y-axis: cross product for right-handed frame
```

### 4. Visual Servoing Loop
```python
# At ~30Hz:
center, angle, confidence = tracker.update(image)
error_xy = center - image_center
velocity = pid.update(error_xy, error_yaw)
send_velocity_command(velocity)
```

## Tracker Details

The system uses a **SimpleMaskTracker** based on template matching:
- Stores reference template from initial detection
- Uses OpenCV `matchTemplate()` with normalized correlation
- Tracks 2D position and orientation
- Returns confidence score for tracking quality
- Fallback: Re-segment with Lang-SAM if tracking is lost

For better performance, you can replace with:
- **STARK**: Fast transformer-based tracker (~60 fps)
- **OSTrack**: One-stream tracking transformer
- **SiamMask**: Siamese mask tracker
- **CSRT**: OpenCV's correlation filter tracker

## Coordinate Frames

- `arm_camera_color_optical_frame`: Camera optical frame (Z forward, Y down)
- `arm_flange`: End-effector flange frame
- Grasp poses are estimated in camera frame, then transformed to flange

## Troubleshooting

### Tracking Lost During Descent
- Adjust `tracker.template` initialization to capture more distinctive features
- Increase lighting or adjust camera exposure
- Use a more robust tracker (STARK, OSTrack)

### Grasp Height Incorrect
- Verify depth units (mm vs m)
- Check `grasp_clearance` parameter
- Verify table plane normal points upward (should be ~[0, 0, 1])

### Segmentation Quality Poor
- Adjust Lang-SAM prompt (be more specific)
- Check camera focus and lighting
- Tune `voxel_size` parameter for point cloud density

### Servo Not Moving
- Ensure MoveIt Servo is running
- Check that `/servo_node/delta_twist_cmds` is being published
- Verify servo is not in collision or joint limits

## Future Improvements

1. **Better tracker**: Integrate STARK or OSTrack for 60Hz tracking
2. **Force control**: Add force/torque sensing for contact detection
3. **Grasp quality estimation**: Score grasp poses based on object geometry
4. **Multi-object support**: Track and grasp multiple objects in sequence
5. **Gripper adaptation**: Support different gripper types and widths
6. **Learning-based pose**: Train CNN for direct grasp pose regression

## References

- [Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything): Language + SAM
- [STARK Tracker](https://github.com/researchmm/Stark): Fast visual tracking
- [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html): Real-time Cartesian control

## File Structure

```
icclab_summit_xl/
├── scripts/
│   ├── visual_servo_grasp.py          # Main grasp node
│   ├── segmentation_node.py           # Lang-SAM segmentation
│   └── VISUAL_SERVO_GRASP_README.md   # This file
├── launch/
│   └── visual_servo_grasp.launch.py   # Launch file
└── CMakeLists.txt                      # Build configuration
```

## Contact

For questions or issues, refer to the main icclab_summit_xl package documentation.
