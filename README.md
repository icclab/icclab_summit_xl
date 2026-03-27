# icclab_summit_xl

ROS2 (Jazzy) package for the ICCLab Summit XL robot platform with UR5 arm, Robotiq gripper, and perception capabilities. Supports simulation (Gazebo Ignition/Harmonic), real robot operation, autonomous navigation (Nav2), manipulation (MoveIt2/Servo), SLAM (rtabmap), and vision-based segmentation (SAM/LangSAM).

**Maintainer**: Giovanni Toffetti (toff@zhaw.ch)

---

## Table of Contents

- [Core Dependencies](#core-dependencies)
- [Optional Feature Dependencies](#optional-feature-dependencies)
- [Launch Files](#launch-files)
  - [Simulation](#simulation)
  - [Navigation](#navigation)
  - [Manipulation](#manipulation)
  - [Perception & Segmentation](#perception--segmentation)
  - [Real Robot](#real-robot)
  - [Utilities & Testing](#utilities--testing)
- [Scripts](#scripts)
- [LangSAM Segmentation Server Setup](#langsam-segmentation-server-setup)
- [Configuration Files](#configuration-files)

---

## Core Dependencies

These are required for the standard simulation and manipulation stack and are declared in `package.xml`:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-moveit ros-jazzy-moveit-servo \
  ros-jazzy-ur ros-jazzy-ur-simulation-gz \
  ros-jazzy-robotiq-description \
  ros-jazzy-depth-image-proc \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image ros-jazzy-ros-gz-sim \
  ros-jazzy-rviz2 ros-jazzy-joint-state-publisher \
  ros-jazzy-realsense2-description
```

---

## Optional Feature Dependencies

These dependencies are **not** declared in `package.xml` (to keep the default install lean) but are required for specific features. Install only what you need.

### SLAM with rtabmap

Required by `rtabmap.launch.py`.

```bash
sudo apt install ros-jazzy-rtabmap-ros
```

### Laser Scan Merging

Required by `summit_xl_nav2.launch.py` for dual-LIDAR fusion.

```bash
sudo apt install ros-jazzy-ros2-laser-scan-merger
```

### OAK-D Camera (Luxonis)

Required by `real/oak.camera.launch.py`.

```bash
sudo apt install ros-jazzy-depthai-ros
```

Or follow the official installation guide at: https://docs.luxonis.com/software/ros/depthai-ros/

### Astra Mini Camera (Orbbec)

Required by `real/astra_mini.launch.py`.

```bash
sudo apt install ros-jazzy-astra-camera
```

Or build from source: https://github.com/orbbec/ros_astra_camera

### Segmentation (Local Mode)

Required by `segmentation.launch.py` and `segmentation_node.py`.

Python packages (install in the ROS Python environment or a venv):

```bash
pip install open3d
pip install git+https://github.com/facebookresearch/segment-anything.git
# For LangSAM (includes SAM2 + grounding DINO):
pip install lang-segment-anything
```

> **Note**: For GPU-accelerated local segmentation, see the [LangSAM Segmentation Server Setup](#langsam-segmentation-server-setup) section, which describes the recommended installation using a dedicated venv.

---

## Launch Files

### Simulation

#### `summit_xl_simulation_ign.launch.py`

Full Gazebo Ignition/Harmonic simulation stack. Spawns the robot, bridges ROS2↔Gazebo topics, and starts all controllers.

```bash
ros2 launch icclab_summit_xl summit_xl_simulation_ign.launch.py
```

**Arguments**:

| Argument | Default | Description |
|---|---|---|
| `robot_id` | `summit` | Robot namespace identifier |
| `robot_xacro` | *(icclab xacro)* | Path to URDF xacro file |
| `world` | `tugbot_depot` | Gazebo world URL |
| `headless` | `false` | Run without Gazebo GUI |

**Notes**:
- Controller startup is sequenced with delays (joint_state_broadcaster at 2s, arm_controller at 8s, gripper at ~10s) to avoid race conditions with `gz_ros2_control`.
- Generates corrected pointclouds via `depth_image_proc` as a workaround for the Gazebo Harmonic rgbd_camera optical frame bug.
- Publishes compressed image topics (15s delay) for MCP server compatibility.

---

### Navigation

#### `summit_xl_nav2.launch.py`

Nav2 autonomous navigation with dual LIDAR scan merging.

**Requires**: `ros2-laser-scan-merger` (see [Optional Dependencies](#optional-feature-dependencies))

```bash
ros2 launch icclab_summit_xl summit_xl_nav2.launch.py
```

**Arguments**:

| Argument | Default | Description |
|---|---|---|
| `map` | `tugbot_depot.yaml` | Nav2 map YAML file (from `maps/`) |
| `params_file` | `nav2_params_sim.yaml` | Nav2 configuration file |
| `rviz` | `true` | Launch RViz |

Available maps: `empty`, `icclab`, `tb3_house`, `tugbot_depot`, `willow_garage`.

#### `map_save.launch.py`

Save a Nav2-generated map to disk.

```bash
ros2 launch icclab_summit_xl map_save.launch.py
```

Saves to `my_map.yaml` / `my_map.pgm` by subscribing to `/summit/map`.

---

### Manipulation

#### `summit_xl_move_it.launch.py`

MoveIt2 `move_group` with optional real-time Servo control.

```bash
ros2 launch icclab_summit_xl summit_xl_move_it.launch.py
# With servo enabled:
ros2 launch icclab_summit_xl summit_xl_move_it.launch.py use_servo:=true
```

**Arguments**:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `use_servo` | `false` | Enable MoveIt Servo node |

**Note**: Namespace and TF remappings are intentionally omitted for MoveItPy (Jazzy) compatibility.

#### `servo_demo_full.launch.py`

Complete MoveIt Servo demo with optional keyboard or autonomous circle control.

```bash
ros2 launch icclab_summit_xl servo_demo_full.launch.py start_keyboard_control:=true
ros2 launch icclab_summit_xl servo_demo_full.launch.py start_circle_demo:=true circle_demo_plane:=xz
```

**Arguments**:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `start_rviz` | `true` | Launch RViz |
| `start_keyboard_control` | `false` | Enable keyboard teleoperation |
| `start_circle_demo` | `false` | Enable autonomous circle motion |
| `circle_demo_plane` | `xy` | Plane for circle: `xy`, `xz`, or `yz` |
| `circle_demo_radius` | `0.1` | Circle radius in meters |

#### `move_arm_to_pose.launch.py`

Move the arm to a specific Cartesian pose (C++ node).

```bash
ros2 launch icclab_summit_xl move_arm_to_pose.launch.py x:=0.7 y:=0.02 z:=1.3
```

**Arguments**: `robot_id`, `x`, `y`, `z`, `rx`, `ry`, `rz`, `rw` (quaternion).

---

### Perception & Segmentation

#### `rtabmap.launch.py`

RGB-D SLAM using rtabmap for real-time mapping and localization.

**Requires**: `ros-jazzy-rtabmap-ros` (see [Optional Dependencies](#optional-feature-dependencies))

```bash
ros2 launch icclab_summit_xl rtabmap.launch.py
# Localization-only mode (no map building):
ros2 launch icclab_summit_xl rtabmap.launch.py localization:=true
```

Configured for 2D SLAM (Force3DoF). Uses the arm RGBD camera. Deletes the previous database on startup by default.

#### `segmentation.launch.py`

Run SAM/LangSAM image segmentation **locally** (requires GPU for reasonable performance).

**Requires**: `open3d`, `segment-anything` or `lang-segment-anything` Python packages.

```bash
ros2 launch icclab_summit_xl segmentation.launch.py
```

**Arguments**:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `rgb_topic` | `/arm_camera/color/image_raw` | Input RGB image |
| `depth_topic` | `/arm_camera/depth/image_raw` | Input depth image |
| `camera_info_topic` | `/arm_camera/color/camera_info` | Camera intrinsics |

Model configuration (device, model_type, etc.) is read from `config/seg_params.yaml`.

#### `segmentation_remote.launch.py`

Connect to a **remote** LangSAM HTTP server for distributed GPU processing. Use this when the GPU is on a separate machine.

**Requires**: Running LangSAM server (see [LangSAM Segmentation Server Setup](#langsam-segmentation-server-setup))

```bash
ros2 launch icclab_summit_xl segmentation_remote.launch.py server_url:=http://192.168.1.100:8000
```

**Arguments**:

| Argument | Default | Description |
|---|---|---|
| `server_url` | `http://localhost:8001` | LangSAM server address |
| `server_timeout` | `30.0` | Request timeout in seconds |
| `sam_type` | `sam2.1_hiera_small` | SAM2 model variant |
| `box_threshold` | `0.3` | Grounding DINO box threshold |
| `text_threshold` | `0.25` | Grounding DINO text threshold |
| `voxel_size` | `0.002` | Pointcloud voxel downsampling (meters) |
| `remove_outliers` | `true` | Statistical outlier removal |

#### `depth_image_proc_arm_camera.launch.py`

Standalone pointcloud generation for the arm camera (workaround for Gazebo Harmonic rgbd_camera optical frame bug). Normally included automatically by `summit_xl_simulation_ign.launch.py`.

---

### Real Robot

#### `real/summit_xl_real.launch.py`

Full bringup for the physical Summit XL robot.

#### `real/ur_control.launch.py`

Connect to a physical UR5 via `ros2_control`.

**Key Arguments**:

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur5` | UR model: `ur3`, `ur5`, `ur10`, `ur5e`, etc. |
| `robot_ip` | `192.168.0.210` | UR controller IP address |
| `use_fake_hardware` | `false` | Use mock hardware (no physical robot) |
| `headless_mode` | `true` | Connect without UR teach pendant |

#### `real/arm_controller.launch.py`

Wrapper for `ur_control.launch.py` with the standard ICCLab UR5 configuration (IP: `192.168.0.210`, TF prefix: `arm_`).

#### `real/astra_mini.launch.py`

Orbbec Astra Mini RGB-D camera with pointcloud generation. Image compression is disabled by default (floods the network).

**Requires**: `ros-jazzy-astra-camera`

#### `real/oak.camera.launch.py`

Luxonis OAK-D PRO camera with configurable RGB-D streams, rectification, and pointcloud generation. Supports RealSense compatibility mode.

**Requires**: `ros-jazzy-depthai-ros`

---

### Utilities & Testing

#### `rviz.launch.py`

Launch RViz2 with a saved configuration.

```bash
ros2 launch icclab_summit_xl rviz.launch.py rviz_config:=robot.rviz
```

#### `regression_test.launch.py`

Orchestrates a full system regression test: starts simulation → Nav2 → MoveIt → Servo → runs test suite.

```bash
ros2 launch icclab_summit_xl regression_test.launch.py headless:=true
```

**Arguments**: `test` (all/simulation/navigation/moveit), `headless`, `timeout`.

#### `test_runner.launch.py`

Run individual tests assuming simulation, Nav2, and MoveIt are already running.

---

## Scripts

All scripts are in the `scripts/` directory.

### `segmentation_node.py`

Local SAM/LangSAM segmentation node. Subscribes to synchronized RGB-D images, runs segmentation, and publishes the segmentation mask and a filtered pointcloud.

**Topics subscribed**: `rgb_topic`, `depth_topic`, `camera_info_topic` (configured via params)
**Topics published**:
- `/segmentation_mask` — binary mask (`sensor_msgs/Image`, uint8)
- `/segmented_pointcloud` — filtered pointcloud (`sensor_msgs/PointCloud2`)
- `/segmentation_status` — status string

**Segmentation triggers**:
- `/segment_text` — text prompt (LangSAM only)
- `/segment_point` — pixel coordinate prompt (SAM only)
- `/segment_bbox` — bounding box prompt (SAM only)

### `segmentation_node_remote.py`

Lightweight ROS2 node that sends RGB images to a remote LangSAM HTTP server and processes the returned masks. Identical output interface to `segmentation_node.py`. Use when the GPU is on a separate machine.

### `servo_keyboard_control.py`

Real-time Cartesian teleoperation via keyboard. Publishes `TwistStamped` at 50 Hz to `/servo_node/delta_twist_cmds`.

**Controls**: `w/x` (X±), `a/d` (Y±), `q/e` (Z±), `i/k` (roll±), `j/l` (pitch±), `u/o` (yaw±), `SPACE` (stop).

### `summit_arm.py`

High-level Python wrapper around MoveItPy for arm and gripper control. Used by `moveit_py_example.py`.

### `moveit_py_example.py`

Demo script showing arm/gripper manipulation using named SRDF configurations (home, up, docked, look_forward, open, closed).

### `nav2_action_client.py`

Action client for sending navigation goals to Nav2.

### `servo_circle_demo.py`

Autonomous circular end-effector motion for Servo testing. Parameters: `plane` (xy/xz/yz), `radius`.

### `servo_joint_jog_control.py`

Joint-level Servo control (alternative to Cartesian).

### `servo_debug_monitor.py`

Display Servo node status and diagnostics.

### `setup_servo.py`

Configure and initialize the Servo node.

### `gripper_attach_node.py`

ROS2 node for dynamic gripper attachment (Gazebo plugin interface).

---

## LangSAM Segmentation Server Setup

The segmentation nodes use [lang-segment-anything](https://github.com/luca-medeiros/lang-segment-anything), which combines SAM2 and Grounding DINO for language-guided segmentation. The server runs in a dedicated Python venv at `~/rap/lang-segment-anything/` for GPU isolation from ROS.

This package ships patches and a custom server in `langsam/`:

```
langsam/
├── install_langsam.sh       # automated install + patch script
├── server_ros.py            # ROS-specific LitServe server (JSON endpoint, port 8001)
└── patches/
    ├── lang_sam.py          # adds gdino_model_id parameter to LangSAM.__init__
    └── gdino.py             # propagates model_id through build_model; default: grounding-dino-tiny
```

### Why patches?

Two changes are needed beyond the upstream release:

1. **`gdino.py` default model** (`grounding-dino-tiny`): the upstream default was `grounding-dino-base`, which fails to load on Ubuntu 24.04 / ROS Jazzy due to a `transformers` version conflict. The fix is to use `-tiny` as the default.

2. **`lang_sam.py` + `gdino.py` — `gdino_model_id` parameter**: the ROS server (`server_ros.py`) uses `grounding-dino-base` for better accuracy but needs to pass the model ID through the constructor. This patch adds a `gdino_model_id` argument to `LangSAM.__init__` and `GDINO.build_model`.

3. **`server_ros.py`** — a ROS-specific server that:
   - Runs on port **8001** (avoids conflict with the original `server.py` on 8000)
   - Adds an `output_format=json` response mode that returns base64-encoded masks + bounding boxes as JSON (required by `segmentation_node_remote.py` for pointcloud generation)
   - Uses `grounding-dino-base` for better detection accuracy

### Automated Installation

**Prerequisites**: CUDA-capable GPU, CUDA toolkit, Python 3.12.

```bash
cd ~/colcon_ws/src/icclab_summit_xl/langsam
bash install_langsam.sh
```

The script will:
- Clone the repo to `~/rap/lang-segment-anything` (skips if already present)
- Install `uv` if not found
- Create a Python 3.12 venv with `--system-site-packages`
- Install PyTorch with auto-detected CUDA version
- Install lang-segment-anything with `uv pip install -e .`
- Apply the two source patches
- Copy `server_ros.py` into the repo

### Manual Installation

If you prefer to install step by step:

```bash
# 1. Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# 2. Clone
mkdir -p ~/rap && cd ~/rap
git clone https://github.com/luca-medeiros/lang-segment-anything.git
cd lang-segment-anything

# 3. Create venv (system-site-packages lets the venv see ROS Python packages)
uv venv --python 3.12 --system-site-packages .venv
source .venv/bin/activate

# 4. Install PyTorch with CUDA (replace cu128 with your CUDA version, e.g. cu118, cu121)
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128

# 5. Install lang-segment-anything
uv pip install -e .

# 6. Apply patches
ICCLAB=~/colcon_ws/src/icclab_summit_xl/langsam
cp $ICCLAB/patches/lang_sam.py lang_sam/lang_sam.py
cp $ICCLAB/patches/gdino.py lang_sam/models/gdino.py
cp $ICCLAB/server_ros.py lang_sam/server_ros.py

# 7. Verify GPU
python -c "import torch; print('CUDA:', torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

### Running the ROS Server

```bash
cd ~/rap/lang-segment-anything
source .venv/bin/activate

# Start the ROS-compatible server on port 8001
python3 -m lang_sam.server_ros
```

On first run the server downloads SAM2 weights (~160 MB for `sam2.1_hiera_small`) and Grounding DINO Base (~680 MB). Both are cached in `~/.cache/huggingface/`.

> The original `server.py` / `app.py` (port 8000, image-only responses) still works but is **not** compatible with `segmentation_node_remote.py`, which requires the JSON endpoint provided by `server_ros.py`.

### Using with the ROS2 Segmentation Node

```bash
# Server on the same machine (default)
ros2 launch icclab_summit_xl segmentation_remote.launch.py

# Server on a different machine
ros2 launch icclab_summit_xl segmentation_remote.launch.py server_url:=http://192.168.1.100:8001
```

### Sending Segmentation Requests

```bash
ros2 topic pub --once /segment_text std_msgs/msg/String "data: 'red box'"
```

The node captures the next synchronized RGB-D frame, sends it to the server, and publishes results on `/segmented_pointcloud` and `/segmentation_mask`.

---

## Configuration Files

| File | Purpose |
|---|---|
| `seg_params.yaml` | Segmentation node parameters (model_type, device, topics) |
| `nav2_params_sim.yaml` | Nav2 parameters for simulation |
| `nav2_params_real.yaml` | Nav2 parameters for real robot |
| `ur_controllers.yaml` | UR5 joint controllers (simulation) |
| `ur_controllers_real.yaml` | UR5 joint controllers (real robot) |
| `joint_limits.yaml` | UR5 joint velocity/acceleration limits |
| `joint_limits_simulation.yaml` | Relaxed limits for simulation |
| `lidar_front.yaml` / `lidar_rear.yaml` | LIDAR topic and frame configuration |
| `ign_gazebo_bridge.yaml` | ROS2↔Gazebo topic bridge config |
| `ign_gazebo_bridge_depth_image.yaml` | Bridge config without pointcloud (Gazebo bug workaround) |
| `astra_mini_params.yaml` | Astra Mini camera parameters |
| `ur5_calibrated_kinematics.yaml` | Calibrated UR5 kinematics for real robot |
| `gripper_controller.yaml` | Robotiq gripper controller config |
