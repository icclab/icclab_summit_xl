"""
Option B: Generate arm_camera pointcloud from depth + color images using depth_image_proc.

This produces a properly-framed pointcloud in the optical frame convention,
working around the Gazebo Harmonic rgbd_camera bug where pointcloud XYZ data
is in Gazebo convention (X-fwd) but labeled with the optical frame (Z-fwd).

Requires:
  - luxonis_camera_gazebo_depth_image.urdf.xacro (with optical_frame_id)
  - ign_gazebo_bridge_depth_image.yaml (without arm_camera/points bridge)

Usage:
  Add to your main launch or run separately:
    ros2 launch icclab_summit_xl depth_image_proc_arm_camera.launch.py
"""

import launch
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='depth_image_proc',
            executable='point_cloud_xyzrgb_node',
            name='arm_camera_pointcloud',
            remappings=[
                ('rgb/image_rect_color', '/arm_camera/color/image_raw'),
                ('rgb/camera_info', '/arm_camera/color/camera_info'),
                ('depth_registered/image_rect', '/arm_camera/depth/image_raw'),
                ('points', '/arm_camera/points'),
            ],
            parameters=[{'use_sim_time': True}],
        ),
    ])
