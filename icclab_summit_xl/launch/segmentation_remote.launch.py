#!/usr/bin/env python3
"""
Remote Segmentation Launch File

Launches the remote segmentation node that connects to a LangSAM server
instead of instantiating the model locally.

Parameters:
- server_url: URL of the remote LangSAM server (default: http://localhost:8001)
- server_timeout: Request timeout in seconds (default: 30.0)
- sam_type: SAM model type to use (default: sam2.1_hiera_small)
- box_threshold: Box detection threshold (default: 0.3)
- text_threshold: Text detection threshold (default: 0.25)
- rgb_topic: RGB camera topic (default: /arm_camera/color/image_raw)
- depth_topic: Depth camera topic (default: /arm_camera/depth/image_raw)
- camera_info_topic: Camera info topic (default: /arm_camera/color/camera_info)
- voxel_size: Voxel downsampling size in meters (default: 0.002)
- remove_outliers: Enable statistical outlier removal (default: True)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    server_url_arg = DeclareLaunchArgument(
        'server_url',
        default_value='http://localhost:8001',
        description='URL of the remote LangSAM server'
    )

    server_timeout_arg = DeclareLaunchArgument(
        'server_timeout',
        default_value='30.0',
        description='Request timeout in seconds'
    )

    sam_type_arg = DeclareLaunchArgument(
        'sam_type',
        default_value='sam2.1_hiera_small',
        description='SAM model type to use on the server'
    )

    box_threshold_arg = DeclareLaunchArgument(
        'box_threshold',
        default_value='0.3',
        description='Box detection threshold'
    )

    text_threshold_arg = DeclareLaunchArgument(
        'text_threshold',
        default_value='0.25',
        description='Text detection threshold'
    )

    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/arm_camera/color/image_raw',
        description='RGB camera topic to subscribe to'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/arm_camera/depth/image_raw',
        description='Depth camera topic to subscribe to'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/arm_camera/color/camera_info',
        description='Camera info topic to subscribe to'
    )

    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.002',
        description='Voxel downsampling size in meters'
    )

    remove_outliers_arg = DeclareLaunchArgument(
        'remove_outliers',
        default_value='True',
        description='Enable statistical outlier removal'
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='remote_segmentation_node',
        description='ROS node name (use distinct names when running multiple instances)'
    )

    topic_prefix_arg = DeclareLaunchArgument(
        'topic_prefix',
        default_value='',
        description='Prefix prepended to /segment_text, /segmentation_mask, /segmented_pointcloud, /segmentation_status (e.g. "/front" for the front camera instance)'
    )

    # Remote Segmentation Node
    remote_segmentation_node = Node(
        package='icclab_summit_xl',
        executable='segmentation_node_remote.py',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'server_url': LaunchConfiguration('server_url'),
            'server_timeout': LaunchConfiguration('server_timeout'),
            'sam_type': LaunchConfiguration('sam_type'),
            'box_threshold': LaunchConfiguration('box_threshold'),
            'text_threshold': LaunchConfiguration('text_threshold'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'remove_outliers': LaunchConfiguration('remove_outliers'),
            'topic_prefix': LaunchConfiguration('topic_prefix'),
        }],
        emulate_tty=True
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(server_url_arg)
    ld.add_action(server_timeout_arg)
    ld.add_action(sam_type_arg)
    ld.add_action(box_threshold_arg)
    ld.add_action(text_threshold_arg)
    ld.add_action(rgb_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(camera_info_topic_arg)
    ld.add_action(voxel_size_arg)
    ld.add_action(remove_outliers_arg)
    ld.add_action(node_name_arg)
    ld.add_action(topic_prefix_arg)

    # Add node
    ld.add_action(remote_segmentation_node)

    return ld
