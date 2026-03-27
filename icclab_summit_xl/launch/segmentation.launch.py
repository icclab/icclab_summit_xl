#!/usr/bin/env python3
"""
MT3 Perception Only Launch File

For testing perception pipeline in isolation:
- Perception node (retrieval + pose estimation)
- Segmentation node (optional)

Use this for:
- Testing camera setup
- Verifying demonstration retrieval
- Debugging pose estimation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('icclab_summit_xl').find('icclab_summit_xl')

    # Configuration file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'seg_params.yaml'
    ])

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
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

    # Segmentation Node
    segmentation_node = Node(
        package='icclab_summit_xl',
        executable='segmentation_node.py',
        name='segmentation_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic')
            }
        ],
        emulate_tty=True
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(rgb_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(camera_info_topic_arg)

    # Add nodes
    ld.add_action(segmentation_node)

    return ld
