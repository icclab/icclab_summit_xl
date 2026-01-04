#!/usr/bin/env python3
"""
Launch file for visual servoing demo
Starts teach and execute nodes for teach-by-demonstration grasping
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """Setup launch with parameters"""

    # Package directories
    pkg_icclab_summit_xl = get_package_share_directory('icclab_summit_xl')

    # Configuration file
    visual_servo_config = PathJoinSubstitution([
        FindPackageShare('icclab_summit_xl'),
        'config',
        'visual_servo.yaml'
    ])

    # Get launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    extractor_type = LaunchConfiguration('extractor_type')
    control_rate = LaunchConfiguration('control_rate')

    # Visual servo teach node
    teach_node = Node(
        package='icclab_summit_xl',
        executable='visual_servo_teach.py',
        name='visual_servo_teach',
        output='screen',
        parameters=[
            visual_servo_config,
            {
                'use_sim_time': use_sim_time,
                'feature_extractor.type': extractor_type,
            }
        ],
    )

    # Visual servo execute node
    execute_node = Node(
        package='icclab_summit_xl',
        executable='visual_servo_execute.py',
        name='visual_servo_execute',
        output='screen',
        parameters=[
            visual_servo_config,
            {
                'use_sim_time': use_sim_time,
                'feature_extractor.type': extractor_type,
                'control.rate': control_rate,
            }
        ],
    )

    return [
        teach_node,
        execute_node,
    ]


def generate_launch_description():
    """Generate launch description for visual servoing demo"""

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'extractor_type',
            default_value='orb',
            description='Feature extractor type (orb, sift, superpoint, xfeat)'
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='30.0',
            description='Visual servoing control rate in Hz'
        ),

        # Launch setup function
        OpaqueFunction(function=launch_setup),
    ])
