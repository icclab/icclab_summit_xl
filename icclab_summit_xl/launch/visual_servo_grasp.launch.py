#!/usr/bin/env python3
"""
Launch file for Visual Servoing Grasp system

This launches:
1. Segmentation node (Lang-SAM)
2. Visual servo grasp node
3. Optional: RViz for visualization

On shutdown, restores MoveIt Servo to JOINT_JOG mode.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='langsam',
        description='Segmentation model type: sam, langsam, or mobile_sam'
    )

    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/arm_camera/color/image_raw',
        description='RGB image topic'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/arm_camera/depth/image_raw',
        description='Depth image topic'
    )

    # # Segmentation node
    # segmentation_node = Node(
    #     package='icclab_summit_xl',
    #     executable='segmentation_node.py',
    #     name='segmentation_node',
    #     output='screen',
    #     parameters=[{
    #         'model_type': LaunchConfiguration('model_type'),
    #         'rgb_topic': LaunchConfiguration('rgb_topic'),
    #         'depth_topic': LaunchConfiguration('depth_topic'),
    #         'voxel_size': 0.002,  # 2mm voxel downsampling
    #         'remove_outliers': True,
    #     }]
    # )

    # Visual servo grasp node
    visual_servo_node = Node(
        package='icclab_summit_xl',
        executable='visual_servo_grasp.py',
        name='visual_servo_grasp',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'pre_grasp_height': 0.25,       # Fingertip distance to table plane at pre-grasp (meters)
            'grasp_clearance': 0.17,        # Fingertip distance to table plane when grasping (meters)
            'descent_speed': 0.01,          # Vertical descent speed (m/s)
            'approach_speed': 0.05,         # Approach speed (m/s)
            'servo_rate': 30.0,             # Control loop rate (Hz)
            'xy_tolerance': 0.005,          # Position tolerance for XY alignment (meters)
            'state_timeout': 90.0,          # Timeout for each state (seconds)
            'stall_threshold': 0.001,       # Joint velocity threshold for stall detection (rad/s)
            'stall_time': 1.0,              # Time without motion before stall (seconds)
            'max_stall_retries': 3,         # Max retries on stall before aborting
            # Camera-to-fingertip offset in camera optical frame (meters)
            # These offsets describe where the fingertips are relative to the camera
            # X: forward (positive = fingertips ahead of camera)
            # Y: left (positive = fingertips left of camera)
            # Z: down in optical frame (positive = fingertips below/closer to table)
            'fingertip_offset_x': 0.17,    # Fingertips ahead of camera
            'fingertip_offset_y': -0.031,   # Fingertips slightly right of camera
            'fingertip_offset_z': 0.080,    # Fingertips below camera (closer to table)
        }]
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('icclab_summit_xl'),
            'rviz',
            'visual_servo_grasp.rviz'
        ])]
    )

    # Shutdown handler to restore JOINT_JOG mode
    restore_joint_control = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/servo_node/switch_command_type',
                        'moveit_msgs/srv/ServoCommandType',
                        '{command_type: 0}'
                    ],
                    output='screen',
                    name='restore_joint_control'
                )
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        model_type_arg,
        rgb_topic_arg,
        depth_topic_arg,
        # segmentation_node,
        visual_servo_node,
        restore_joint_control,
        # rviz_node,  # Uncomment if RViz config exists
    ])
