#!/usr/bin/env python3
"""
Launch file for MoveIt Servo demo with Summit XL robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz for visualization",
    )

    use_servo_node_arg = DeclareLaunchArgument(
        "use_servo_node",
        default_value="true",
        description="Start the servo node",
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    use_servo_node = LaunchConfiguration("use_servo_node")

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config")
        .robot_description_semantic(file_path="config/summit_xl.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Get servo configuration file path
    servo_params = PathJoinSubstitution(
        [
            FindPackageShare("icclab_summit_xl_move_it_config"),
            "config",
            "moveit_servo.yaml",
        ]
    )

    # MoveIt Servo Node - delayed to ensure move_group is ready
    servo_node = TimerAction(
        period=5.0,  # Wait 5 seconds for move_group to initialize
        actions=[
            Node(
                package="moveit_servo",
                executable="servo_node",
                output="screen",
                parameters=[
                    # Only pass essential parameters, not full moveit_config
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    servo_params,
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(use_servo_node),
            )
        ]
    )

    # Start move_group for collision checking and planning scene
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("icclab_summit_xl_move_it_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Start RViz
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("icclab_summit_xl_move_it_config"),
            "config",
            "moveit.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            start_rviz_arg,
            use_servo_node_arg,
            move_group_launch,
            servo_node,
            rviz_node,
        ]
    )
