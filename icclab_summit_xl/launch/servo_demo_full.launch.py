#!/usr/bin/env python3
"""
Complete MoveIt Servo demo launch file for Summit XL
This launch file starts:
- Robot simulation (Gazebo) or real robot interface
- MoveIt move_group
- MoveIt Servo node
- RViz for visualization
- Optional: demo control nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    start_keyboard_control_arg = DeclareLaunchArgument(
        "start_keyboard_control",
        default_value="false",
        description="Start keyboard control node",
    )

    start_circle_demo_arg = DeclareLaunchArgument(
        "start_circle_demo",
        default_value="false",
        description="Start circle demo node (circular motion)",
    )

    circle_demo_plane_arg = DeclareLaunchArgument(
        "circle_demo_plane",
        default_value="xy",
        description="Plane for circle demo: xy, xz, or yz",
    )

    circle_demo_radius_arg = DeclareLaunchArgument(
        "circle_demo_radius",
        default_value="0.1",
        description="Radius for circle demo in meters",
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    start_keyboard_control = LaunchConfiguration("start_keyboard_control")
    start_circle_demo = LaunchConfiguration("start_circle_demo")
    circle_demo_plane = LaunchConfiguration("circle_demo_plane")
    circle_demo_radius = LaunchConfiguration("circle_demo_radius")

    # Start servo demo (includes move_group, servo node, and rviz)
    servo_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("icclab_summit_xl_move_it_config"),
                    "launch",
                    "servo_demo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_rviz": start_rviz,
        }.items(),
    )

    # Keyboard control node
    keyboard_control_node = Node(
        package="icclab_summit_xl",
        executable="servo_keyboard_control.py",
        name="servo_keyboard_control",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_keyboard_control),
    )

    # Circle demo node
    circle_demo_node = Node(
        package="icclab_summit_xl",
        executable="servo_circle_demo.py",
        name="servo_circle_demo",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"plane": circle_demo_plane},
            {"radius": circle_demo_radius},
        ],
        condition=IfCondition(start_circle_demo),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            start_rviz_arg,
            start_keyboard_control_arg,
            start_circle_demo_arg,
            circle_demo_plane_arg,
            circle_demo_radius_arg,
            servo_demo_launch,
            keyboard_control_node,
            circle_demo_node,
        ]
    )
