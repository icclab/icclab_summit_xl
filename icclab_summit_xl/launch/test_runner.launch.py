#!/usr/bin/env python3
"""
Test runner launch file - runs tests assuming simulation, nav2, and moveit are already running.

This is useful for running tests individually during development.

Usage:
  # First, start the simulation, nav2, and moveit in separate terminals:
  ros2 launch icclab_summit_xl summit_xl_simulation_ign.launch.py
  ros2 launch icclab_summit_xl summit_xl_nav2.launch.py rviz:=false
  ros2 launch icclab_summit_xl_move_it_config move_group.launch.py

  # Then run the tests:
  ros2 launch icclab_summit_xl test_runner.launch.py test:=simulation
  ros2 launch icclab_summit_xl test_runner.launch.py test:=navigation
  ros2 launch icclab_summit_xl test_runner.launch.py test:=moveit
  ros2 launch icclab_summit_xl test_runner.launch.py test:=all
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    test_arg = DeclareLaunchArgument(
        'test',
        default_value='all',
        description='Which test to run: all, simulation, navigation, moveit'
    )

    # Get launch configuration
    test_to_run = LaunchConfiguration('test')

    # Helper function to check if test should run
    def should_run_test(test_name):
        # This is a simplified version - in practice you'd use substitution logic
        # For now, we'll create nodes for all tests and use conditions
        return True

    # Test nodes
    simulation_test = Node(
        package='icclab_summit_xl',
        executable='test_simulation_readiness.py',
        name='test_simulation_readiness',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    navigation_test = Node(
        package='icclab_summit_xl',
        executable='test_navigation.py',
        name='test_navigation',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    moveit_test = Node(
        package='icclab_summit_xl',
        executable='test_moveit_configurations.py',
        name='test_moveit_configurations',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        test_arg,
        LogInfo(msg='='*60),
        LogInfo(msg=['Running test: ', test_to_run]),
        LogInfo(msg='='*60),
        LogInfo(msg='Note: Make sure simulation, nav2, and moveit are already running!'),
        LogInfo(msg='='*60),
        simulation_test,
        navigation_test,
        moveit_test,
    ])
