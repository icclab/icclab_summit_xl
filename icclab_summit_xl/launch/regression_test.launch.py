#!/usr/bin/env python3
"""
Comprehensive regression test launch file for Summit XL robot.

This launch file:
1. Starts Gazebo simulation
2. Starts Nav2 navigation stack
3. Starts MoveIt move_group
4. Runs regression tests for:
   - Simulation readiness
   - Navigation functionality
   - MoveIt pre-set configurations

Usage:
  ros2 launch icclab_summit_xl regression_test.launch.py

  To run specific test:
  ros2 launch icclab_summit_xl regression_test.launch.py test:=simulation
  ros2 launch icclab_summit_xl regression_test.launch.py test:=navigation
  ros2 launch icclab_summit_xl regression_test.launch.py test:=moveit
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_pytest.tools import LaunchPytestTool
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    test_arg = DeclareLaunchArgument(
        'test',
        default_value='all',
        description='Which test to run: all, simulation, navigation, moveit'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run in headless mode (no GUI)'
    )

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='600',
        description='Test timeout in seconds'
    )

    # Get launch configurations
    test_to_run = LaunchConfiguration('test')
    headless = LaunchConfiguration('headless')

    # Get package directories
    pkg_icclab_summit_xl = get_package_share_directory('icclab_summit_xl')
    pkg_moveit_config = get_package_share_directory('icclab_summit_xl_move_it_config')

    # 1. Start Gazebo simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_icclab_summit_xl, 'launch', 'summit_xl_simulation_ign.launch.py')
        ),
        launch_arguments={
            'robot_id': 'summit',
        }.items(),
    )

    # 2. Start Nav2 (with delay to let simulation start)
    nav2_launch = TimerAction(
        period=10.0,  # Wait 10 seconds for simulation to be ready
        actions=[
            LogInfo(msg='Starting Nav2...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_icclab_summit_xl, 'launch', 'summit_xl_nav2.launch.py')
                ),
                launch_arguments={
                    'rviz': 'false',  # Don't start RViz in tests
                }.items(),
            ),
        ]
    )

    # 3. Start MoveIt move_group (with delay)
    moveit_launch = TimerAction(
        period=15.0,  # Wait 15 seconds for simulation and controllers
        actions=[
            LogInfo(msg='Starting MoveIt...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                }.items(),
            ),
        ]
    )

    # 4. Run tests (with delay to let everything start)
    # Test 1: Simulation readiness
    simulation_test = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='='*60),
            LogInfo(msg='Running simulation readiness test...'),
            LogInfo(msg='='*60),
            Node(
                package='icclab_summit_xl',
                executable='test_simulation_readiness.py',
                name='test_simulation_readiness',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
    )

    # Test 2: Navigation
    navigation_test = TimerAction(
        period=30.0,  # Wait for Nav2 to be ready
        actions=[
            LogInfo(msg='='*60),
            LogInfo(msg='Running navigation test...'),
            LogInfo(msg='='*60),
            Node(
                package='icclab_summit_xl',
                executable='test_navigation.py',
                name='test_navigation',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
    )

    # Test 3: MoveIt configurations
    moveit_test = TimerAction(
        period=25.0,  # Wait for MoveIt to be ready
        actions=[
            LogInfo(msg='='*60),
            LogInfo(msg='Running MoveIt configurations test...'),
            LogInfo(msg='='*60),
            Node(
                package='icclab_summit_xl',
                executable='test_moveit_configurations.py',
                name='test_moveit_configurations',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
    )

    return LaunchDescription([
        test_arg,
        headless_arg,
        timeout_arg,
        LogInfo(msg='='*60),
        LogInfo(msg='Starting Summit XL Regression Tests'),
        LogInfo(msg='='*60),
        simulation_launch,
        nav2_launch,
        moveit_launch,
        simulation_test,
        navigation_test,
        moveit_test,
    ])
