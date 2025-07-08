"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    mcb = MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config")
    mcb.moveit_cpp(file_path=(get_package_share_directory("icclab_summit_xl_move_it_config") + "/config/moveit_cpp_python_api.yaml_bak"))
    moveit_config = mcb.to_moveit_configs()

    example_file = DeclareLaunchArgument(
        "example_file",
        default_value="motion_planning_python_api.py",
        description="Python API tutorial file name",
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="icclab_summit_xl",
        executable=LaunchConfiguration("example_file"),
        output="both",
        namespace="summit",
        parameters=[moveit_config.to_dict()], #{'use_sim_time': True'}
        # arguments=["--ros-args", "-p", "use_sim_time:=True"],
        remappings=[
            ("/tf", "/summit/tf"),
            ("/tf_static", "/summit/tf_static"),
            ("/robot_description", "/summit/robot_description"),
            ("/robot_description_semantic", "/summit/robot_description_semantic"),
            ("/robot_description_planning_scene", "/summit/robot_description_planning_scene"),
            ("/arm_controller/follow_joint_trajectory", "/summit/arm_controller/follow_joint_trajectory"),
            ("/arm_controller/follow_joint_trajectory/_action/goal", "/summit/arm_controller/follow_joint_trajectory/_action/goal"),
            ("/arm_controller/follow_joint_trajectory/_action/status", "/summit/arm_controller/follow_joint_trajectory/_action/status"),
            ("/arm_controller/follow_joint_trajectory/_action/feedback", "/summit/arm_controller/follow_joint_trajectory/_action/feedback"),
            ("/arm_controller/joint_trajectory", "/summit/arm_controller/joint_trajectory"),
            ("/arm_controller/state", "/summit/arm_controller/state"),
            ("/joint_states", "/summit/joint_states"),
            ("/collision_object", "/summit/collision_object"),
            ("/attached_collision_object", "/summit/attached_collision_object"),
            ("/planning_scene", "/summit/planning_scene"),
            ("/planning_scene_world", "/summit/planning_scene_world"),
            ("/move_group/parameter_descriptions", "/summit/move_group/parameter_descriptions"),
            ("/move_group/parameter_updates", "/summit/move_group/parameter_updates"),
            ("/move_group/robot_description", "/summit/move_group/robot_description"),
             ("/trajectory_execution_event", "/summit/trajectory_execution_event"),
        ],
    )


    return LaunchDescription(
        [
            LogInfo(msg=str(moveit_config)),
            example_file,
            moveit_py_node 
        ]

    )
