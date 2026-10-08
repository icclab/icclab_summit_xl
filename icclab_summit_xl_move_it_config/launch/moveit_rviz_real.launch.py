# MoveIt RViz for the real Summit, the counterpart of move_group_real.launch.py: RViz runs in
# the /summit namespace, so the relative topics in moveit.rviz (robot_description,
# monitored_planning_scene, display_planned_path) and the move_group actions resolve to
# /summit/..., and TF is read from /summit/tf. Wall time, no Gazebo clock.
#
#   ros2 launch icclab_summit_xl_move_it_config moveit_rviz_real.launch.py
#
# The simulation keeps using moveit_rviz.launch.py.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # same xacro arguments as summit_xl_real.launch.py and move_group_real.launch.py
    moveit_config = (
        MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config")
        .robot_description(mappings={"use_fake_hardware": "false", "robot_id": "summit", "robot_ns": "summit"})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
            description="RViz configuration file",
        ),
        GroupAction([
            PushRosNamespace("summit"),
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {"use_sim_time": False},
                ],
            ),
        ]),
    ])
