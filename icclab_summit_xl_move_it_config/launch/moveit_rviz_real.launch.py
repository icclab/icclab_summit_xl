# MoveIt RViz for the real robot: runs in the robot_id namespace (default summit), TF from <robot_id>/tf.
# config/moveit_real.rviz is moveit.rviz without the simulation camera displays and with
# Move Group Namespace /summit; for another robot_id pass a config with rviz_config:=.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config")
        .robot_description(mappings={"use_fake_hardware": "false"})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="summit", description="Id of the robot"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit_real.rviz"),
            description="RViz configuration file",
        ),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot_id")),
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
