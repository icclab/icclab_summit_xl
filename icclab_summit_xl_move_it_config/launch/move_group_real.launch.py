# move_group for the real robot: runs in the robot_id namespace (default summit), TF from <robot_id>/tf.
# Executes trajectories like the simulation; allow_trajectory_execution:=false only plans.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config")
        .robot_description(mappings={"use_fake_hardware": "false"})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # sensors_3d.yaml uses the simulation camera topics
    moveit_config.sensors_3d = {}

    allow_execution = LaunchConfiguration("allow_trajectory_execution")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": ParameterValue(allow_execution, value_type=bool),
        "capabilities": "",
        "disable_capabilities": "",
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": False,
    }

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="summit", description="Id of the robot"),
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot_id")),
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), move_group_configuration],
            ),
        ]),
    ])
