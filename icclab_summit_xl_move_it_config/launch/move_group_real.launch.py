# move_group for the real Summit: everything there runs in the /summit namespace
# (controller_manager, arm_controller, joint_states, tf), on wall time.
#
#   ros2 launch icclab_summit_xl_move_it_config move_group_real.launch.py
#   ros2 launch icclab_summit_xl_move_it_config move_group_real.launch.py allow_trajectory_execution:=true
#
# allow_trajectory_execution defaults to false: move_group then only plans and never
# sends a trajectory to the arm. The simulation keeps using move_group.launch.py.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # same xacro arguments as summit_xl_real.launch.py, so the model matches /summit/robot_description
    moveit_config = (
        MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config")
        .robot_description(mappings={"use_fake_hardware": "false", "robot_id": "summit", "robot_ns": "summit"})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # sensors_3d.yaml points at the simulation's camera topics
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
        DeclareLaunchArgument("allow_trajectory_execution", default_value="false"),
        GroupAction([
            PushRosNamespace("summit"),
            # robot_state_publisher on the Summit publishes to /summit/tf and /summit/tf_static
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
