import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetRemap, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():

  ld = launch.LaunchDescription()

  # Removed robot_id namespace to work with MoveItPy
  # robot_id = launch.substitutions.LaunchConfiguration('robot_id')

  # ld.add_action(launch.actions.DeclareLaunchArgument(
  #   name='robot_id',
  #   description='Id of the robot',
  #   default_value='summit',
  # ))

  use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
  use_servo = launch.substitutions.LaunchConfiguration('use_servo')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='use_sim_time',
    description='Whether simulation or not',
    default_value='true',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='use_servo',
    description='Launch MoveIt Servo node for real-time control',
    default_value='false',
  ))

  # Removed namespace push - MoveItPy doesn't work well with namespaces in Jazzy
  # namespace = launch_ros.actions.PushRosNamespace(namespace=robot_id)
  # ld.add_action(namespace)

  # Removed TF remappings - keep TF in global namespace
  # ld.add_action(SetRemap('/tf', 'tf'))
  # ld.add_action(SetRemap('/tf_static', 'tf_static'))
  # ld.add_action(SetRemap('/execute_trajectory', 'execute_trajectory'))
  # ld.add_action(SetRemap('/move_action', 'move_action'))

  move_group_include = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('icclab_summit_xl_move_it_config'), 'launch', 'move_group.launch.py')
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items(), # the included launchfile unfortunately doesn't allow setting any arguments
    # we modified it directly
  )
  ld.add_action(move_group_include)  

  # Launch rviz with kinematics and other needed params
  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('icclab_summit_xl_move_it_config'), 'launch', 'moveit_rviz.launch.py')
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items(),
  ))

  # ld.add_action(Node(
  #   package='rviz2',
  #   executable='rviz2',
  #   #remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')],
  #   arguments=['-d', os.path.join(get_package_share_directory('icclab_summit_xl'), 'rviz', "grasping.rviz")],
  #   output='screen'))

  # MoveIt Servo Node (optional, enabled with use_servo:=true)
  # Build MoveIt configuration for servo
  moveit_config = (
    MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config")
    .robot_description_semantic(file_path="config/summit_xl.srdf")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
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
          moveit_config.robot_description,
          moveit_config.robot_description_semantic,
          moveit_config.robot_description_kinematics,
          servo_params,
          {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_servo),
      )
    ]
  )

  ld.add_action(servo_node)

  # ld.add_action(LogInfo(msg=str(ld.entities)))

  return ld