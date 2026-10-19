import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  
  ld = launch.LaunchDescription()

  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  robot_xacro = launch.substitutions.LaunchConfiguration('robot_xacro')
  world = launch.substitutions.LaunchConfiguration('world')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_id',
    description='Id of the robot',
    default_value='summit',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_xacro',
        description='Robot xacro file path for the robot model',
        default_value=os.path.join(get_package_share_directory('icclab_summit_xl'), 'robots', 'summit_xls_icclab.urdf.xacro')
  ))

  # start robot_state_publisher and robot description, without namespace (as in the simulation)

  robot_description = ParameterValue(
    launch.substitutions.Command(['xacro ', robot_xacro, ' robot_id:=', robot_id, ' use_fake_hardware:=false']),
    value_type=str)

  ld.add_action(Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'robot_description': robot_description,
      'publish_frequency': 100.0,
    }],
  ))

  # Arm and gripper contollers launched separately in arm_controller.launch

  # start lidars

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('icclab_summit_xl'), 'launch/real', 'urg_node_launch.py')
    ),
    launch_arguments={
      'lidar_position': 'front',
      }.items(),
  ))

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('icclab_summit_xl'), 'launch/real', 'urg_node_launch.py')
    ),
    launch_arguments={
      'lidar_position': 'rear',
      }.items(),
  ))

  return ld
