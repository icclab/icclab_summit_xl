import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
  
  ld = launch.LaunchDescription()

  namespace = launch.substitutions.LaunchConfiguration('namespace')
  rviz = launch.substitutions.LaunchConfiguration('rviz')
  map = launch.substitutions.LaunchConfiguration('map')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='namespace',
    description='Namespace / Id of the robot',
    default_value='summit',
  ))
  
  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='rviz',
    description='Start rviz',
    default_value='False',
  ))

  declare_map_yaml_cmd = DeclareLaunchArgument(
    'map',
    default_value=os.path.join(get_package_share_directory('summit_xl_gazebo'), 'maps', 'turtlebot3_house.yaml'),
    # default_value=os.path.join(get_package_share_directory('nav2_bringup'), 'maps', 'turtlebot3_world.yaml'),
    description='Full path to map yaml file to load')

  ld.add_action(declare_map_yaml_cmd)
  
  # start nav2

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('summit_xl_navigation'), 'launch', 'nav2_bringup_launch.py')
    ),
    launch_arguments={
      'namespace': namespace,
      'map': map,
      }.items(),
  ))
  

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('icclab_summit_xl'), 'launch', 'rviz.launch.py')
    ),
    condition=IfCondition(rviz),
    launch_arguments={
      'rviz_config': 'navigation.rviz',
      }.items(),
  ))

  return ld
