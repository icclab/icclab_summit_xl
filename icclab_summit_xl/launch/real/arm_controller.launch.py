import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
  
  ld = launch.LaunchDescription()

  namespace = launch.substitutions.LaunchConfiguration('namespace')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='namespace',
    description='Namespace / Id of the robot',
    default_value='summit',
  ))
  
  
  # start controller 
  # ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.210 headless_mode:=true launch_rviz:=false  tf_prefix:=/summit/arm_

  driver_action = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('icclab_summit_xl'), 'launch', 'real', 'ur_control.launch.py')
    ),
    launch_arguments={
      'ur_type': 'ur5',
      'robot_ip': '192.168.0.210', 
      'headless_mode': 'true', 
      'launch_rviz': 'false',  
      'tf_prefix': 'arm_'
      }.items()
  )  

  ga = GroupAction(actions=[PushRosNamespace(namespace), driver_action])
  ld.add_action(ga)
  return ld

