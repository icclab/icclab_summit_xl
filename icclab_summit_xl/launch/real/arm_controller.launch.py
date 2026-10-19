import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
  
  ld = launch.LaunchDescription()

  # start controller 
  # ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.210 headless_mode:=true launch_rviz:=false  tf_prefix:=arm_

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

  ld.add_action(driver_action)
  return ld

