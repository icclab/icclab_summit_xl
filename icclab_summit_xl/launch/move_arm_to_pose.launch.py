import launch
import launch_ros
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  
  ld = launch.LaunchDescription()

  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  x = launch.substitutions.LaunchConfiguration('x')
  y = launch.substitutions.LaunchConfiguration('y')
  z = launch.substitutions.LaunchConfiguration('z')
  rx = launch.substitutions.LaunchConfiguration('rx')
  ry = launch.substitutions.LaunchConfiguration('ry')
  rz = launch.substitutions.LaunchConfiguration('rz')
  rw = launch.substitutions.LaunchConfiguration('rw')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_id',
    description='Id of the robot',
    default_value='summit',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='x',
    description='end effector position x',
    default_value="0.7", # 0.7 0.02 1.3 0 0 0 1
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='y',
    description='end effector position y',
    default_value="0.02", 
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='z',
    description='end effector position z',
    default_value="1.3", 
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='rx',
    description='end effector orientation quaternion rx',
    default_value="0", 
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='ry',
    description='end effector orientation quaternion ry',
    default_value="0", 
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='rz',
    description='end effector orientation quaternion rz',
    default_value="0", 
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='rw',
    description='end effector orientation quaternion rw',
    default_value="1", 
  ))
  

# Launch rviz
  ld.add_action(Node(        
    package='icclab_summit_xl',
    executable='move_arm_to_pose',
    namespace=robot_id,
    #remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')],
    arguments=[x, y, z, rx, ry, rz, rw, '--skip-safety'],
    parameters=[{"use_sim_time": True}],
    output='screen'))
  
  return ld
