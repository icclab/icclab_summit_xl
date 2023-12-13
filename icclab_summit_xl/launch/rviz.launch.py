import launch
import launch_ros
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  
  ld = launch.LaunchDescription()

  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  rviz_config = launch.substitutions.LaunchConfiguration('rviz_config')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_id',
    description='Id of the robot',
    default_value='summit',
  ))
  
  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='rviz_config',
    description='Rviz configuration file to use (from icclab_summit_xl/rviz dir)',
    default_value='robot.rviz',
  ))

# Launch rviz
  ld.add_action(Node(        
    package='rviz2',
    executable='rviz2',
    namespace=robot_id,
    remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')],
    arguments=['-d', [os.path.join(get_package_share_directory('icclab_summit_xl'), 'rviz/'), rviz_config]],
    output='screen'))
  
  return ld
