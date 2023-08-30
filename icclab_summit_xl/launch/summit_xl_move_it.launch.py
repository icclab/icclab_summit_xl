import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo



def generate_launch_description():
  
  ld = launch.LaunchDescription()

  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
 
  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_id',
    description='Id of the robot',
    default_value='summit',
  ))
 
  # push namespace
  namespace = launch_ros.actions.PushRosNamespace(namespace=robot_id)
  ld.add_action(namespace)

  # add remappings for all
  ld.add_action(SetRemap('/tf', 'tf'))
  ld.add_action(SetRemap('/tf_static', 'tf_static'))
  # ld.add_action(SetRemap('/execute_trajectory', 'execute_trajectory'))
  # ld.add_action(SetRemap('/move_action', 'move_action'))

  move_group_include = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('icclab_summit_xl_move_it_config'), 'launch', 'move_group.launch.py')
    ),
    launch_arguments={'use_sim_time': 'true'}.items(), # the included launchfile unfortunately doesn't allow setting any arguments
    # we modified it directly
  )
  ld.add_action(move_group_include)  

  # Launch rviz with kinematics and other needed params
  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('icclab_summit_xl_move_it_config'), 'launch', 'moveit_rviz.launch.py')
    ),
    launch_arguments={'use_sim_time': 'true'}.items(),
  ))
 
  # ld.add_action(Node(        
  #   package='rviz2',
  #   executable='rviz2',
  #   #remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')],
  #   arguments=['-d', os.path.join(get_package_share_directory('icclab_summit_xl'), 'rviz', "grasping.rviz")],
  #   output='screen'))

  # ld.add_action(LogInfo(msg=str(ld.entities)))

  return ld