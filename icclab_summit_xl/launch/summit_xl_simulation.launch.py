import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource

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

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='world',
    description='World to load',
    default_value=[launch_ros.substitutions.FindPackageShare('turtlebot3_gazebo'), '/worlds/', 'turtlebot3_house.world']
  ))

  summit_xl_gazebo = get_package_share_directory('summit_xl_gazebo')


  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(summit_xl_gazebo, 'launch', 'default.launch.py')
    ),
    launch_arguments={
      'robot_id': robot_id,
      'namespace': robot_id,
      'robot_xacro': robot_xacro,
      'world' : world,
      'controllers_file': [launch_ros.substitutions.FindPackageShare('icclab_summit_xl'), '/config/', 'ur_controllers.yaml']
      }.items(),
  ))

  arm_controller = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["arm_controller", "--controller-manager", ["/", robot_id, "/controller_manager"]],
  )
  ld.add_action(arm_controller)

  gripper_controller = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotiq_gripper_controller", "--controller-manager", ["/", robot_id, "/controller_manager"]],
  )
  ld.add_action(gripper_controller)

  
  return ld