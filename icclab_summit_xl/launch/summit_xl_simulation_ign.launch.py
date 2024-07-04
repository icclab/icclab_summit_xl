import launch
import launch_ros
import os
import re
from launch.actions import LogInfo, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from robotnik_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):

    use_sim_time = True
    # controllers_file = launch.substitutions.LaunchConfiguration('controllers_file')
    controllers_file= [launch_ros.substitutions.FindPackageShare('icclab_summit_xl'), '/config/', 'ur_controllers.yaml']
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    robot_xacro = launch.substitutions.LaunchConfiguration('robot_xacro')

    config_file_rewritten = RewrittenYaml(
        source_file=controllers_file,
        param_rewrites={},
        root_key=[robot_id],
        convert_types=True,
    )

    robot_description_content = launch.substitutions.Command(
        [
            launch.substitutions.PathJoinSubstitution(
                [launch.substitutions.FindExecutable(name="xacro")]),
            " ",
            robot_xacro,
            " robot_id:=", robot_id,
            " robot_ns:=", robot_id,
            " config_controllers:=", config_file_rewritten,
        ]
    )

    # Get rid of XML comments
    # Workaround because of this bug: https://github.com/ros-controls/gazebo_ros2_control/issues/295
    pattern = r'<!--(.*?)-->'
    robot_description_param_no_comments = re.sub(pattern, '', robot_description_content.perform(context), flags=re.DOTALL)    

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_id,
        remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_param_no_comments,
            'publish_frequency': 100.0,
            'frame_prefix': "", # [params['robot_id'], '/'],
        }],
    )

    return [robot_state_publisher, LogInfo(msg=["summit_xl_simulation_ign", " robot_description_param: \n", robot_description_content])]


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
    default_value=['https://fuel.gazebosim.org/1.0/sonay/worlds/tugbot_depot']
  ))

  ros_gz_sim = get_package_share_directory('ros_gz_sim')


  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': ['-v1 ', world]}.items()
  ))

  robot_spawner = launch_ros.actions.Node(
    package="ros_gz_sim",
    executable="create",
    arguments=["-name", robot_id, "-topic", ("/",  robot_id, "/robot_description"), '-y', '2.0'],
  )
  ld.add_action(robot_spawner)

  ld.add_action(OpaqueFunction(function=launch_setup))

  # arm_controller = launch_ros.actions.Node(
  #   package="controller_manager",
  #   executable="spawner",
  #   arguments=["arm_controller", "--controller-manager", ["/", robot_id, "/controller_manager"]],
  # )
  # ld.add_action(arm_controller)

  # gripper_controller = launch_ros.actions.Node(
  #   package="controller_manager",
  #   executable="spawner",
  #   arguments=["robotiq_gripper_controller", "--controller-manager", ["/", robot_id, "/controller_manager"]],
  # )
  # ld.add_action(gripper_controller)

  
  return ld