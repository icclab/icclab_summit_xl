import launch
import launch_ros
import os
import re
from launch.actions import LogInfo, OpaqueFunction, AppendEnvironmentVariable, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch.event_handlers import OnProcessExit

def launch_setup(context, *args, **kwargs):

    use_sim_time = True
    # controllers_file = launch.substitutions.LaunchConfiguration('controllers_file')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    robot_xacro = launch.substitutions.LaunchConfiguration('robot_xacro')

    # Don't use RewrittenYaml with root_key since we removed namespaces
    # Use PathJoinSubstitution to create the controllers file path
    config_file_rewritten = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('icclab_summit_xl'),
        'config',
        'ur_controllers.yaml'
    ])

    robot_description_content = launch.substitutions.Command(
        [
            launch.substitutions.PathJoinSubstitution(
                [launch.substitutions.FindExecutable(name="xacro")]),
            " ",
            robot_xacro,
            " robot_id:=", robot_id,
            # robot_ns defaults to empty in xacro for MoveItPy compatibility
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
        # Removed namespace for MoveItPy compatibility
        # namespace=robot_id,
        # Removed TF remappings - keep in global namespace
        # remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_param_no_comments,
            'publish_frequency': 100.0,
            'frame_prefix': "", # [params['robot_id'], '/'],
        }],
    )

    return [robot_state_publisher, 
            #LogInfo(msg=["summit_xl_simulation_ign", " robot_description_param: \n", robot_description_content])
            ]


def generate_launch_description():
  
  ld = launch.LaunchDescription()

  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  robot_xacro = launch.substitutions.LaunchConfiguration('robot_xacro')
  world = launch.substitutions.LaunchConfiguration('world')

  ld.add_action(launch.actions.AppendEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=("/opt/ros/jazzy/share" + ":" 
    + os.environ['COLCON_PREFIX_PATH'] + "/icclab_summit_xl/share" + ":"
    + os.environ['COLCON_PREFIX_PATH'] + "/robotiq_description/share")))

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
    default_value=['https://fuel.gazebosim.org/1.0/sonay/worlds/tugbot_depot'] #"empty.sdf"
  ))

  ros_gz_sim = get_package_share_directory('ros_gz_sim')


  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': ['-v 1 ', world]}.items()
  ))

  robot_spawner = launch_ros.actions.Node(
    package="ros_gz_sim",
    executable="create",
    # Removed namespace from robot_description topic
    arguments=["-name", robot_id, "-topic", "robot_description", '-y', '2.0'],
  )
  ld.add_action(robot_spawner)

  ld.add_action(OpaqueFunction(function=launch_setup))
  
  joint_broadcaster = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    # Removed namespace from controller_manager path
    arguments=["joint_state_broadcaster", "--switch-timeout", "600", "--controller-manager", "/controller_manager"],
  )

  # Delay joint_broadcaster start after `robot_spawner`
  delay_joint_broadcaster_after_robot_spawner = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=robot_spawner,
          on_exit=[joint_broadcaster],
      )
  )
  ld.add_action(delay_joint_broadcaster_after_robot_spawner)

  arm_controller = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    # Removed namespace from controller_manager path
    arguments=["arm_controller", "--switch-timeout",  "600", "--controller-manager", "/controller_manager"],
  )

  # Delay arm_controller start after `joint_state_broadcaster`
  delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=joint_broadcaster,
          on_exit=[arm_controller],
      )
  )
  ld.add_action(delay_arm_controller_after_joint_state_broadcaster)

  gripper_controller = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    # Removed namespace from controller_manager path
    arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
  )
  # Delay gripper_controller start after `arm_controller`
  delay_gripper_controller_after_arm_controller = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=arm_controller,
          on_exit=[gripper_controller],
      )
  )
  ld.add_action(delay_gripper_controller_after_arm_controller)

  # robotnik_base_control = launch_ros.actions.Node(
  #   package="controller_manager",
  #   executable="spawner",
  #   arguments=["robotnik_base_control", "--controller-manager", ["/", robot_id, "/controller_manager"]],
  # )
  # ld.add_action(robotnik_base_control)

  bridge_params = os.path.join(
        get_package_share_directory('icclab_summit_xl'),
        'config',
        'ign_gazebo_bridge.yaml'
    )

  start_gazebo_ros_bridge_cmd = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

  # Delay joint_broadcaster start after `robot_spawner`
  delay_bridge_after_robot_spawner = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=robot_spawner,
          on_exit=[start_gazebo_ros_bridge_cmd],
      )
  )
  ld.add_action(delay_bridge_after_robot_spawner)


  # odom_tf = launch_ros.actions.Node(
  #       package='icclab_summit_xl',
  #       executable='odom_tf',
  #       name='odom_to_base_link_publisher',
  #       remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
  #   )
  # ld.add_action(odom_tf)

  # cmd_vel_topic_remap = launch_ros.actions.Node(
  #       package='icclab_summit_xl',
  #       executable='cmd_vel_topic_remap',
  #       name='cmd_vel_topic_remap',
  #   )
  # ld.add_action(cmd_vel_topic_remap)
  
  return ld
