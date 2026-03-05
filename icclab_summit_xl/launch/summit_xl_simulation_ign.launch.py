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
  headless = launch.substitutions.LaunchConfiguration('headless')

  ld.add_action(launch.actions.AppendEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=("/opt/ros/jazzy/share" + ":"
    + os.environ['COLCON_PREFIX_PATH'] + "/icclab_summit_xl/share" + ":"
    + os.environ['COLCON_PREFIX_PATH'] + "/icclab_summit_xl/share/icclab_summit_xl/worlds/models" + ":"
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

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='headless',
    description='Run Gazebo in headless mode (no GUI)',
    default_value='false'
  ))

  ros_gz_sim = get_package_share_directory('ros_gz_sim')


  # Build gz_args conditionally based on headless mode
  # Note: Gazebo must run (not be paused) for gz_ros2_control to work properly
  # The -r flag makes it run immediately on start
  gz_args = launch.substitutions.PythonExpression([
    '"',
    '-v 1 -s -r ',  # -s for headless (server only), -r to run on start
    world,
    '" if "',
    headless,
    '" == "true" else "',
    '-v 1 -r ',  # GUI mode with -r to run on start
    world,
    '"'
  ])

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': gz_args}.items()
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
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
  )

  # Use TimerAction to give gz_ros2_control time to initialize after robot spawn
  # This avoids race condition where controllers try to activate before plugin is ready
  delay_joint_broadcaster_after_robot_spawner = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=robot_spawner,
          on_exit=[
              launch.actions.TimerAction(
                  period=2.0,  # Wait 2 seconds after robot spawn for gz_ros2_control to fully initialize
                  actions=[joint_broadcaster]
              )
          ],
      )
  )
  ld.add_action(delay_joint_broadcaster_after_robot_spawner)

  # Load and activate arm_controller and robotiq_gripper_controller after joint_state_broadcaster
  # Note: gz_ros2_control auto-loads controllers, which can take ~10s. Spawner will wait/retry.
  arm_controller = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["arm_controller", "--controller-manager", "/controller_manager"],
  )

  gripper_controller = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
  )

  # Delay arm controller after joint_state_broadcaster
  # Longer delay needed because gz_ros2_control auto-loads the controller (from YAML config)
  # and this takes ~10 seconds. We need to wait for that to complete before spawner can activate it.
  delay_arm_controller = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=joint_broadcaster,
          on_exit=[
              launch.actions.TimerAction(
                  period=8.0,  # Wait 8 seconds after joint_state_broadcaster for gz_ros2_control to finish loading
                  actions=[arm_controller]
              )
          ],
      )
  )
  ld.add_action(delay_arm_controller)

  # Delay gripper controller after arm controller
  delay_gripper_controller = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=arm_controller,
          on_exit=[gripper_controller],
      )
  )
  ld.add_action(delay_gripper_controller)

  # robotnik_base_control = launch_ros.actions.Node(
  #   package="controller_manager",
  #   executable="spawner",
  #   arguments=["robotnik_base_control", "--controller-manager", ["/", robot_id, "/controller_manager"]],
  # )
  # ld.add_action(robotnik_base_control)

  bridge_params = os.path.join(
        get_package_share_directory('icclab_summit_xl'),
        'config',
        'ign_gazebo_bridge_depth_image.yaml'
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


  # Generate arm_camera pointcloud from depth+color via depth_image_proc
  # (Gazebo rgbd_camera pointcloud is in wrong frame convention, so we skip it
  # and regenerate on the ROS side with correct optical frame)
  arm_camera_pointcloud = launch_ros.actions.Node(
      package='depth_image_proc',
      executable='point_cloud_xyzrgb_node',
      name='arm_camera_pointcloud',
      remappings=[
          ('rgb/image_rect_color', '/arm_camera/color/image_raw'),
          ('rgb/camera_info', '/arm_camera/color/camera_info'),
          ('depth_registered/image_rect', '/arm_camera/depth/image_raw'),
          ('points', '/arm_camera/points'),
      ],
      parameters=[{'use_sim_time': True}],
  )

  delay_pointcloud_after_bridge = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=robot_spawner,
          on_exit=[
              launch.actions.TimerAction(
                  period=5.0,
                  actions=[arm_camera_pointcloud]
              )
          ],
      )
  )
  ld.add_action(delay_pointcloud_after_bridge)

  # IMAGE TRANSPORT REPUBLISHERS FOR MCP SERVER COMPATIBILITY
  # These convert raw Gazebo images to compressed format for MCP server

  # Front camera color image compression
  front_color_compression = launch_ros.actions.Node(
      package='image_transport',
      executable='republish',
      arguments=['raw', 'compressed'],
      remappings=[
          ('in', '/front_rgbd_camera/color/image_raw'),
          ('out/compressed', '/front_rgbd_camera/color/image_raw/compressed')
      ],
      output='screen',
      parameters=[{'use_sim_time': True}]
  )

  # Front camera depth image compression
  front_depth_compression = launch_ros.actions.Node(
      package='image_transport',
      executable='republish',
      arguments=['raw', 'compressedDepth'],
      remappings=[
          ('in', '/front_rgbd_camera/depth/image_raw'),
          ('out/compressedDepth', '/front_rgbd_camera/depth/image_raw/compressedDepth')
      ],
      output='screen',
      parameters=[{'use_sim_time': True}]
  )

  # Arm camera color image compression
  arm_color_compression = launch_ros.actions.Node(
      package='image_transport',
      executable='republish',
      arguments=['raw', 'compressed'],
      remappings=[
          ('in', '/arm_camera/color/image_raw'),
          ('out/compressed', '/arm_camera/color/image_raw/compressed')
      ],
      output='screen',
      parameters=[{'use_sim_time': True}]
  )

  # Arm camera depth image compression
  arm_depth_compression = launch_ros.actions.Node(
      package='image_transport',
      executable='republish',
      arguments=['raw', 'compressedDepth'],
      remappings=[
          ('in', '/arm_camera/depth/image_raw'),
          ('out/compressedDepth', '/arm_camera/depth/image_raw/compressedDepth')
      ],
      output='screen',
      parameters=[{'use_sim_time': True}]
  )

  # Start image compression nodes with delay after everything is initialized
  delay_compression = launch.actions.TimerAction(
      period=15.0,  # Wait 15 seconds for all systems to be ready
      actions=[front_color_compression, front_depth_compression,
              arm_color_compression, arm_depth_compression]
  )
  ld.add_action(delay_compression)

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
