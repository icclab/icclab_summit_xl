import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return {}


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_controller_manager",
            default_value="moveit_simple_controller_manager/MoveItSimpleControllerManager",
            description="Moveit controller manager"
        )
    )
    
    # Planning pipeline arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "planning_pipelines",
            default_value="['ompl', 'pilz_industrial_motion_planner']",
            description="Pipelines to add to the planning scene",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "default_planning_pipeline",
            default_value="ompl",
            description="Default planning plugin for MoveGroup",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Debug flag for MoveGroup",
        )
    )
    
    # Get parameters and paths
    moveit_config_package = "icclab_summit_xl_move_it_config"
    
    # Create MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config").to_dict()
    
    # OMPL Planning Configuration
    ompl_planning_yaml = load_yaml(
        moveit_config_package,
        os.path.join("config", "ompl_planning.yaml")
    )
    
    # If the OMPL file doesn't exist or is empty, create a default one
    if not ompl_planning_yaml:
        ompl_planning_yaml = {
            'planner_configs': {
                'BiTRRT': {
                    'type': 'geometric::BiTRRT',
                    'range': 0.0,
                    'temp_change_factor': 0.1,
                    'init_temperature': 100,
                    'frountier_threshold': 0.0,
                    'frountier_node_ratio': 0.1,
                    'cost_threshold': 1e300
                },
                'RRTConnect': {
                    'type': 'geometric::RRTConnect',
                    'range': 0.0
                },
                'RRTstar': {
                    'type': 'geometric::RRTstar',
                    'range': 0.0,
                    'goal_bias': 0.05,
                    'delay_collision_checking': 1
                },
                'BITstar': {
                    'type': 'geometric::BITstar',
                    'range': 0.0
                },
                'LBTRRT': {
                    'type': 'geometric::LBTRRT',
                    'range': 0.0,
                    'goal_bias': 0.05,
                    'epsilon': 0.4
                }
            },
            'manipulator': {
                'planner_configs': ['RRTConnect', 'BiTRRT', 'BITstar', 'RRTstar'],
                'projection_evaluator': 'joints(shoulder_pan_joint,shoulder_lift_joint)',
                'longest_valid_segment_fraction': 0.005
            },
            'mobile_base': {
                'planner_configs': ['RRTConnect', 'BiTRRT'],
                'projection_evaluator': 'joints(virtual_x_joint,virtual_y_joint,virtual_z_joint)',
                'longest_valid_segment_fraction': 0.01
            }
        }
    
    # Pilz Industrial Motion Planner Configuration
    pilz_yaml = {
        'cartesian_limits': {
            'max_trans_vel': 1.0,
            'max_trans_acc': 2.25,
            'max_trans_dec': 2.25,
            'max_rot_vel': 1.5,
            'max_rot_acc': 3.14,
            'max_rot_dec': 3.14
        },
        'joint_limits': {
            'shoulder_pan_joint': {
                'has_velocity_limits': True,
                'max_velocity': 3.14,
                'has_acceleration_limits': True,
                'max_acceleration': 3.14,
                'has_deceleration_limits': True,
                'max_deceleration': 3.14
            },
            'shoulder_lift_joint': {
                'has_velocity_limits': True,
                'max_velocity': 3.14,
                'has_acceleration_limits': True,
                'max_acceleration': 3.14,
                'has_deceleration_limits': True,
                'max_deceleration': 3.14
            },
            'elbow_joint': {
                'has_velocity_limits': True,
                'max_velocity': 3.14,
                'has_acceleration_limits': True,
                'max_acceleration': 3.14,
                'has_deceleration_limits': True,
                'max_deceleration': 3.14
            },
            'wrist_1_joint': {
                'has_velocity_limits': True,
                'max_velocity': 6.28,
                'has_acceleration_limits': True,
                'max_acceleration': 6.28,
                'has_deceleration_limits': True,
                'max_deceleration': 6.28
            },
            'wrist_2_joint': {
                'has_velocity_limits': True,
                'max_velocity': 6.28,
                'has_acceleration_limits': True,
                'max_acceleration': 6.28,
                'has_deceleration_limits': True,
                'max_deceleration': 6.28
            },
            'wrist_3_joint': {
                'has_velocity_limits': True,
                'max_velocity': 6.28,
                'has_acceleration_limits': True,
                'max_acceleration': 6.28,
                'has_deceleration_limits': True,
                'max_deceleration': 6.28
            }
            # Add base joints if necessary
        },
        'planning_groups': {
            'manipulator': {
                'default_planner': 'PTP',
                'planners': ['PTP', 'LIN', 'CIRC']
            }
        }
    }
    
    # Planning pipeline configuration
    planning_pipelines = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
        'pilz_industrial_motion_planner': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    # MoveGroup node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config,
            {'use_sim_time': False},
            {'move_group': {
                'planning_plugin': planning_pipelines[LaunchConfiguration('default_planning_pipeline').perform(None)]['planning_plugin'],
                'request_adapters': planning_pipelines[LaunchConfiguration('default_planning_pipeline').perform(None)]['request_adapters'],
                'start_state_max_bounds_error': planning_pipelines[LaunchConfiguration('default_planning_pipeline').perform(None)]['start_state_max_bounds_error'],
                'planning_pipelines': planning_pipelines,
                'planning_default_planning_pipeline': LaunchConfiguration('default_planning_pipeline'),
                'allowed_planning_time': 10.0,
                'max_planning_attempts': 10,
            }},
            {'ompl': ompl_planning_yaml},
            {'pilz_industrial_motion_planner': pilz_yaml}
        ],
        condition=UnlessCondition(LaunchConfiguration("debug")),
    )
    
    # Debug node
    debug_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        prefix=['gdb -ex run --args'],
        parameters=[
            moveit_config,
            {'use_sim_time': False},
            {'move_group': {
                'planning_plugin': planning_pipelines[LaunchConfiguration('default_planning_pipeline').perform(None)]['planning_plugin'],
                'request_adapters': planning_pipelines[LaunchConfiguration('default_planning_pipeline').perform(None)]['request_adapters'],
                'start_state_max_bounds_error': planning_pipelines[LaunchConfiguration('default_planning_pipeline').perform(None)]['start_state_max_bounds_error'],
                'planning_pipelines': planning_pipelines,
                'planning_default_planning_pipeline': LaunchConfiguration('default_planning_pipeline'),
                'allowed_planning_time': 10.0,
                'max_planning_attempts': 10,
            }},
            {'ompl': ompl_planning_yaml},
            {'pilz_industrial_motion_planner': pilz_yaml}
        ],
        condition=IfCondition(LaunchConfiguration("debug")),
    )
    
    return LaunchDescription(declared_arguments + [move_group_node, debug_move_group_node])