#!/usr/bin/env python3
"""
MoveItPy Example Script for Summit XL Arm Control
Based on the working moveit_wrapper.py pattern
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from ament_index_python.packages import get_package_share_directory
import os
import time
import logging
from typing import Dict, List

logger = logging.getLogger(__name__)


class MoveItPyExample:
    """Example class demonstrating MoveItPy usage for Summit XL robot."""

    def __init__(self):
        """Initialize MoveItPy using the exact pattern from working wrapper."""
        # Initialize ROS 2 if not already initialized
        if not rclpy.ok():
            logger.info("Initializing ROS 2 context")
            rclpy.init()

        # Use the exact same node name pattern as working wrapper
        node_name = "moveit_py_example_summit"

        # Build MoveIt configuration using MoveItConfigsBuilder (EXACT pattern from wrapper)
        logger.info(f"Building MoveIt configuration for summit_xl")

        try:
            # Try to find moveit_py config file using ROS2 package discovery
            moveit_cpp_yaml_path = None

            # Try different packages that might contain the config
            search_packages = ['icclab_summit_xl_move_it_config', 'icclab_summit_xl']
            for package_name in search_packages:
                try:
                    pkg_share = get_package_share_directory(package_name)
                    candidate_path = os.path.join(pkg_share, 'config', 'summit_xl_moveit_py.yaml')
                    if os.path.exists(candidate_path):
                        moveit_cpp_yaml_path = candidate_path
                        logger.info(f"Found moveit_py config: {moveit_cpp_yaml_path}")
                        break
                except Exception:
                    continue
                    
            # If not found, try generic config name
            if not moveit_cpp_yaml_path:
                try:
                    pkg_share = get_package_share_directory('icclab_summit_xl')
                    candidate_path = os.path.join(pkg_share, 'config', 'moveit_py.yaml')
                    if os.path.exists(candidate_path):
                        moveit_cpp_yaml_path = candidate_path
                        logger.info(f"Found moveit_py config: {moveit_cpp_yaml_path}")
                except Exception:
                    pass

            if not moveit_cpp_yaml_path:
                logger.warning("No moveit_py.yaml config found, using defaults")

            # Build MoveIt configuration (EXACT pattern from working wrapper)
            builder = MoveItConfigsBuilder("summit_xl", package_name="icclab_summit_xl_move_it_config")
            builder = builder.robot_description(file_path="robots/summit_xls_icclab.urdf.xacro")
            builder = builder.robot_description_semantic(file_path="config/summit_xl.srdf")
            builder = builder.trajectory_execution(file_path="config/moveit_controllers.yaml")
            builder = builder.planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True
            )
            builder = builder.planning_pipelines(pipelines=["ompl"])

            # Add moveit_cpp config if found
            if moveit_cpp_yaml_path:
                builder = builder.moveit_cpp(file_path=moveit_cpp_yaml_path)

            moveit_config = builder.to_moveit_configs()
        except Exception as e:
            logger.error(f"Failed to build MoveIt configuration: {e}")
            raise RuntimeError(f"Failed to load MoveIt configuration for summit_xl: {e}")

        # Initialize MoveItPy with the configuration (EXACT pattern from wrapper)
        logger.info(f"Initializing MoveItPy with node name: {node_name}")

        try:
            # Convert config to dictionary and pass as node parameters
            config_dict = moveit_config.to_dict()

            # Add use_sim_time parameter if configured (like wrapper)
            logger.info("Configuring MoveItPy to use simulation time")
            config_dict['use_sim_time'] = True

            # Workaround for MoveItPy bug in Jazzy with use_sim_time
            logger.info("Applying QoS overrides workaround for /clock topic")
            config_dict['qos_overrides'] = {
                '/clock': {
                    'subscription': {
                        'depth': 1,
                        'history': 'keep_last',
                        'durability': 'volatile',
                        'reliability': 'best_effort'
                    }
                }
            }

            # Debug: Print config keys to see what we have
            logger.debug(f"MoveIt config keys: {list(config_dict.keys())}")
            if 'planning_pipelines' in config_dict:
                logger.debug(f"Planning pipelines config: {config_dict['planning_pipelines']}")

            # Create MoveItPy (EXACT pattern from wrapper)
            self.moveit = MoveItPy(node_name=node_name, config_dict=config_dict)
        except RuntimeError as e:
            logger.error(f"Failed to initialize MoveItPy: {e}")
            logger.error(f"Config keys that were provided: {list(config_dict.keys()) if 'config_dict' in locals() else 'config_dict not created'}")
            raise

        # Cache planning components (EXACT pattern from wrapper)
        self.planning_components: Dict[str, PlanningComponent] = {}

        # Get robot model and planning scene monitor
        self.robot_model = self.moveit.get_robot_model()
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        logger.info("MoveItWrapper initialized successfully")

        # Don't create a separate node - just use logging like the tutorial
        self.logger = logger

        self.logger.info('MoveItPy example initialized successfully!')

        # Wait for planning scene to receive joint state updates
        # MoveItPy handles its own spinning internally
        self.logger.info('Waiting for planning scene to initialize...')
        time.sleep(2.0)  # Give time for subscriptions and initial messages
        self.logger.info('Planning scene initialization complete')

    def get_planning_component(self, group_name: str) -> PlanningComponent:
        """Get or create a planning component for a group (EXACT wrapper pattern)."""
        if group_name not in self.planning_components:
            if not self.robot_model.has_joint_model_group(group_name):
                available = self.get_planning_groups()
                raise ValueError(
                    f"Planning group '{group_name}' not found. "
                    f"Available groups: {available}"
                )

            logger.debug(f"Creating planning component for group: {group_name}")
            self.planning_components[group_name] = self.moveit.get_planning_component(
                group_name
            )

        return self.planning_components[group_name]

    def get_planning_groups(self) -> List[str]:
        """Get list of available planning groups (EXACT wrapper pattern)."""
        return self.robot_model.joint_model_group_names

    def publish_trajectory_for_visualization(self, trajectory, group_name: str):
        """Publish trajectory to RViz for visualization."""
        from moveit_msgs.msg import RobotTrajectory

        display_trajectory = DisplayTrajectory()
        display_trajectory.model_id = "summit_xl"

        # Convert the MoveItPy trajectory object to a ROS message
        # MoveItPy trajectories have a get_robot_trajectory_msg() method
        if hasattr(trajectory, 'get_robot_trajectory_msg'):
            robot_traj_msg = trajectory.get_robot_trajectory_msg()
        else:
            # Fallback: assume it's already a message
            robot_traj_msg = trajectory

        # Set the trajectory
        display_trajectory.trajectory.append(robot_traj_msg)

        # Publish for visualization
        self.display_trajectory_publisher.publish(display_trajectory)
        self.logger.debug(f"Published trajectory for group '{group_name}' to /display_planned_path")

    def move_to_named_configuration(self, group_name: str, config_name: str):
        """Move to a named configuration using EXACT wrapper pattern."""
        self.logger.info(f'Moving group {group_name} to configuration: {config_name}')

        try:
            planning_component = self.get_planning_component(group_name)

            # Set start state to current state - MoveItPy handles spinning internally
            planning_component.set_start_state_to_current_state()

            # Set goal to named state
            planning_component.set_goal_state(configuration_name=config_name)

            # Plan
            self.logger.info(f"Planning to named state '{config_name}' for group '{group_name}'")
            plan_result = planning_component.plan()

            if plan_result:
                self.logger.info(f"Planning succeeded for group '{group_name}'")

                # Extract RobotTrajectory from MotionPlanResponse if needed (EXACT MCP pattern)
                robot_trajectory = plan_result
                if hasattr(plan_result, 'trajectory'):
                    logger.debug("Extracting trajectory from MotionPlanResponse")
                    robot_trajectory = plan_result.trajectory

                # Skip visualization for now
                time.sleep(0.5)

                # Execute using wrapper pattern (EXACT MCP pattern)
                self.logger.info("Executing trajectory")
                success = self.moveit.execute(robot_trajectory, controllers=[])

                if success:
                    self.logger.info("Trajectory execution succeeded")
                    return True
                else:
                    self.logger.warning("Trajectory execution failed")
                    return False
            else:
                self.logger.warning(f"Planning failed for group '{group_name}'")
                return False

        except Exception as e:
            self.logger.error(f"Error moving to {config_name}: {e}")
            return False

    def demonstrate_arm_movements(self):
        """Demonstrate moving the arm through various configurations."""
        self.logger.info('=' * 60)
        self.logger.info('DEMONSTRATING ARM MOVEMENTS')
        self.logger.info('=' * 60)

        # List of arm configurations to demonstrate
        configurations = ['home', 'up', 'docked', 'look_forward']

        for config in configurations:
            success = self.move_to_named_configuration("arm", config)
            if not success:
                self.logger.error(f'Failed to move arm to {config}. Stopping demonstration.')
                return False

            # Pause between movements to let controller fully settle
            # This prevents accumulated tracking errors
            time.sleep(2.0)

        self.logger.info('Arm movement demonstration completed successfully!')
        return True

    def demonstrate_gripper_movements(self):
        """Demonstrate opening and closing the gripper."""
        self.logger.info('=' * 60)
        self.logger.info('DEMONSTRATING GRIPPER MOVEMENTS')
        self.logger.info('=' * 60)

        # Demonstrate gripper open/close cycle
        configurations = ['open', 'closed']

        for config in configurations:
            success = self.move_to_named_configuration("gripper", config)
            if not success:
                self.logger.error(f'Failed to move gripper to {config}. Stopping demonstration.')
                return False

            # Pause between movements to let controller fully settle
            # This prevents accumulated tracking errors
            time.sleep(2.0)

        self.logger.info('Gripper movement demonstration completed successfully!')
        return True

    def run_demonstration(self):
        """Run the full demonstration."""
        self.logger.info('')
        self.logger.info('#' * 60)
        self.logger.info('# Summit XL MoveItPy Demonstration')
        self.logger.info('#' * 60)
        self.logger.info('')

        # First demonstrate arm movements
        if not self.demonstrate_arm_movements():
            self.logger.error('Arm demonstration failed!')
            return False

        self.logger.info('')

        # Then demonstrate gripper movements
        if not self.demonstrate_gripper_movements():
            self.logger.error('Gripper demonstration failed!')
            return False

        self.logger.info('')
        self.logger.info('#' * 60)
        self.logger.info('# All demonstrations completed successfully!')
        self.logger.info('#' * 60)
        return True

    def shutdown(self):
        """Clean shutdown (EXACT wrapper pattern)."""
        logger.info("Shutting down MoveItWrapper")
        self.planning_components.clear()
        # MoveItPy handles its own cleanup
        
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    """Main entry point for the script."""
    try:
        # Create and run the example using EXACT wrapper pattern
        example = MoveItPyExample()
        success = example.run_demonstration()
        example.shutdown()

        # Exit with appropriate code
        exit(0 if success else 1)

    except KeyboardInterrupt:
        print('\nInterrupted by user')
        exit(0)
    except Exception as e:
        print(f'Error: {e}')
        exit(1)


if __name__ == '__main__':
    main()
