#!/usr/bin/env python3
"""
Test to verify that MoveIt can reach all pre-set configurations.
This checks that:
1. MoveIt move_group is running
2. The arm can reach all pre-set configurations (home, up, docked, look_forward)
3. The gripper can reach all pre-set configurations (open, closed)
4. No planning or execution errors occur

Uses the EXACT same initialization pattern as moveit_py_example.py
"""

import unittest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
import time
import os
import logging
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger(__name__)


class MoveItConfigurationsTest(unittest.TestCase):

    # Pre-set configurations from summit_xl.srdf
    ARM_CONFIGURATIONS = {
        'home': {
            'arm_elbow_joint': 0.0,
            'arm_shoulder_lift_joint': 0.0,
            'arm_shoulder_pan_joint': 0.0,
            'arm_wrist_1_joint': 0.0,
            'arm_wrist_2_joint': 0.0,
            'arm_wrist_3_joint': 0.0,
        },
        'up': {
            'arm_elbow_joint': 0.0,
            'arm_shoulder_lift_joint': -1.5708,
            'arm_shoulder_pan_joint': 0.0,
            'arm_wrist_1_joint': 0.0,
            'arm_wrist_2_joint': 0.0,
            'arm_wrist_3_joint': 0.0,
        },
        'docked': {
            'arm_elbow_joint': -2.8291,
            'arm_shoulder_lift_joint': 0.0,
            'arm_shoulder_pan_joint': 0.0,
            'arm_wrist_1_joint': 0.0,
            'arm_wrist_2_joint': 0.0,
            'arm_wrist_3_joint': 0.0,
        },
        'look_forward': {
            'arm_elbow_joint': -2.8291,
            'arm_shoulder_lift_joint': -0.243,
            'arm_shoulder_pan_joint': 0.0,
            'arm_wrist_1_joint': -0.7984,
            'arm_wrist_2_joint': 1.5621,
            'arm_wrist_3_joint': 0.0,
        },
    }

    GRIPPER_CONFIGURATIONS = {
        'open': {
            'finger_joint': 0.01,
        },
        'closed': {
            'finger_joint': 0.69,
        },
    }

    @classmethod
    def setUpClass(cls):
        """Initialize MoveItPy using EXACT pattern from moveit_py_example.py"""
        # Initialize ROS 2 if not already initialized
        if not rclpy.ok():
            logger.info("Initializing ROS 2 context")
            rclpy.init()

        # Use the exact same node name pattern as working example
        node_name = "moveit_py_test_summit"

        # Build MoveIt configuration using MoveItConfigsBuilder (EXACT pattern from example)
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

            # Build MoveIt configuration (EXACT pattern from working example)
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

        # Initialize MoveItPy with the configuration (EXACT pattern from example)
        logger.info(f"Initializing MoveItPy with node name: {node_name}")

        try:
            # Convert config to dictionary and pass as node parameters
            config_dict = moveit_config.to_dict()

            # Add use_sim_time parameter if configured (like example)
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

            # Create MoveItPy (EXACT pattern from example)
            cls.moveit = MoveItPy(node_name=node_name, config_dict=config_dict)
            logger.info("MoveItPy initialized successfully")
        except RuntimeError as e:
            logger.error(f"Failed to initialize MoveItPy: {e}")
            logger.error(f"Config keys that were provided: {list(config_dict.keys()) if 'config_dict' in locals() else 'config_dict not created'}")
            raise

        # Get robot model and planning scene monitor
        cls.robot_model = cls.moveit.get_robot_model()
        cls.planning_scene_monitor = cls.moveit.get_planning_scene_monitor()

        # Get planning components
        try:
            cls.arm_group = cls.moveit.get_planning_component("arm")
            cls.gripper_group = cls.moveit.get_planning_component("gripper")
            logger.info('Planning components retrieved successfully')
        except Exception as e:
            logger.error(f'Failed to get planning components: {e}')
            raise

        # Wait for planning scene to receive joint state updates
        # MoveItPy handles its own spinning internally
        logger.info('Waiting for planning scene to initialize...')
        time.sleep(5.0)  # Give time for subscriptions and initial messages
        logger.info('Planning scene initialization complete')

    @classmethod
    def tearDownClass(cls):
        """Clean shutdown - just clear planning components.

        Don't explicitly shutdown rclpy or delete moveit object.
        Let Python's normal exit sequence handle cleanup to avoid
        crashes from MoveItPy's internal threads being torn down improperly.
        """
        logger.info("Shutting down test suite")
        # Don't call rclpy.shutdown() - let Python handle cleanup

    def _plan_and_execute(self, planning_component, goal_state_name, timeout=15.0):
        """Plan and execute motion to a named goal state using EXACT example pattern."""
        logger.info(f'Planning to: {goal_state_name}')

        # Set start state to current state - MoveItPy handles spinning internally
        planning_component.set_start_state_to_current_state()

        # Set goal state
        planning_component.set_goal_state(configuration_name=goal_state_name)

        # Plan
        logger.info('Planning...')
        start_time = time.time()
        plan_result = planning_component.plan()
        planning_time = time.time() - start_time

        if not plan_result:
            logger.error(f'Planning failed for {goal_state_name}')
            return False

        logger.info(f'Planning successful (took {planning_time:.2f}s)')

        # Extract RobotTrajectory from MotionPlanResponse if needed (EXACT example pattern)
        robot_trajectory = plan_result
        if hasattr(plan_result, 'trajectory'):
            logger.debug("Extracting trajectory from MotionPlanResponse")
            robot_trajectory = plan_result.trajectory

        # Execute using example pattern (EXACT pattern)
        logger.info('Executing...')
        start_time = time.time()
        success = self.moveit.execute(robot_trajectory, controllers=[])
        execution_time = time.time() - start_time

        if success:
            logger.info(
                f'✓ Successfully moved to {goal_state_name} '
                f'(planning: {planning_time:.2f}s, execution: {execution_time:.2f}s)'
            )
        else:
            logger.warning(f'Execution failed for {goal_state_name}')
            return False

        # Wait for the robot to fully settle and joint states to stabilize
        # This prevents numerical precision errors in subsequent planning attempts
        time.sleep(2.0)

        return True

    def test_moveit_initialization(self):
        """Test that MoveItPy initialized successfully."""
        logger.info('Testing MoveIt initialization...')
        self.assertIsNotNone(self.moveit, "MoveItPy not initialized")
        self.assertIsNotNone(self.arm_group, "Arm planning component not available")
        self.assertIsNotNone(self.gripper_group, "Gripper planning component not available")
        logger.info('✓ MoveIt initialized successfully')

    def test_arm_home_configuration(self):
        """Test moving arm to home configuration."""
        logger.info('Testing arm home configuration...')
        success = self._plan_and_execute(self.arm_group, 'home')
        self.assertTrue(success, "Failed to move arm to home configuration")

    def test_arm_up_configuration(self):
        """Test moving arm to up configuration."""
        logger.info('Testing arm up configuration...')
        success = self._plan_and_execute(self.arm_group, 'up')
        self.assertTrue(success, "Failed to move arm to up configuration")

    def test_arm_docked_configuration(self):
        """Test moving arm to docked configuration."""
        logger.info('Testing arm docked configuration...')
        success = self._plan_and_execute(self.arm_group, 'docked')
        self.assertTrue(success, "Failed to move arm to docked configuration")

    def test_arm_look_forward_configuration(self):
        """Test moving arm to look_forward configuration."""
        logger.info('Testing arm look_forward configuration...')
        success = self._plan_and_execute(self.arm_group, 'look_forward')
        self.assertTrue(success, "Failed to move arm to look_forward configuration")

    def test_gripper_open_configuration(self):
        """Test moving gripper to open configuration."""
        logger.info('Testing gripper open configuration...')
        success = self._plan_and_execute(self.gripper_group, 'open')
        self.assertTrue(success, "Failed to move gripper to open configuration")

    def test_gripper_closed_configuration(self):
        """Test moving gripper to closed configuration."""
        logger.info('Testing gripper closed configuration...')
        success = self._plan_and_execute(self.gripper_group, 'closed')
        self.assertTrue(success, "Failed to move gripper to closed configuration")

    def test_arm_configuration_cycle(self):
        """Test cycling through all arm configurations."""
        logger.info('Testing arm configuration cycle...')

        configurations = ['home', 'up', 'docked', 'look_forward', 'home']

        for config in configurations:
            success = self._plan_and_execute(self.arm_group, config)
            self.assertTrue(
                success,
                f"Failed during arm configuration cycle at: {config}"
            )

        logger.info('✓ Successfully cycled through all arm configurations')

    def test_gripper_configuration_cycle(self):
        """Test cycling through all gripper configurations."""
        logger.info('Testing gripper configuration cycle...')

        # Cycle between open and closed multiple times
        configurations = ['open', 'closed', 'open', 'closed', 'open']

        for config in configurations:
            success = self._plan_and_execute(self.gripper_group, config)
            self.assertTrue(
                success,
                f"Failed during gripper configuration cycle at: {config}"
            )

        logger.info('✓ Successfully cycled through gripper configurations')


if __name__ == '__main__':
    import sys
    # Remove ROS args before running unittest
    filtered_argv = [arg for arg in sys.argv if not arg.startswith('__')]

    # Run tests and capture result
    runner = unittest.TextTestRunner()
    suite = unittest.TestLoader().loadTestsFromModule(sys.modules[__name__])
    result = runner.run(suite)

    # Use os._exit() to bypass Python's cleanup that causes MoveItPy segfault
    # This is a known issue with MoveItPy's C++ destructor in Jazzy
    exit_code = 0 if result.wasSuccessful() else 1
    os._exit(exit_code)
