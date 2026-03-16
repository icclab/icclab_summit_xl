#!/usr/bin/env python3
"""
SummitArm: reusable MoveItPy wrapper for the Summit XL robot arm.

Importable from other ROS 2 Python scripts::

    from summit_arm import SummitArm

    arm = SummitArm()
    arm.move_to_named_configuration("arm", "home")
    arm.shutdown()
"""

import os
import time
import logging
from typing import Dict, List, Optional

import rclpy
from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger(__name__)


def _find_moveit_cpp_yaml() -> Optional[str]:
    """Search known packages for a summit_xl MoveItPy YAML config file."""
    candidates = [
        ('icclab_summit_xl_move_it_config', 'config/summit_xl_moveit_py.yaml'),
        ('icclab_summit_xl',               'config/summit_xl_moveit_py.yaml'),
        ('icclab_summit_xl',               'config/moveit_py.yaml'),
    ]
    for package_name, relative_path in candidates:
        try:
            pkg_share = get_package_share_directory(package_name)
            full_path = os.path.join(pkg_share, relative_path)
            if os.path.exists(full_path):
                logger.info(f"Found moveit_py config: {full_path}")
                return full_path
        except Exception:
            continue
    logger.warning("No moveit_py YAML config found; using MoveIt defaults")
    return None


class SummitArm:
    """
    Thin MoveItPy wrapper for the Summit XL robot arm.

    Handles ROS 2 initialisation, MoveIt configuration loading, and exposes
    high-level motion primitives that other scripts can reuse.

    Parameters
    ----------
    use_sim_time:
        Pass ``True`` when running against a simulation (Gazebo / Ignition).
        Applies the QoS /clock workaround required by MoveItPy on Jazzy.
    node_name:
        Name of the internal MoveItPy node. Override only if you need to run
        multiple instances in the same process.
    init_sleep:
        Seconds to wait after MoveItPy starts so the planning scene can
        receive initial joint-state updates.
    """

    def __init__(
        self,
        use_sim_time: bool = True,
        node_name: str = "summit_arm",
        init_sleep: float = 5.0,
    ):
        if not rclpy.ok():
            logger.info("Initialising ROS 2 context")
            rclpy.init()

        moveit_config = self._build_moveit_config()
        self.moveit = self._init_moveit_py(node_name, moveit_config, use_sim_time)

        self.robot_model = self.moveit.get_robot_model()
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        self._planning_components: Dict[str, PlanningComponent] = {}

        logger.info("SummitArm initialised successfully")
        logger.info("Waiting for planning scene to initialise…")
        time.sleep(init_sleep)
        logger.info("Planning scene ready")

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _build_moveit_config(self):
        """Build and return the MoveIt configuration object."""
        try:
            builder = MoveItConfigsBuilder(
                "summit_xl", package_name="icclab_summit_xl_move_it_config"
            )
            builder = builder.robot_description(
                file_path="robots/summit_xls_icclab.urdf.xacro"
            )
            builder = builder.robot_description_semantic(
                file_path="config/summit_xl.srdf"
            )
            builder = builder.trajectory_execution(
                file_path="config/moveit_controllers.yaml"
            )
            builder = builder.planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            builder = builder.planning_pipelines(pipelines=["ompl"])

            yaml_path = _find_moveit_cpp_yaml()
            if yaml_path:
                builder = builder.moveit_cpp(file_path=yaml_path)

            return builder.to_moveit_configs()
        except Exception as exc:
            raise RuntimeError(
                f"Failed to build MoveIt configuration: {exc}"
            ) from exc

    def _init_moveit_py(self, node_name: str, moveit_config, use_sim_time: bool) -> MoveItPy:
        """Instantiate and return the MoveItPy object."""
        config_dict = moveit_config.to_dict()

        if use_sim_time:
            logger.info("Configuring MoveItPy for simulation time")
            config_dict["use_sim_time"] = True
            # Workaround for MoveItPy/Jazzy bug: QoS mismatch on /clock
            config_dict["qos_overrides"] = {
                "/clock": {
                    "subscription": {
                        "depth": 1,
                        "history": "keep_last",
                        "durability": "volatile",
                        "reliability": "best_effort",
                    }
                }
            }

        logger.debug(f"MoveIt config keys: {list(config_dict.keys())}")
        try:
            return MoveItPy(node_name=node_name, config_dict=config_dict)
        except RuntimeError as exc:
            logger.error(f"Failed to initialise MoveItPy: {exc}")
            raise

    # ------------------------------------------------------------------
    # Planning group access
    # ------------------------------------------------------------------

    def get_planning_groups(self) -> List[str]:
        """Return the names of all planning groups defined in the SRDF."""
        return self.robot_model.joint_model_group_names

    def get_planning_component(self, group_name: str) -> PlanningComponent:
        """Return a cached PlanningComponent for *group_name*.

        Raises ``ValueError`` if the group does not exist.
        """
        if group_name not in self._planning_components:
            if not self.robot_model.has_joint_model_group(group_name):
                raise ValueError(
                    f"Planning group '{group_name}' not found. "
                    f"Available: {self.get_planning_groups()}"
                )
            logger.debug(f"Creating PlanningComponent for '{group_name}'")
            self._planning_components[group_name] = (
                self.moveit.get_planning_component(group_name)
            )
        return self._planning_components[group_name]

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------

    def move_to_named_configuration(self, group_name: str, config_name: str) -> bool:
        """Plan and execute a move to a named SRDF configuration.

        Parameters
        ----------
        group_name:
            Planning group (e.g. ``"arm"``, ``"gripper"``).
        config_name:
            Named state defined in the SRDF (e.g. ``"home"``, ``"open"``).

        Returns
        -------
        bool
            ``True`` on success, ``False`` on planning or execution failure.
        """
        logger.info(f"Moving '{group_name}' → '{config_name}'")
        try:
            component = self.get_planning_component(group_name)
            component.set_start_state_to_current_state()
            component.set_goal_state(configuration_name=config_name)

            plan_result = component.plan()
            if not plan_result:
                logger.warning(f"Planning failed for '{group_name}' → '{config_name}'")
                return False

            robot_trajectory = (
                plan_result.trajectory
                if hasattr(plan_result, "trajectory")
                else plan_result
            )

            success = self.moveit.execute(robot_trajectory, controllers=[])
            if success:
                logger.info("Execution succeeded")
            else:
                logger.warning("Execution failed")
            return bool(success)

        except Exception as exc:
            logger.error(f"Error moving to '{config_name}': {exc}")
            return False

    def move_to_pose(
        self,
        pose_stamped: PoseStamped,
        group_name: str = "arm",
        end_effector_link: str = "arm_flange",
    ) -> bool:
        """Plan and execute a move to a Cartesian pose goal.

        Parameters
        ----------
        pose_stamped:
            Target pose.  The ``header.frame_id`` must be a frame known to
            the planning scene (e.g. ``"base_footprint"`` or a TF frame
            that MoveItPy can resolve).
        group_name:
            Planning group that owns the end-effector link.
        end_effector_link:
            The link that should reach *pose_stamped*.

        Returns
        -------
        bool
            ``True`` on success, ``False`` on planning or execution failure.
        """
        logger.info(
            f"Moving '{end_effector_link}' of '{group_name}' "
            f"to pose in frame '{pose_stamped.header.frame_id}'"
        )
        try:
            component = self.get_planning_component(group_name)
            component.set_start_state_to_current_state()
            component.set_goal_state(
                pose_stamped_msg=pose_stamped,
                pose_link=end_effector_link,
            )

            plan_result = component.plan()
            if not plan_result:
                logger.warning(f"Planning failed for pose goal on '{group_name}'")
                return False

            robot_trajectory = (
                plan_result.trajectory
                if hasattr(plan_result, "trajectory")
                else plan_result
            )

            success = self.moveit.execute(robot_trajectory, controllers=[])
            if success:
                logger.info("Pose execution succeeded")
            else:
                logger.warning("Pose execution failed")
            return bool(success)

        except Exception as exc:
            logger.error(f"Error moving to pose: {exc}")
            return False

    def publish_trajectory_for_visualization(
        self, trajectory, group_name: str, publisher
    ) -> None:
        """Publish a planned trajectory to RViz via *publisher*.

        Parameters
        ----------
        trajectory:
            A MoveItPy trajectory object (or a ``RobotTrajectory`` message).
        group_name:
            Used only for the debug log message.
        publisher:
            A ROS 2 ``Publisher[DisplayTrajectory]`` created by the caller.
        """
        display = DisplayTrajectory()
        display.model_id = "summit_xl"

        robot_traj_msg = (
            trajectory.get_robot_trajectory_msg()
            if hasattr(trajectory, "get_robot_trajectory_msg")
            else trajectory
        )
        display.trajectory.append(robot_traj_msg)
        publisher.publish(display)
        logger.debug(f"Published trajectory for '{group_name}' to /display_planned_path")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """Release planning components.

        Does **not** call ``rclpy.shutdown()`` or delete the MoveItPy object —
        Python's normal exit sequence handles teardown to avoid crashes from
        MoveItPy's internal C++ threads (known Jazzy issue).
        """
        logger.info("SummitArm shutting down")
        self._planning_components.clear()
