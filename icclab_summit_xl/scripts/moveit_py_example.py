#!/usr/bin/env python3
"""
Summit XL MoveItPy demonstration script.

Exercises the arm and gripper through their named SRDF configurations using
the reusable :class:`SummitArm` wrapper from ``summit_arm.py``.
"""

import os
import sys
import time
import logging

# Allow importing summit_arm when this script is run directly (not via ROS install)
sys.path.insert(0, os.path.dirname(__file__))
from summit_arm import SummitArm

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def demonstrate_arm(arm: SummitArm) -> bool:
    """Move the arm through a standard set of named configurations."""
    logger.info("=" * 60)
    logger.info("DEMONSTRATING ARM MOVEMENTS")
    logger.info("=" * 60)

    for config in ["home", "up", "docked", "look_forward", "home"]:
        if not arm.move_to_named_configuration("arm", config):
            logger.error(f"Failed to move arm to '{config}'. Stopping.")
            return False
        time.sleep(1.0)

    logger.info("Arm demonstration completed successfully!")
    return True


def demonstrate_gripper(arm: SummitArm) -> bool:
    """Open and close the gripper."""
    logger.info("=" * 60)
    logger.info("DEMONSTRATING GRIPPER MOVEMENTS")
    logger.info("=" * 60)

    for config in ["open", "closed"]:
        if not arm.move_to_named_configuration("gripper", config):
            logger.error(f"Failed to move gripper to '{config}'. Stopping.")
            return False
        time.sleep(1.0)

    logger.info("Gripper demonstration completed successfully!")
    return True


def run_demonstration(arm: SummitArm) -> bool:
    """Run the full arm + gripper demonstration."""
    logger.info("")
    logger.info("#" * 60)
    logger.info("# Summit XL MoveItPy Demonstration")
    logger.info("#" * 60)
    logger.info("")

    if not demonstrate_arm(arm):
        logger.error("Arm demonstration failed!")
        return False

    logger.info("")

    if not demonstrate_gripper(arm):
        logger.error("Gripper demonstration failed!")
        return False

    logger.info("")
    logger.info("#" * 60)
    logger.info("# All demonstrations completed successfully!")
    logger.info("#" * 60)
    return True


def main():
    arm = None
    try:
        arm = SummitArm()
        success = run_demonstration(arm)
        time.sleep(0.5)
        arm.shutdown()
        # os._exit avoids MoveItPy C++ destructor segfault (known Jazzy issue)
        os._exit(0 if success else 1)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        if arm:
            arm.shutdown()
        os._exit(0)

    except Exception as exc:
        print(f"Error: {exc}")
        if arm:
            arm.shutdown()
        os._exit(1)


if __name__ == "__main__":
    main()
