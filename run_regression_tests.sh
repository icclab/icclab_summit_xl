#!/bin/bash
# Regression test runner script for Summit XL robot
# This script runs the complete regression test suite

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Print banner
echo -e "${GREEN}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║     Summit XL Regression Test Suite                       ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS 2 is not sourced!${NC}"
    echo "Please run: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo -e "${YELLOW}ROS Distribution: $ROS_DISTRO${NC}"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo -e "${YELLOW}Workspace not built. Building now...${NC}"
    colcon build --symlink-install
fi

# Source the workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash

# Parse arguments
TEST_TYPE="${1:-all}"
HEADLESS="${2:-false}"

echo -e "${YELLOW}Test type: $TEST_TYPE${NC}"
echo -e "${YELLOW}Headless mode: $HEADLESS${NC}"
echo ""

# Run the regression tests
echo -e "${GREEN}Starting regression tests...${NC}"
echo ""

if [ "$TEST_TYPE" = "all" ]; then
    echo -e "${YELLOW}Running complete regression test suite...${NC}"
    ros2 launch icclab_summit_xl regression_test.launch.py headless:=$HEADLESS
elif [ "$TEST_TYPE" = "simulation" ]; then
    echo -e "${YELLOW}Running simulation readiness test only...${NC}"
    echo "Note: Make sure simulation is running!"
    ros2 run icclab_summit_xl test_simulation_readiness.py
elif [ "$TEST_TYPE" = "navigation" ]; then
    echo -e "${YELLOW}Running navigation test only...${NC}"
    echo "Note: Make sure simulation and Nav2 are running!"
    ros2 run icclab_summit_xl test_navigation.py
elif [ "$TEST_TYPE" = "moveit" ]; then
    echo -e "${YELLOW}Running MoveIt configurations test only...${NC}"
    echo "Note: Make sure simulation and MoveIt are running!"
    ros2 run icclab_summit_xl test_moveit_configurations.py
else
    echo -e "${RED}Invalid test type: $TEST_TYPE${NC}"
    echo "Usage: $0 [all|simulation|navigation|moveit] [headless]"
    echo "  all          - Run complete test suite (default)"
    echo "  simulation   - Run simulation readiness test only"
    echo "  navigation   - Run navigation test only"
    echo "  moveit       - Run MoveIt configurations test only"
    echo "  headless     - true/false (optional, default: false)"
    exit 1
fi

# Check exit code
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}"
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║     ✓ ALL TESTS PASSED!                                   ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}"
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║     ✗ TESTS FAILED!                                       ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    exit 1
fi
