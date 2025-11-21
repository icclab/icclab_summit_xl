#!/bin/bash
# Helper script to run regression tests with ROS logs hidden

# Create timestamped log directory
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="/tmp/summit_xl_regression_${TIMESTAMP}"
LOG_FILE="${LOG_DIR}/ros_logs.log"
mkdir -p "${LOG_DIR}"

echo "============================================================"
echo "Summit XL Regression Test Runner"
echo "============================================================"
echo "ROS node logs will be saved to: ${LOG_FILE}"
echo "Only test output will be shown on screen"
echo "Launch will exit automatically after all tests complete"
echo "============================================================"
echo ""

# Source the workspace
cd /home/ros/colcon_ws
source install/setup.bash

# Start tail -f with grep filter in background to show only test output
tail -f "${LOG_FILE}" 2>/dev/null | \
    grep --line-buffered -E "(Running|test_|OK|FAILED|Error|Traceback|Ran [0-9]+ test|All regression tests|Shutting down|^===)" | \
    grep --line-buffered -v "Possible reasons" &
TAIL_PID=$!

# Run the regression tests, redirect ALL output to log file
ros2 launch icclab_summit_xl regression_test.launch.py > "${LOG_FILE}" 2>&1
EXIT_CODE=$?

# Kill the tail process
kill $TAIL_PID 2>/dev/null
wait $TAIL_PID 2>/dev/null

echo ""
echo "============================================================"
echo "Regression tests completed with exit code: ${EXIT_CODE}"
echo "Full logs saved to: ${LOG_FILE}"
echo "============================================================"

exit ${EXIT_CODE}
