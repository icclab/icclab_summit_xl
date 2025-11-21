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
    grep --line-buffered -E "(Running.*test|test_.*\(|^OK$|^FAILED|Error|Traceback|Ran [0-9]+ test|All regression tests)" | \
    grep --line-buffered -v -E "(Possible reasons|preshutdown|lifecycle)" &
TAIL_PID=$!

# Run the regression tests, redirect ALL output to log file
ros2 launch icclab_summit_xl regression_test.launch.py > "${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

# Wait for launch process to complete
# Check if "All regression tests completed" appears in log
POLL_INTERVAL=2

echo ""
echo "Waiting for tests to complete..."

# Wait for completion message
while [ ! -f "${LOG_FILE}" ] || ! grep -q "All regression tests completed" "${LOG_FILE}" 2>/dev/null; do
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        # Launch process died unexpectedly
        break
    fi
    sleep $POLL_INTERVAL
done

echo "Tests completed, initiating shutdown..."

# Kill the tail/grep pipeline immediately to prevent any blocking
echo "Stopping log monitor..."
kill -TERM $TAIL_PID 2>/dev/null
sleep 0.2
if kill -0 $TAIL_PID 2>/dev/null; then
    kill -KILL $TAIL_PID 2>/dev/null
fi

# Give launch system 2 seconds to process the shutdown event
sleep 2

# Now forcefully kill everything - the ShutdownEvent should have been emitted already
echo "Forcing termination of all processes..."

# Kill entire process tree immediately
pkill -TERM -P $LAUNCH_PID 2>/dev/null
kill -TERM $LAUNCH_PID 2>/dev/null

# Wait only 3 seconds for graceful TERM shutdown
for i in {1..3}; do
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        echo "Launch process terminated gracefully"
        break
    fi
    echo "Waiting for graceful shutdown... (${i}s / 3s)"
    sleep 1
done

# Force kill if still alive after 3 seconds
if kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "Graceful shutdown timeout - sending SIGKILL..."
    pkill -KILL -P $LAUNCH_PID 2>/dev/null
    kill -KILL $LAUNCH_PID 2>/dev/null
    sleep 1
fi

# Don't wait on launch PID - it's already dead or will be dead
# Just capture whatever exit code we can
LAUNCH_EXIT_CODE=0

# Final cleanup: kill any remaining Gazebo/ROS processes from this test run
# This is a safety net in case child processes are still hanging around
echo "Performing final cleanup..."
pkill -f "gz sim" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
sleep 1

# Give a moment for any final log writes
sleep 0.5

# Parse log file to detect test failures
# Look for "FAILED" in unittest output (with process prefix like [ros2-17])
if grep -q "FAILED (failures=" "${LOG_FILE}" || grep -q "FAILED (errors=" "${LOG_FILE}"; then
    echo ""
    echo "============================================================"
    echo "❌ REGRESSION TESTS FAILED"
    echo "============================================================"
    echo ""
    # Count how many test suites failed
    FAILED_COUNT=$(grep -c "FAILED (failures=\|FAILED (errors=" "${LOG_FILE}")
    OK_COUNT=$(grep -c "] OK$" "${LOG_FILE}")

    echo "Test Results: $OK_COUNT passed, $FAILED_COUNT failed"
    echo ""
    echo "Failed test details:"
    echo ""
    # Show failure summary - extract failure details
    grep -B 10 "FAILED (failures=\|FAILED (errors=" "${LOG_FILE}" | grep -E "(FAIL:|AssertionError:|FAILED \()" | head -30
    echo ""
    echo "Full logs saved to: ${LOG_FILE}"
    echo "============================================================"
    exit 1
elif grep -q "] OK$" "${LOG_FILE}"; then
    # Check if ALL test runs passed (there should be 4 test runs)
    OK_COUNT=$(grep -c "] OK$" "${LOG_FILE}")
    if [ "$OK_COUNT" -ge 4 ]; then
        echo ""
        echo "============================================================"
        echo "✅ ALL REGRESSION TESTS PASSED"
        echo "============================================================"
        echo "All $OK_COUNT test suites completed successfully"
        echo "Full logs saved to: ${LOG_FILE}"
        echo "============================================================"
        exit 0
    else
        echo ""
        echo "============================================================"
        echo "⚠️  INCOMPLETE TEST RUN"
        echo "============================================================"
        echo "Only $OK_COUNT out of 4 expected test suites completed"
        echo "Full logs saved to: ${LOG_FILE}"
        echo "============================================================"
        exit 1
    fi
else
    echo ""
    echo "============================================================"
    echo "❌ TESTS DID NOT COMPLETE PROPERLY"
    echo "============================================================"
    echo "Could not find test completion markers in log"
    echo "Launch exit code: ${LAUNCH_EXIT_CODE}"
    echo "Full logs saved to: ${LOG_FILE}"
    echo "============================================================"
    exit 1
fi
