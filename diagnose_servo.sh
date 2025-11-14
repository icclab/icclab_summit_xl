#!/bin/bash
# Diagnostic script to check MoveIt Servo setup

echo "=== MoveIt Servo Diagnostic ==="
echo ""

echo "1. Checking if joint_states topic exists:"
timeout 2 ros2 topic list | grep joint_states || echo "  ❌ No joint_states topic found"
echo ""

echo "2. Checking joint_states publication rate:"
timeout 5 ros2 topic hz /joint_states 2>&1 | head -5 || echo "  ❌ Not publishing"
echo ""

echo "3. Checking servo_node topics:"
timeout 2 ros2 node info /servo_node 2>&1 | grep -A 20 "Subscribers:" || echo "  ❌ servo_node not running"
echo ""

echo "4. Checking if servo is subscribed to joint_states:"
timeout 2 ros2 topic info /joint_states | grep servo || echo "  ⚠️  servo_node not subscribed to joint_states"
echo ""

echo "5. Checking servo parameters:"
ros2 param list /servo_node 2>&1 | grep -E "(joint_topic|move_group|planning_frame)" || echo "  ❌ Can't get servo params"
echo ""

echo "6. Checking servo status topic:"
timeout 2 ros2 topic list | grep servo.*status || echo "  ⚠️  No servo status topic"
echo ""

echo "7. Testing twist command publication:"
echo "  Publishing a test twist command..."
timeout 2 ros2 topic pub --once /servo_node/twist_cmds geometry_msgs/msg/TwistStamped "{header: {frame_id: 'arm_base_link'}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" 2>&1 | head -3
echo ""

echo "8. Checking if move_group is running:"
timeout 2 ros2 node list | grep move_group || echo "  ❌ move_group not running"
echo ""

echo "=== End Diagnostic ==="
