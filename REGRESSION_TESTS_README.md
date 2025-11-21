# Regression Tests for Summit XL Robot

This document describes the comprehensive regression test suite for the Summit XL robot with UR arm and Robotiq gripper.

## Overview

The regression test suite verifies that the complete robot system works correctly by testing:
1. **Simulation readiness** - Gazebo simulation is running and publishing data
2. **Navigation functionality** - Nav2 can navigate the robot to goal positions
3. **MoveIt configurations** - All pre-set arm and gripper configurations are reachable
4. **MoveIt Servo motion** - Real-time servo control functionality for the arm

## Test Files

### Test Nodes

Located in `icclab_summit_xl/test/`:

1. **test_simulation_readiness.py**
   - Verifies simulation clock is being published
   - Checks joint states are being published
   - Validates all required joints are present
   - Confirms simulation time is advancing (not paused)

2. **test_navigation.py**
   - Verifies Nav2 action server is available
   - Checks odometry is being published
   - Tests navigation to a goal position
   - Validates the robot reaches the goal within tolerance (0.5m)

3. **test_moveit_configurations.py**
   - Verifies MoveIt is initialized correctly
   - Tests all pre-set arm configurations:
     - `home` - All joints at zero
     - `up` - Arm pointing upward
     - `docked` - Arm in compact position
     - `look_forward` - Arm positioned to look ahead
   - Tests all pre-set gripper configurations:
     - `open` - Gripper fully open
     - `closed` - Gripper closed
   - Cycles through configurations to ensure transitions work

4. **test_servo_motion.py**
   - Verifies MoveIt Servo node is running and responsive
   - Tests switching between command types (JOINT_JOG, TWIST)
   - Tests joint jog commands move individual joints
   - Tests twist (Cartesian velocity) commands move the arm
   - Tests circular motion patterns with twist commands
   - Verifies stop commands work correctly
   - Checks collision detection during safe motions

### Launch Files

Located in `icclab_summit_xl/launch/`:

1. **regression_test.launch.py**
   - Comprehensive integration test
   - Starts simulation, Nav2, and MoveIt
   - Runs all tests automatically with proper timing
   - Use this for CI/CD or complete system validation

2. **test_runner.launch.py**
   - Runs tests assuming system components are already running
   - Useful for development and debugging individual tests
   - Faster iteration during test development

## Usage

### Option 1: Complete Integration Test (Recommended for CI/CD)

Run the complete test suite that starts everything automatically:

```bash
ros2 launch icclab_summit_xl regression_test.launch.py
```

This will:
1. Start Gazebo simulation with Summit XL robot
2. Wait 10 seconds, then start Nav2
3. Wait 15 seconds, then start MoveIt move_group
4. Wait 20 seconds, then start MoveIt Servo
5. Run simulation readiness test at 25 seconds
6. Run navigation test (after simulation test completes)
7. Run MoveIt configurations test (after navigation test completes)
8. Run MoveIt Servo motion test (after MoveIt test completes)

**Expected duration:** 5-7 minutes

**Note:** The tests run in sequence with delays to ensure components are fully initialized.

### Option 2: Manual Test Execution (For Development)

Start each component manually in separate terminals:

**Terminal 1: Start Simulation**
```bash
ros2 launch icclab_summit_xl summit_xl_simulation_ign.launch.py
```

Wait for Gazebo to fully load and the robot to spawn (look for "Spawn service called successfully").

**Terminal 2: Start Nav2**
```bash
ros2 launch icclab_summit_xl summit_xl_nav2.launch.py rviz:=false
```

Wait for Nav2 to initialize (look for "Creating bond timer").

**Terminal 3: Start MoveIt**
```bash
ros2 launch icclab_summit_xl_move_it_config move_group.launch.py
```

Wait for MoveIt to load (look for "You can start planning now!").

**Terminal 4: Start MoveIt Servo**
```bash
ros2 launch icclab_summit_xl_move_it_config servo_demo.launch.py start_rviz:=false
```

Wait for Servo to initialize (look for servo status messages).

**Terminal 5: Run Tests**

Run individual tests:
```bash
# Test 1: Simulation readiness
ros2 run icclab_summit_xl test_simulation_readiness.py

# Test 2: Navigation
ros2 run icclab_summit_xl test_navigation.py

# Test 3: MoveIt configurations
ros2 run icclab_summit_xl test_moveit_configurations.py

# Test 4: MoveIt Servo motion
ros2 run icclab_summit_xl test_servo_motion.py
```

Or run all tests via launch file:
```bash
ros2 launch icclab_summit_xl test_runner.launch.py
```

### Option 3: Running Individual Tests

You can also run individual test files directly:

```bash
# Run with Python unittest
python3 /path/to/test_simulation_readiness.py

# Or use pytest
pytest-3 /path/to/test_simulation_readiness.py -v
```

## Test Details

### Simulation Readiness Test

**Tests performed:**
- ✓ Clock topic `/clock` is being published
- ✓ Joint states topic `/joint_states` is being published
- ✓ All required joints are present:
  - Arm joints: `arm_shoulder_pan_joint`, `arm_shoulder_lift_joint`, `arm_elbow_joint`, `arm_wrist_1_joint`, `arm_wrist_2_joint`, `arm_wrist_3_joint`
  - Gripper joints: `finger_joint`
- ✓ Simulation time is advancing (not paused)

**Success criteria:**
- All topics are published within timeout
- All required joints are present in joint states
- Simulation time advances by at least 0.5 seconds in 3 seconds of real time

### Navigation Test

**Tests performed:**
- ✓ Nav2 `navigate_to_pose` action server is available
- ✓ Odometry is being published on `/odom`
- ✓ Robot can navigate from (0, 2) to (3, 2) on tugbot_depot map
- ✓ Robot reaches goal within tolerance

**Success criteria:**
- Action server responds within 15 seconds
- Navigation completes within 90 seconds
- Final position is within 0.5m of goal position

**Notes:**
- Initial pose is set to (0, 2) with theta=0
- Goal position (3, 2) is chosen as a known free space on tugbot_depot map
- Tolerance of 0.5m accounts for dynamic obstacles and planner behavior

### MoveIt Configurations Test

**Tests performed:**
- ✓ MoveItPy initializes successfully
- ✓ Planning components for arm and gripper are available
- ✓ Each arm configuration is reachable:
  - `home` - Reset position
  - `up` - Vertical reach position
  - `docked` - Compact storage position
  - `look_forward` - Camera viewing position
- ✓ Each gripper configuration is reachable:
  - `open` - Ready to grasp
  - `closed` - Grasping position
- ✓ Configuration cycles complete without errors

**Success criteria:**
- MoveIt initializes without errors
- Planning succeeds for all configurations
- Execution completes for all configurations
- No collision or kinematic errors occur

**Notes:**
- Each configuration test includes planning and execution
- 0.5 second settling time between motions
- Cycle tests ensure smooth transitions between configurations

### MoveIt Servo Motion Test

**Tests performed:**
- ✓ Servo status is being published
- ✓ Can switch to joint jog (JOINT_JOG) mode
- ✓ Can switch to twist (TWIST/Cartesian) mode
- ✓ Joint jog commands move individual joints
- ✓ Twist commands move the end-effector in Cartesian space
- ✓ Circular motion patterns work correctly
- ✓ Stop commands halt motion
- ✓ No collision warnings during safe motions

**Success criteria:**
- Servo node is running and publishing status
- Command type switching succeeds
- Joint jog commands produce measurable joint motion (>0.05 rad)
- Twist commands produce measurable arm motion (>0.01 rad total)
- Circular motion completes without errors
- No collision or singularity halts during test motions

**Notes:**
- **IMPORTANT:** Test automatically moves arm to a safe position away from singularities before testing
- Safe position matches the one used in `setup_servo.py` from the servo demo
- Tests use small, safe velocities to avoid singularities
- Joint jog test moves wrist_3 joint (safe to move)
- Twist tests use small upward motions (Z-axis)
- Circular motion uses 10cm radius in YZ plane (vertical circle) for 5 seconds
- All motions are designed to be collision-free

## Expected Results

### All Tests Pass

```
======================================================================
Ran 24 tests in 240.567s

OK
```

All tests should pass with green checkmarks (✓) in the logs. The test count includes:
- 3 simulation readiness tests
- 3 navigation tests
- 9 MoveIt configuration tests
- 9 MoveIt Servo motion tests

### Common Failures and Solutions

#### Test 1: Simulation Readiness Failures

**Failure:** "Simulation clock not being published"
- **Cause:** Gazebo not started or still loading
- **Solution:** Wait longer, or check Gazebo launched correctly

**Failure:** "Joint states not being published"
- **Cause:** Robot not spawned or controllers not loaded
- **Solution:** Check simulation logs for spawn errors

**Failure:** "Missing required joints"
- **Cause:** Wrong robot model or URDF issue
- **Solution:** Verify correct URDF is loaded, check robot_description topic

**Failure:** "Simulation time not advancing"
- **Cause:** Simulation is paused
- **Solution:** Press play in Gazebo GUI or ensure simulation starts unpaused

#### Test 2: Navigation Failures

**Failure:** "Nav2 action server not available"
- **Cause:** Nav2 not started or initialization failed
- **Solution:** Check Nav2 logs, ensure map is loaded correctly

**Failure:** "Navigation did not complete successfully"
- **Cause:** Path blocked, robot stuck, or timeout too short
- **Solution:** Check map and initial pose, increase timeout, verify localization

**Failure:** "Robot did not reach goal"
- **Cause:** Goal unreachable, localization drift, or obstacles
- **Solution:** Choose different goal, check map, verify odometry

#### Test 3: MoveIt Failures

**Failure:** "MoveItPy not initialized"
- **Cause:** MoveIt move_group not running or configuration error
- **Solution:** Check move_group logs, verify SRDF and configuration files

**Failure:** "Planning failed for [configuration]"
- **Cause:** Configuration unreachable, joint limits, or collision
- **Solution:** Check joint limits, verify SRDF configuration values

**Failure:** "Execution failed for [configuration]"
- **Cause:** Controller not responding or trajectory execution error
- **Solution:** Check controller status, verify controller configuration

#### Test 4: MoveIt Servo Failures

**Failure:** "Servo status not being published"
- **Cause:** Servo node not started or failed to initialize
- **Solution:** Check that servo_demo.launch.py was run, verify move_group is running first

**Failure:** "Failed to switch to [mode] mode"
- **Cause:** Servo service not available or in invalid state
- **Solution:** Check servo node logs, verify servo is fully initialized

**Failure:** "Joint/Twist command did not produce motion"
- **Cause:** Commands not reaching servo, wrong command type active, or arm at limit
- **Solution:** Verify command type is correct, check joint limits, review servo logs

**Failure:** "Collision detected during safe motion"
- **Cause:** Self-collision due to arm configuration or planning scene issues
- **Solution:** Check arm starting position, verify planning scene is accurate

**Failure:** "Servo halted due to singularity"
- **Cause:** Arm too close to kinematic singularity
- **Solution:** Adjust singularity thresholds in moveit_servo.yaml or use different test motions

## Integration with CI/CD

### GitHub Actions Example

```yaml
name: Regression Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout code
        uses: actions/checkout@v2

      - name: Setup ROS 2 Jazzy
        uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: jazzy

      - name: Build workspace
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build --symlink-install

      - name: Run regression tests
        run: |
          source install/setup.bash
          ros2 launch icclab_summit_xl regression_test.launch.py
```

### Local CI Script

Create a script `run_tests.sh`:

```bash
#!/bin/bash
set -e

echo "Building workspace..."
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

echo "Running regression tests..."
source install/setup.bash
ros2 launch icclab_summit_xl regression_test.launch.py

echo "All tests passed!"
```

Make it executable and run:
```bash
chmod +x run_tests.sh
./run_tests.sh
```

## Test Configuration

### Timeouts

Default timeouts can be adjusted in the test files:

- **Simulation readiness:** 10-15 seconds for topic availability
- **Navigation:** 90 seconds for navigation completion
- **MoveIt:** 15 seconds per configuration

### Tolerances

- **Navigation position tolerance:** 0.5m (adjustable in test_navigation.py)
- **MoveIt goal tolerance:** Default MoveIt tolerances (0.01 rad for joints)

### Map and Goals

The navigation test uses:
- **Map:** `tugbot_depot` (default in summit_xl_nav2.launch.py)
- **Initial pose:** (0.0, 2.0, 0.0)
- **Goal pose:** (3.0, 2.0, 0.0)

To use a different map or goals, modify `test_navigation.py` or create a custom test.

## Extending the Tests

### Adding New Test Cases

1. Create a new test file in `icclab_summit_xl/test/`:
```python
#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node

class MyNewTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('my_test_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_my_feature(self):
        # Your test code here
        self.assertTrue(True)

if __name__ == '__main__':
    import sys
    filtered_argv = [arg for arg in sys.argv if not arg.startswith('__')]
    unittest.main(argv=filtered_argv)
```

2. Make it executable:
```bash
chmod +x test/my_new_test.py
```

3. Add to CMakeLists.txt:
```cmake
install(PROGRAMS
  test/my_new_test.py
  DESTINATION lib/${PROJECT_NAME}
)
```

4. Add to regression_test.launch.py:
```python
my_test = TimerAction(
    period=40.0,
    actions=[
        LogInfo(msg='Running my new test...'),
        Node(
            package='icclab_summit_xl',
            executable='my_new_test.py',
            name='my_new_test',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]
)
```

### Adding New MoveIt Configurations

1. Define the configuration in `summit_xl.srdf`:
```xml
<group_state name="my_config" group="arm">
    <joint name="arm_shoulder_pan_joint" value="1.57"/>
    <!-- other joints... -->
</group_state>
```

2. Add test case in `test_moveit_configurations.py`:
```python
def test_arm_my_config(self):
    """Test moving arm to my_config configuration."""
    success = self._plan_and_execute(self.arm_group, 'my_config')
    self.assertTrue(success, "Failed to move arm to my_config")
```

## Troubleshooting

### Gazebo Issues

**Problem:** Gazebo crashes or freezes
- Check system resources (RAM, GPU)
- Try headless mode
- Reduce simulation complexity

**Problem:** Robot falls through floor
- Check physics engine settings
- Verify ground plane collision
- Check robot mass/inertia values

### Nav2 Issues

**Problem:** Robot doesn't localize
- Check initial pose is set correctly
- Verify map matches environment
- Check sensor data (laser scans)

**Problem:** Planner fails to find path
- Check inflation radius in costmap
- Verify goal is in free space
- Check planner parameters

### MoveIt Issues

**Problem:** Planning always fails
- Check joint limits in URDF
- Verify collision meshes
- Check planning scene

**Problem:** Execution doesn't start
- Verify controllers are running
- Check controller names match
- Verify trajectory execution is enabled

## Performance Benchmarks

Expected test durations on typical hardware (Intel i7, 16GB RAM, GTX 1060):

- **Simulation readiness:** 5-10 seconds
- **Navigation test:** 60-90 seconds
- **MoveIt configurations:** 90-120 seconds
- **MoveIt Servo motion:** 60-90 seconds
- **Total (all tests):** 5-7 minutes

## Contributing

When adding new features to the Summit XL robot:

1. Write regression tests for the new functionality
2. Ensure all existing tests still pass
3. Update this documentation
4. Run the complete test suite before submitting PR

## References

- [ROS 2 Testing Guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html)
- [unittest Documentation](https://docs.python.org/3/library/unittest.html)
- [Nav2 Testing](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)
- [MoveIt Testing](https://moveit.picknik.ai/main/doc/examples/tests/tests_tutorial.html)
