#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit.planning_interface import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
import sys

class PlannerTest(Node):
    def __init__(self):
        super().__init__('planner_test')
        # You can change 'manipulator' to the name of your move group
        self.arm_group = MoveGroupCommander('manipulator', node=self)
        
    def test_ompl_planners(self):
        planners = ['RRTConnect', 'BiTRRT', 'RRTstar', 'BITstar', 'LBTRRT']
        
        # Set a target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1.0
        
        for planner in planners:
            self.get_logger().info(f"Testing OMPL planner: {planner}")
            
            # Configure for OMPL with specific planner
            self.arm_group.set_planning_pipeline_id("ompl")
            self.arm_group.set_planner_id(planner)
            self.arm_group.set_planning_time(10.0)
            self.arm_group.set_num_planning_attempts(10)
            self.arm_group.set_max_velocity_scaling_factor(0.5)
            self.arm_group.set_max_acceleration_scaling_factor(0.5)
            
            # Plan to the target pose
            self.arm_group.set_pose_target(target_pose)
            success, plan, planning_time, error_code = self.arm_group.plan()
            
            if success:
                self.get_logger().info(f"Planning with {planner} succeeded in {planning_time:.2f} seconds!")
                return plan
            else:
                self.get_logger().warn(f"Planning with {planner} failed with error code: {error_code}")
        
        return None
    
    def test_pilz_planners(self):
        planners = ['PTP', 'LIN', 'CIRC']
        
        # Set a target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1.0
        
        for planner in planners:
            self.get_logger().info(f"Testing Pilz planner: {planner}")
            
            # Configure for Pilz with specific planner
            self.arm_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            self.arm_group.set_planner_id(planner)
            self.arm_group.set_max_velocity_scaling_factor(0.5)
            self.arm_group.set_max_acceleration_scaling_factor(0.5)
            
            # Plan to the target pose
            self.arm_group.set_pose_target(target_pose)
            
            # For CIRC motion we need waypoints
            if planner == 'CIRC':
                # For CIRC we need to define waypoints
                current_pose = self.arm_group.get_current_pose().pose
                waypoints = []
                
                # Add current pose
                waypoints.append(current_pose)
                
                # Add a via point (needed for CIRC)
                via_point = PoseStamped()
                via_point.header.frame_id = "base_link"
                via_point.pose.position.x = current_pose.position.x + 0.2
                via_point.pose.position.y = current_pose.position.y + 0.2
                via_point.pose.position.z = current_pose.position.z
                via_point.pose.orientation = current_pose.orientation
                waypoints.append(via_point.pose)
                
                # Add target pose
                waypoints.append(target_pose.pose)
                
                # Plan with waypoints
                (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
                success = fraction > 0.95
            else:
                # For PTP and LIN
                success, plan, planning_time, error_code = self.arm_group.plan()
            
            if success:
                self.get_logger().info(f"Planning with {planner} succeeded!")
                return plan
            else:
                self.get_logger().warn(f"Planning with {planner} failed!")
        
        return None
    
    def execute_plan(self, plan):
        if plan:
            self.get_logger().info("Executing plan...")
            self.arm_group.execute(plan, wait=True)
            self.get_logger().info("Execution complete!")
            return True
        else:
            self.get_logger().error("No valid plan to execute!")
            return False

def main():
    rclpy.init()
    node = PlannerTest()
    
    # Try OMPL planners first
    plan = node.test_ompl_planners()
    
    # If OMPL fails, try Pilz planners
    if not plan:
        node.get_logger().info("All OMPL planners failed, trying Pilz planners...")
        plan = node.test_pilz_planners()
    
    # Execute successful plan if any
    node.execute_plan(plan)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
