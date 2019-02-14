/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
using namespace std;

int main(int argc, char** argv)
{
  static const std::string NODE_NAME = "move_arm_to_joint_position"; 
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string ROBOT_DESCRIPTION = "/summit_xl/robot_description";
  static const moveit::planning_interface::MoveGroupInterface::Options CONFIG_OPTIONS(PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle); 

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  ros::WallDuration timeout_duration = ros::WallDuration(60);
  moveit::planning_interface::MoveGroupInterface move_group(CONFIG_OPTIONS, boost::shared_ptr<tf::Transformer>(), timeout_duration);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  joint_model_group->printGroupInfo();

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("arm_tool0");
  visual_tools.deleteAllMarkers();

//  // Remote control is an introspection tool that allows users to step through a high level script
//  // via buttons and keyboard shortcuts in RViz
//  visual_tools.loadRemoteControl();
// 
//  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED(NODE_NAME, "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED(NODE_NAME, "End effector link: %s", move_group.getEndEffectorLink().c_str());


//  // Now, we call the planner to compute the plan and visualize it.
//  // Note that we are just planning, not asking move_group
//  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
//  ROS_INFO_NAMED(NODE_NAME, "RobotState: %s", *current_state);
//  current_state->printStateInfo();

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  
  // position constraint
  moveit_msgs::PositionConstraint pcm;
  moveit_msgs::BoundingVolume volume;
  shape_msgs::SolidPrimitive bounding_box;
  bounding_box.type = bounding_box.BOX;
  std::vector<double> dim;
  dim.push_back(2); // x
  dim.push_back(1); // y
  dim.push_back(2); // z
  bounding_box.dimensions = dim;
  pcm.constraint_region.primitives.push_back(bounding_box);
  geometry_msgs::Pose box_pose;
  geometry_msgs::Point position;
  position.x = 0;
  position.y = 0;
  position.z = 0;
  geometry_msgs::Quaternion orientation;
  orientation.w = 1;
  box_pose.position = position;
  box_pose.orientation = orientation;
  pcm.constraint_region.primitive_poses.push_back(box_pose);
  pcm.link_name = "arm_tool0";
  pcm.header.frame_id = "summit_xl_base_footprint";
  geometry_msgs::Vector3 offset;
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0.3;
  pcm.constraint_region = volume;
  pcm.weight = 1.0;
  
//  // joint constraint
//  moveit_msgs::JointConstraint jcm;
//  jcm.joint_name = "arm_shoulder_pan_joint";
//  jcm.position = 0;
//  jcm.tolerance_above = 0.3;
//  jcm.tolerance_below = 0.3;
//  jcm.weight = 20;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints constraints;
  constraints.position_constraints.push_back(pcm);
//  constraints.joint_constraints.push_back(jcm);
  move_group.setPathConstraints(constraints);
  move_group.setPlanningTime(300.0);
  
  std::vector<double> joint_group_positions;
  std::vector<string> joint_names = joint_model_group->getActiveJointModelNames();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  for (int i = 0; i < joint_group_positions.size(); i++){
    ROS_INFO_NAMED(NODE_NAME, "Current joint %s position: %f", joint_names[i].c_str(), joint_group_positions[i]);
  }
  // Now, let's modify the joints, plan to the new joint space goal and visualize the plan.
//  name: [arm_elbow_joint, arm_shoulder_lift_joint, arm_shoulder_pan_joint, arm_wrist_1_joint,
//  arm_wrist_2_joint, arm_wrist_3_joint]
//position: [2.7645761966705322, -3.090015236531393, 3.119974374771118, 3.392937183380127, -4.649675909672872, 3.1415650844573975]
  joint_group_positions[0] = 0;  // radians arm_shoulder_pan_joint position
  joint_group_positions[1] = 0;  // radians arm_shoulder_lift_joint position
  joint_group_positions[2] = -2.70;  // radians arm_elbow_joint
  joint_group_positions[3] = 0;  // radians arm_wrist_1_joint 
  joint_group_positions[4] = -1.52;  // radians arm_wrist_2_joint 
  joint_group_positions[5] = 0;  // radians arm_wrist_3_joint
  move_group.setJointValueTarget(joint_group_positions);
  
  // since we plan to use this as a script, let's move slowly
  move_group.setMaxVelocityScalingFactor(0.05);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED(NODE_NAME, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
  if (success){
    cout << "Please verify that the plan will not hit any objects uisng rviz.\n";
    cout << "Do you want to proceed with the planned movement? (y/N): ";
    string reply;
    getline (cin, reply);
    if (reply.compare("y") == 0){
      move_group.move();
    }
  }

  //  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  ros::shutdown();
  return 0;
}
