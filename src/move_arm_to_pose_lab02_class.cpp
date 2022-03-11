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

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>

using namespace std;

static const std::string NODE_NAME = "lab02_move_arm_to_pose";
static const std::string PLANNING_GROUP = "manipulator";
static const std::string ROBOT_DESCRIPTION = "/summit_xl/robot_description";

class LabPlanner
{
protected:
  ros::NodeHandle *n;
  ros::Subscriber sub;
  moveit::planning_interface::MoveGroupInterface *move_group;
  // const robot_state::JointModelGroup *joint_model_group;
  void callback(const geometry_msgs::PoseStamped::ConstPtr &);
  void plan_and_move(geometry_msgs::Pose);

public:
  LabPlanner(ros::NodeHandle);
};

void LabPlanner::callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ROS_INFO_STREAM("Received pose: " << msg->pose);
  plan_and_move(msg->pose);
}

// Constructor using initialization list
LabPlanner::LabPlanner(ros::NodeHandle node_handle)
    : n(&node_handle),
      sub(n->subscribe("/summit_xl/lab02_goal_position", 1, &LabPlanner::callback, this))
//                                                       move_group(& moveit::planning_interface::MoveGroupInterface(CONFIG_OPTIONS, std::shared_ptr<tf2_ros::Buffer>(), ros::WallDuration(60))),
//                                                       joint_model_group(move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP))
{
  static const moveit::planning_interface::MoveGroupInterface::Options CONFIG_OPTIONS(PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle);
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  ros::WallDuration timeout_duration = ros::WallDuration(60);
  ros::spinOnce();
  moveit::planning_interface::MoveGroupInterface mg(CONFIG_OPTIONS, std::shared_ptr<tf2_ros::Buffer>(), timeout_duration);
  ros::spinOnce();
  this->move_group = &mg;
}

void LabPlanner::plan_and_move(geometry_msgs::Pose target_pose)
{
  ROS_INFO_STREAM("Planning to pose: " << target_pose);
  ROS_INFO_STREAM("Move group: " << move_group->getActiveJoints());
  const robot_state::JointModelGroup *joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  joint_model_group->printGroupInfo();
  

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  // //
  // // We can print the name of the reference frame for this robot.
  // ROS_INFO_NAMED(NODE_NAME, "Reference frame: %s", move_group->getPlanningFrame().c_str());

  // // We can also print the name of the end-effector link for this group.
  // ROS_INFO_NAMED(NODE_NAME, "End effector link: %s", move_group->getEndEffectorLink().c_str());

  // Set the goal pose
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // move_group->setPoseTarget(target_pose);
  // // Visualization
  // // ^^^^^^^^^^^^^
  // //
  // // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("summit_xl_base_footprint");
  // visual_tools.deleteAllMarkers();

  // // Remote control is an introspection tool that allows users to step through a high level script
  // // via buttons and keyboard shortcuts in RViz
  // visual_tools.loadRemoteControl();

  // // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // visual_tools.trigger();


  // //  // Now, we call the planner to compute the plan and visualize it.
  // //  // Note that we are just planning, not asking move_group
  // //  // to actually move the robot.
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // // Planning to a joint-space goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // // To start, we'll create an pointer that references the current robot's state.
  // // RobotState is the object that contains all the current position/velocity/acceleration data.
  // moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

  // // Planning with Path Constraints
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Path constraints can easily be specified for a link on the robot.
  // // Let's specify a path constraint and a pose goal for our group.
  // // First define the path constraint.

  // // Now, set it as the path constraint for the group.
  // moveit_msgs::Constraints constraints;
  // move_group->setPlanningTime(10.0);

  // std::vector<double> joint_group_positions;
  // std::vector<string> joint_names = joint_model_group->getActiveJointModelNames();
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // for (int i = 0; i < joint_group_positions.size(); i++)
  // {
  //   ROS_INFO_NAMED(NODE_NAME, "Current joint %s position: %f", joint_names[i].c_str(), joint_group_positions[i]);
  // }

  // // add some tolerance to make the planner's life easier
  // move_group->setGoalTolerance(0.005);

  // // since we plan to use this as a script, let's move slowly
  // move_group->setMaxVelocityScalingFactor(0.5);

  // // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  // // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "Move arm to pose", rvt::WHITE, rvt::XLARGE, false);

  // // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // visual_tools.trigger();

  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED(NODE_NAME, "Visualizing pose goal");
  // visual_tools.publishAxisLabeled(target_pose, "Pose Goal");
  // visual_tools.publishText(text_pose, "Visualizing Pose Goal", rvt::WHITE, rvt::XLARGE, false);
  // // visual_tools.trigger();
  // //  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan to the pose");

  // ROS_INFO_NAMED(NODE_NAME, "Invoking planner");

  // bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // ROS_INFO_NAMED(NODE_NAME, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
  // if (success)
  // {
  //   cout << "Please verify that the plan will not hit any objects uisng rviz.\n";
  //   cout << "Do you want to proceed with the planned movement? (y/N): ";
  //   string reply;
  //   getline(cin, reply);
  //   if (reply.compare("y") == 0)
  //   {
  //     move_group->execute(my_plan);
  //   }
  // }

  // //  // When done with the path constraint be sure to clear it.
  // move_group->clearPathConstraints();

  // // Since we set the start state we have to clear it before planning other paths
  // move_group->setStartStateToCurrentState();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ROS_INFO("Starting node");
  ros::NodeHandle node_handle;
  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  LabPlanner lp(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
