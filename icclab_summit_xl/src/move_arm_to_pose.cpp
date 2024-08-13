#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("move_arm_to_pose_goal", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  geometry_msgs::msg::Pose target_pose;
  std::string skip_safety_flag ("--skip-safety");
  bool skip_safety = false;
  
  if (argc >= 8)
  {
    target_pose.position.x = atof(argv[1]);
    target_pose.position.y = atof(argv[2]);
    target_pose.position.z = atof(argv[3]);
    target_pose.orientation.x = atof(argv[4]);
    target_pose.orientation.y = atof(argv[5]);
    target_pose.orientation.z = atof(argv[6]);
    target_pose.orientation.w = atof(argv[7]);
    if (argc >= 9 && skip_safety_flag.compare(argv[8]) == 0)
    {
      skip_safety = true;
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Usage: move_arm_to_pose_goal position.x position.y position.z orientation.x orientation.y orientation.z orientation.w");
    return -1;  
  }

  static const std::string ROBOT_DESCRIPTION = "robot_description";

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  auto options = moveit::planning_interface::MoveGroupInterface::Options(PLANNING_GROUP, ROBOT_DESCRIPTION, "/summit");
  moveit::planning_interface::MoveGroupInterface move_group(node, options);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  joint_model_group->printGroupInfo();

  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("summit_xl_base_footprint", rvt::RVIZ_MARKER_TOPIC, node);
  // visual_tools.deleteAllMarkers();
  // visual_tools.loadRemoteControl();
  // visual_tools.trigger();

  RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  move_group.setPlanningTime(20.0);
  
  std::vector<double> joint_group_positions;
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  for (int i = 0; i < joint_group_positions.size(); i++){
    RCLCPP_INFO(node->get_logger(), "Current joint %s position: %f", joint_names[i].c_str(), joint_group_positions[i]);
  }
  
  move_group.setGoalTolerance(0.005);
  move_group.setMaxVelocityScalingFactor(0.5);
  
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "Move arm to pose", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();
  
  // RCLCPP_INFO(node->get_logger(), "Visualizing pose goal");
  // visual_tools.publishAxisLabeled(target_pose, "Pose Goal");
  // visual_tools.publishText(text_pose, "Visualizing Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();
  
  // if (!skip_safety){
  //   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan to the pose");
  // }
   
  RCLCPP_INFO(node->get_logger(), "Invoking planner");

  auto const [success, plan] = [&move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  RCLCPP_INFO(node->get_logger(), "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
  
  if (success){
    if (!skip_safety){
      cout << "Please verify that the plan will not hit any objects using rviz.\n";
      cout << "Do you want to proceed with the planned movement? (y/N): ";
      string reply;
      getline (cin, reply);
      if (reply.compare("y") == 0){
        move_group.execute(plan);
      }
    } else {
        move_group.execute(plan);
    }
  }

  move_group.clearPathConstraints();
  move_group.setStartStateToCurrentState();

  rclcpp::shutdown();
  return 0;
}