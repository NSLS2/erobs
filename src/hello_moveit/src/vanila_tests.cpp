/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
/*
This needs robot_description and robot_description_semantic parameters somehow.
*/

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create a node for synchronously grabbing params
  auto parameter_client_node = rclcpp::Node::make_shared("param_client");
  auto parent_parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_client_node, "move_group");
  // Boiler plate wait block
  while (!parent_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "move_group service not available, waiting again...");
  }

  // Get robot config parameters from parameter server
  auto parameters = parent_parameters_client->get_parameters(
    {"robot_description_semantic",
      "robot_description"});

  // create the Node for moveit with
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  std::string parameter_value = parameters[0].value_to_string();
  node->declare_parameter<std::string>("robot_description_semantic", parameter_value);
  parameter_value = parameters[1].value_to_string();
  node->declare_parameter<std::string>("robot_description", parameter_value);

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  RCLCPP_INFO(logger, "assembling move_group_interface");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");


  for (int i = 0; i < 30; i++) {

    RCLCPP_INFO(logger, "********* Try : %d ********", i);

    std::vector target_pose = {4.089481, -0.987856, 2.167873, -1.174083, 0.899019, 3.141593};
    move_group_interface.setJointValueTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();

    // Execute the plan
    if (success) {
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }

    // geometry_msgs::msg::Pose target_pose_msg = move_group_interface.getCurrentPose().pose;
    // // Move 2 cm down in the z direction
    // target_pose_msg.position.y += -0.05;
    // // target_pose_msg.position.z += -0.1;
    // std::vector<geometry_msgs::msg::Pose> target_pose_vector;
    // target_pose_vector.push_back(target_pose_msg);

    // auto const [planing_success, plan_cart] = [&move_group_interface, target_pose_vector] {
    //     moveit::planning_interface::MoveGroupInterface::Plan plan_cart;
    //     // path_achieved_fraction, between 0.0 and 1.0, indicating the fraction of the path
    //     // achieved as described by the waypoints. Return -1.0 in case of error.
    //     double path_achieved_fraction = move_group_interface.computeCartesianPath(
    //       target_pose_vector, 0.01, 0.0,
    //       plan_cart.trajectory_);
    //     return std::make_pair(path_achieved_fraction, plan_cart);
    //   }();

    // // Execute the plan
    // if (planing_success > 0.95) {
    //   move_group_interface.execute(plan_cart);
    // } else {
    //   RCLCPP_ERROR(logger, "Planning failed!");
    // }


    std::vector target_pose_2 = {-0.11536, -1.783732, 0.38816, -1.75492, 0.11484, 3.14159};
    move_group_interface.setJointValueTarget(target_pose_2);

    // Create a plan to that target pose
    auto const [success_2, plan_2] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();

    // Execute the plan
    if (success_2) {
      move_group_interface.execute(plan_2);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }

    std::this_thread::sleep_for(3s);
    /* code */
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
