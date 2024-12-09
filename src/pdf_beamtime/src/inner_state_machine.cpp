/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/inner_state_machine.hpp>

using namespace std::chrono_literals;

InnerStateMachine::InnerStateMachine(
  const rclcpp::Node::SharedPtr node, const rclcpp::Node::SharedPtr gripper_node)
: node_(node), gripper_node_(gripper_node)
{
  internal_state_enum_ = Internal_State::RESTING;

  // Create gripper client
  gripper_client_ =
    gripper_node_->create_client<pdf_beamtime_interfaces::srv::GripperControlMsg>(
    "gripper_service");
}

moveit::core::MoveItErrorCode InnerStateMachine::move_robot(
  moveit::planning_interface::MoveGroupInterface & mgi, std::vector<double> joint_goal)
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;
  joint_goal_ = joint_goal;

  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {

        for (size_t i = 0; i < joint_goal_.size(); ++i) {
          std::cout << "Joint goal# Joint: " << i << ", Value: " <<
            joint_goal_[i] << std::endl;
        }

        // Get the current robot state
        auto current_state = mgi.getCurrentState(10); // Timeout in seconds
        if (!current_state) {
          RCLCPP_ERROR(rclcpp::get_logger("example"), "Failed to get current state");
          return;
        }

        // // Access the joint model group
        // const robot_state::JointModelGroup * joint_model_group =
        //   mgi.getRobotModel()->getJointModelGroup(mgi.getName());

// Modify the start state
        auto joint_positions = current_state->getVariablePositions();
        double wrist_3_position = joint_positions[5];
        std::cout << "Current wrist 3 from mgi current state: " << wrist_3_position << std::endl;

// // Normalize wrist_3_joint to your desired convention
//         if (wrist_3_position < 0) {
//           wrist_3_position = -M_PI; // Force -pi
//         } else {
//           wrist_3_position = M_PI; // Force pi
//         }
//         joint_positions[5] = wrist_3_position;
//         current_state->setVariablePositions(joint_positions);

// // Set the modified start state
//         move_group_interface.setStartState(*current_state);


        mgi.setStartStateToCurrentState();
        mgi.setJointValueTarget(joint_goal_);
        // Create a plan to that target pose
        auto const [planing_success, plan] = [&mgi] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto const ok = static_cast<bool>(mgi.plan(plan));
            return std::make_pair(ok, plan);
          }();

        const auto & trajectory = plan.trajectory_.joint_trajectory;
        const auto & last_point = trajectory.points.back();
        const auto & all_points = trajectory.points;
        // Get the joint names
        const auto & joint_names = trajectory.joint_names;
        // Get the joint values at the last point
        const auto & joint_values = last_point.positions;
        // // Print joint values
        // for (size_t i = 0; i < joint_names.size(); ++i) {
        //   std::cout << "Joint Planning# Joint: " << joint_names[i] << ", Value: " <<
        //     joint_values[i] << std::endl;
        // }

        for (size_t i = 0; i < all_points.size(); ++i) {
          std::cout << "Joint Planning# Joint: " << joint_names[5] << ", Value: " <<
            all_points[i].positions[5] << std::endl;
        }

        if (planing_success) {
          // Change inner state to Moving if the robot is ready to move and not on clean up
          if (internal_state_enum_ == Internal_State::RESTING) {
            set_internal_state(Internal_State::MOVING);
          }
          auto exec_results = mgi.execute(plan);
          return_error_code = exec_results;
        } else {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }

  return return_error_code;
}

moveit::core::MoveItErrorCode InnerStateMachine::move_robot_cartesian(
  moveit::planning_interface::MoveGroupInterface & mgi,
  std::vector<geometry_msgs::msg::Pose> target_pose)
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;

  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        mgi.setStartStateToCurrentState();
        // Create a plan to that target pose
        auto const [planing_success, plan] = [&mgi, target_pose] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            // path_achieved_fraction, between 0.0 and 1.0, indicating the fraction of the path
            // achieved as described by the waypoints. Return -1.0 in case of error.
            double path_achieved_fraction = mgi.computeCartesianPath(
              target_pose, 0.01, 0.0,
              plan.trajectory_);
            return std::make_pair(path_achieved_fraction, plan);
          }();
        const auto & trajectory = plan.trajectory_.joint_trajectory;
        const auto & last_point = trajectory.points.back();
        // Get the joint names
        const auto & joint_names = trajectory.joint_names;
        // Get the joint values at the last point
        const auto & joint_values = last_point.positions;
        // Print joint values
        for (size_t i = 0; i < joint_names.size(); ++i) {
          std::cout << "Cart Planning# Joint: " << joint_names[i] << ", Value: " <<
            joint_values[i] << std::endl;
        }

        if (1.0 - planing_success < 0.000001) {
          // Change inner state to Moving if the robot is ready to move and not on clean up
          if (internal_state_enum_ == Internal_State::RESTING) {
            set_internal_state(Internal_State::MOVING);
          }
          auto exec_results = mgi.execute(plan);
          return_error_code = exec_results;
        } else {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }

  return return_error_code;
}

moveit::core::MoveItErrorCode InnerStateMachine::close_gripper()
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;
  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        auto request = std::make_shared<pdf_beamtime_interfaces::srv::GripperControlMsg::Request>();
        request->command = "CLOSE";
        request->grip = 100;

        if (!gripper_client_->wait_for_service(10s)) {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
          break;
        } else {
          set_internal_state(Internal_State::MOVING);
          auto result = gripper_client_->async_send_request(request);
          if (rclcpp::spin_until_future_complete(gripper_node_, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper open: %d", result.get()->results);
            std::this_thread::sleep_for(3s);
            return_error_code = moveit::core::MoveItErrorCode::SUCCESS;
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return_error_code = moveit::core::MoveItErrorCode::FAILURE;
          }
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }
  return return_error_code;
}

moveit::core::MoveItErrorCode InnerStateMachine::open_gripper()
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;

  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        auto request = std::make_shared<pdf_beamtime_interfaces::srv::GripperControlMsg::Request>();
        request->command = "OPEN";
        request->grip = 100;

        if (!gripper_client_->wait_for_service(10s)) {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
          break;
        } else {
          set_internal_state(Internal_State::MOVING);
          auto result = gripper_client_->async_send_request(request);
          if (rclcpp::spin_until_future_complete(gripper_node_, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper open: %d", result.get()->results);
            std::this_thread::sleep_for(3s);
            return_error_code = moveit::core::MoveItErrorCode::SUCCESS;
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return_error_code = moveit::core::MoveItErrorCode::FAILURE;
          }
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }
  return return_error_code;
}

void InnerStateMachine::pause(moveit::planning_interface::MoveGroupInterface & mgi)
{
  switch (internal_state_enum_) {
    case Internal_State::RESTING:
      RCLCPP_INFO(
        node_->get_logger(), "Paused while at internal state %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      set_internal_state(Internal_State::PAUSED);
      break;

    case Internal_State::MOVING:
      mgi.stop();
      RCLCPP_INFO(
        node_->get_logger(), "Paused while at internal state %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      set_internal_state(Internal_State::PAUSED);
      break;
    default:
      break;
  }
}

void InnerStateMachine::abort(moveit::planning_interface::MoveGroupInterface & mgi)
{
  mgi.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Stopped while at internal state %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  set_internal_state(Internal_State::ABORT);
}

void InnerStateMachine::halt(moveit::planning_interface::MoveGroupInterface & mgi)
{
  mgi.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Halted while at internal state %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  set_internal_state(Internal_State::HALT);
}

void InnerStateMachine::rewind()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    set_internal_state(Internal_State::RESTING);
  }
}

void InnerStateMachine::set_internal_state(Internal_State state)
{
  RCLCPP_INFO(
    node_->get_logger(), "Internal state changed from %s to %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str(),
    internal_state_names[static_cast<int>(state)].c_str());
  internal_state_enum_ = state;
}

Internal_State InnerStateMachine::get_internal_state()
{
  return internal_state_enum_;
}
