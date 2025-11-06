/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Wonik Robotics.
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
 *   * Neither the name of Wonik Robotics nor the names of its
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
 */

/// \author ('c')void

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <allegro_hand_grasp_controllers/grasp_action_position_controller.hpp>

namespace grasp_action_controller {

void AllegroHandGraspPositionController::preempt_active_goal() {
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal) {
    active_goal->setCanceled(std::make_shared<CommandAction::Result>());
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

controller_interface::CallbackReturn AllegroHandGraspPositionController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AllegroHandGraspPositionController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  command_struct_rt_ = *(command_.readFromRT());

  std::vector<double> error_positions;
  std::vector<double> current_velocities;

  for (const auto& joint_name : joint_names_) {
    const double current_position = joint_position_state_interfaces_.at(joint_name).get().get_value();
    const double current_velocity = joint_velocity_state_interfaces_.at(joint_name).get().get_value();

    int joint_idx = joint_name_map_.at(joint_name);
    double command = command_struct_rt_.positions_[joint_idx];
    const double error_position = std::abs(command - current_position);

    error_positions.push_back(error_position);
    current_velocities.push_back(moving_average_filter_map_.at(joint_name)->filter(current_velocity));
    joint_command_interfaces_.at(joint_name).get().set_value(command);
  }

  check_for_success(time, error_positions, current_velocities);

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse AllegroHandGraspPositionController::goal_callback(const rclcpp_action::GoalUUID&,
                                                                              std::shared_ptr<const CommandAction::Goal> goal) {
  if (goal->command.name.size() != this->joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received goal with incorrect number of joints. Expected %zu, but got %zu. Rejecting goal.",
                 this->joint_names_.size(), goal->command.name.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->command.name.size() != goal->command.position.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Goal has mismatched joint name (%zu) and position (%zu) array sizes.",
                 goal->command.name.size(), goal->command.position.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  for (const auto& joint_name : goal->command.name) {
    if (joint_name_map_.find(joint_name) == joint_name_map_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Received goal with unknown joint name '%s'. Rejecting goal.", joint_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  for (const auto& command : goal->command.position) {
    if (std::isnan(command)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Received malformed goal: position is NaN.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "Received & accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void AllegroHandGraspPositionController::accepted_callback(std::shared_ptr<GoalHandle> goal_handle) {
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

  preempt_active_goal();

  for (size_t i = 0; i < goal_handle->get_goal()->command.name.size(); i++) {
    const auto joint_name = goal_handle->get_goal()->command.name[i];
    const auto joint_idx = joint_name_map_.at(joint_name);
    command_struct_.positions_[joint_idx] = goal_handle->get_goal()->command.position[i];
  }

  command_.writeFromNonRT(command_struct_);

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  last_movement_time_ = get_node()->now();
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  goal_handle_timer_.reset();

  goal_handle_timer_ = get_node()->create_wall_timer(action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                                     std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

rclcpp_action::CancelResponse AllegroHandGraspPositionController::cancel_callback(const std::shared_ptr<GoalHandle> goal_handle) {
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    set_hold_position();
    RCLCPP_INFO(get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    auto action_res = std::make_shared<CommandAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AllegroHandGraspPositionController::set_hold_position() {
  for (size_t i = 0; i < joint_names_.size(); i++) {
    command_struct_.positions_[i] = joint_position_state_interfaces_.at(joint_names_[i]).get().get_value();
    command_struct_.efforts_[i] = 0.0;
  }
  command_.writeFromNonRT(command_struct_);
}

void AllegroHandGraspPositionController::check_for_success(const rclcpp::Time& time, const std::vector<double>& error_positions,
                                                           const std::vector<double>& current_velocities) {
  (void)time;

  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (!active_goal) {
    return;
  }

  bool success = true;
  for (const auto& error : error_positions) {
    if (error > params_.goal_tolerance) {
      success = false;
      break;
    }
  }

  double max_velocity = 0.0;
  for (const auto& velocity : current_velocities) {
    if (max_velocity < std::abs(velocity)) {
      max_velocity = std::abs(velocity);
    }
  }

  auto fill_pre_alloc_result = [&]() {
    pre_alloc_result_->state.name = joint_names_;
    pre_alloc_result_->state.position.clear();
    for (size_t i = 0; i < joint_names_.size(); i++) {
      const auto& joint_name = joint_names_[i];
      const double current_position = joint_position_state_interfaces_.at(joint_name).get().get_value();
      pre_alloc_result_->state.position.push_back(current_position);
    }
  };

  if (success) {
    fill_pre_alloc_result();
    pre_alloc_result_->reached_goal = true;

    RCLCPP_INFO(get_node()->get_logger(), "Successfully moved to goal.");
    active_goal->setSucceeded(pre_alloc_result_);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  } else {
    if (max_velocity > params_.stall_velocity_threshold) {
      last_movement_time_ = time;
    } else if ((time - last_movement_time_).seconds() > params_.stall_timeout) {
      fill_pre_alloc_result();
      pre_alloc_result_->reached_goal = false;
      pre_alloc_result_->stalled = true;

      if (params_.allow_stalling) {
        RCLCPP_DEBUG(get_node()->get_logger(), "Stall detected moving to goal. Returning success.");
        active_goal->setSucceeded(pre_alloc_result_);
      } else {
        RCLCPP_DEBUG(get_node()->get_logger(), "Stall detected moving to goal. Aborting action!");
        active_goal->setAborted(pre_alloc_result_);
      }
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
  }
}

controller_interface::CallbackReturn AllegroHandGraspPositionController::on_configure(const rclcpp_lifecycle::State&) {
  spdlog::set_level(spdlog::level::trace);

  const auto logger = get_node()->get_logger();
  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);
  RCLCPP_INFO(logger, "Action status changes will be monitored at %f Hz.", params_.action_monitor_rate);

  try {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
    RCLCPP_ERROR(logger, "Parameter 'joints' is not declared. Please provide the list of joints to control.");
    return controller_interface::CallbackReturn::ERROR;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_ERROR(logger, "Parameter 'joints' has an invalid type. It must be a string array.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger, "The 'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (joint_names_.size() > JOINT_MAX) {
    RCLCPP_ERROR(logger, "The number of joints (%zu) exceeds the maximum allowed (%d).", joint_names_.size(), JOINT_MAX);
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_name_map_.clear();
  for (size_t i = 0; i < joint_names_.size(); i++) {
    joint_name_map_.emplace(joint_names_[i], i);
    moving_average_filter_map_.emplace(joint_names_[i], std::make_unique<ExponentialMovingAverageFilter>(100));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AllegroHandGraspPositionController::on_activate(const rclcpp_lifecycle::State&) {
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint names are not set.");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  computed_commands_.clear();

  for (const auto& joint_name : joint_names_) {
    auto cmd_it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface& iface) {
          return iface.get_prefix_name() == joint_name && iface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });
    if (cmd_it == command_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Command interface not found for joint %s", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_command_interfaces_.emplace(joint_name, *cmd_it);

    auto pos_it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(), [&joint_name](const hardware_interface::LoanedStateInterface& iface) {
          return iface.get_prefix_name() == joint_name && iface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });
    if (pos_it == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Position state interface not found for joint %s", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_position_state_interfaces_.emplace(joint_name, *pos_it);

    auto vel_it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(), [&joint_name](const hardware_interface::LoanedStateInterface& iface) {
          return iface.get_prefix_name() == joint_name && iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });
    if (vel_it == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Velocity state interface not found for joint %s", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_velocity_state_interfaces_.emplace(joint_name, *vel_it);

    computed_commands_.emplace(joint_name, 0.0);
  }

  for (size_t i = 0; i < joint_names_.size(); i++) {
    command_struct_.positions_[i] = joint_position_state_interfaces_.at(joint_names_[i]).get().get_value();
    command_struct_.efforts_[i] = 0.0;
  }
  command_.initRT(command_struct_);

  pre_alloc_result_ = std::make_shared<CommandAction::Result>();
  pre_alloc_result_->state.name = joint_names_;
  pre_alloc_result_->state.position.clear();
  for (size_t i = 0; i < joint_names_.size(); i++) {
    int joint_idx = joint_name_map_.at(joint_names_[i]);
    pre_alloc_result_->state.position.push_back(command_struct_.positions_[joint_idx]);
  }
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  action_server_ = rclcpp_action::create_server<CommandAction>(
      get_node(), "~/posture_cmd",
      std::bind(&AllegroHandGraspPositionController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AllegroHandGraspPositionController::cancel_callback, this, std::placeholders::_1),
      std::bind(&AllegroHandGraspPositionController::accepted_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AllegroHandGraspPositionController::on_deactivate(const rclcpp_lifecycle::State&) {
  joint_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  computed_commands_.clear();

  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AllegroHandGraspPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration AllegroHandGraspPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

AllegroHandGraspPositionController::AllegroHandGraspPositionController()
    : controller_interface::ControllerInterface(), action_monitor_period_(rclcpp::Duration::from_seconds(0)) {}

} // namespace grasp_action_controller
