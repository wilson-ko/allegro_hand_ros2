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

#include <allegro_hand_grasp_controllers/grasp_action_effort_controller.hpp>

namespace grasp_action_controller {

AllegroHandGraspEffortController::AllegroHandGraspEffortController()
    : controller_interface::ControllerInterface(), action_monitor_period_(rclcpp::Duration::from_seconds(0)) {

  supported_command_map_ = {
      {"home", AllegroHandGrasp::MotionType::MOTION_TYPE_HOME},
      {"ready", AllegroHandGrasp::MotionType::MOTION_TYPE_READY},       // ready position
      {"grasp_3", AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3},   // grasp with 3 fingers
      {"grasp_4", AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4},   // grasp with 4 fingers
      {"pinch_it", AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT}, // pinch, index & thumb
      {"pinch_mt", AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT}, // pinch, middle & thumb
      {"envelop", AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP},   // envelop grasp (power-y)
      {"off", AllegroHandGrasp::MotionType::MOTION_TYPE_NONE},          // turn joints off
  };
}

void AllegroHandGraspEffortController::preempt_active_goal() {
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal) {
    active_goal->setCanceled(std::make_shared<CommandAction::Result>());
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

controller_interface::CallbackReturn AllegroHandGraspEffortController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

#if 0
controller_interface::return_type AllegroHandGraspEffortController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {

  std::vector<double> current_velocities;

  // double bhand_current_positions[JOINT_MAX];
  // double bhand_desired_torque[JOINT_MAX];

  for (const auto& joint_name : joint_names_) {
    double current_velocity = joint_velocity_state_interfaces_.at(joint_name).get().get_value();
    current_velocity = moving_average_filter_map_.at(joint_name)->filter(current_velocity);
    current_velocities.push_back(current_velocity);
  }

  // for (size_t i = 0; i < joint_names_.size(); i++) {
  //   const auto& joint_name = bhand_joint_sequence_.at(i);
  //   const double current_position = joint_position_state_interfaces_.at(joint_name).get().get_value();
  //   bhand_current_positions[i] = current_position;
  // }

  // bhand_v4_->SetTimeInterval(period.seconds());
  // bhand_v4_->SetJointPosition(bhand_current_positions);
  // bhand_v4_->UpdateControl(0);
  // bhand_v4_->GetJointTorque(bhand_desired_torque);

  // for (size_t i = 0; i < joint_names_.size(); i++) {
  //   const auto& joint_name = bhand_joint_sequence_.at(i);
  //   const auto& effort_command = bhand_desired_torque[i];
  //   joint_command_interfaces_.at(joint_name).get().set_value(effort_command);
  // }

  check_for_success(time, current_velocities);

  return controller_interface::return_type::OK;
}
#else
controller_interface::return_type AllegroHandGraspEffortController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  std::vector<double> filtered_current_velocities;
  std::map<std::string, std::pair<double, double>> joint_state;

  for (const auto& joint_name : joint_names_) {
    const double current_position = joint_position_state_interfaces_.at(joint_name).get().get_value();
    const double current_velocity = joint_velocity_state_interfaces_.at(joint_name).get().get_value();

    joint_state[joint_name] = std::make_pair(current_position, current_velocity);
    filtered_current_velocities.push_back(moving_average_filter_map_.at(joint_name)->filter(current_velocity));
  }

  ah_grasp_->set_joint_state(joint_state);
  ah_grasp_->update(period.seconds());

  std::map<std::string, double> joint_torque_form_ah_grasp;
  ah_grasp_->get_joint_torque(joint_torque_form_ah_grasp);

  for (const auto& [joint_name, torque] : joint_torque_form_ah_grasp) {
    joint_command_interfaces_.at(joint_name).get().set_value(torque);
  }

  check_for_success(time, filtered_current_velocities);

  return controller_interface::return_type::OK;
}
#endif

rclcpp_action::GoalResponse AllegroHandGraspEffortController::goal_callback(const rclcpp_action::GoalUUID&,
                                                                            std::shared_ptr<const CommandAction::Goal> goal) {
  (void)goal;
  // FIXME . check command name
  if (supported_command_map_.count(goal->command) == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Not supported command '%s'", goal->command.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Received & accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

#if 0
void AllegroHandGraspEffortController::accepted_callback(std::shared_ptr<GoalHandle> goal_handle) {
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

  preempt_active_goal();

  // bhand_v4_->SetMotionType(supported_command_map_.at(goal_handle->get_goal()->command));
  // if (goal_handle->get_goal()->effort > 0) {
  //   bhand_v4_->SetEnvelopTorqueScalar(goal_handle->get_goal()->effort);
  // }

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  last_movement_time_ = get_node()->now();
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  goal_handle_timer_.reset();
  goal_handle_timer_ = get_node()->create_wall_timer(action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                                     std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}
#else
void AllegroHandGraspEffortController::accepted_callback(std::shared_ptr<GoalHandle> goal_handle) {
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

  preempt_active_goal();

  // bhand_v4_->SetMotionType(supported_command_map_.at(goal_handle->get_goal()->command));
  // if (goal_handle->get_goal()->effort > 0) {
  //   bhand_v4_->SetEnvelopTorqueScalar(goal_handle->get_goal()->effort);
  // }

  ah_grasp_->set_motion_type(supported_command_map_.at(goal_handle->get_goal()->command));
  if (goal_handle->get_goal()->effort > 0) {
    ah_grasp_->set_envelop_torque(goal_handle->get_goal()->effort);
  }

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  last_movement_time_ = get_node()->now();
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  goal_handle_timer_.reset();
  goal_handle_timer_ = get_node()->create_wall_timer(action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                                     std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}
#endif

rclcpp_action::CancelResponse AllegroHandGraspEffortController::cancel_callback(const std::shared_ptr<GoalHandle> goal_handle) {
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    RCLCPP_INFO(get_node()->get_logger(), "Canceling active action goal because cancel callback received.");
    auto action_res = std::make_shared<CommandAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AllegroHandGraspEffortController::check_for_success(const rclcpp::Time& time, const std::vector<double>& current_velocities) {
  (void)time;
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (!active_goal) {
    return;
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

controller_interface::CallbackReturn AllegroHandGraspEffortController::on_configure(const rclcpp_lifecycle::State&) {
  spdlog::set_level(spdlog::level::trace);

  const auto logger = get_node()->get_logger();
  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  params_ = param_listener_->get_params();

  // Load joint names first, as they are needed for subsequent logic.
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

  const static std::set<std::string> supported_device = {
      "v4",
  };

  if (supported_device.count(params_.device) == 0) {
    RCLCPP_ERROR(logger, "Parameter 'device' has an unsupported value: '%s'. Supported values are: 'v4'.", params_.device.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

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

  /*
   *
   */
  AllegroHandGrasp::Options ah_grasp_options;
  ah_grasp_options.reference_frame_name = params_.grasp_reference_frame;

  try {
    ah_grasp_options.fingertip_link_names = get_node()->get_parameter("fingertips").as_string_array();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
    RCLCPP_ERROR(logger, "Parameter 'fingertips' is not declared. Please provide the list of joints to control.");
    return controller_interface::CallbackReturn::ERROR;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_ERROR(logger, "Parameter 'fingertips' has an invalid type. It must be a string array.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (ah_grasp_options.fingertip_link_names.empty()) {
    RCLCPP_ERROR(logger, "The 'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    // Get the "robot_description" string from the node's parameter server.
    ah_grasp_options.urdf_string = get_node()->get_parameter("robot_description").as_string();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException& ex) {
    RCLCPP_ERROR(logger, "Parameter 'robot_description' not found. Please load the URDF to the controller manager node.");
    return controller_interface::CallbackReturn::ERROR;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Failed to build Pinocchio model from 'robot_description': %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // lists the links in the following order: Thumb, Index, Middle, Ring....
  // If each link name is defined to follow the Finger Naming Rule ($Prefix$link_$finger_index$$joint_index$),
  // sorting them will naturally result in the order of Thumb, Index, Middle, Ring....
  std::sort(ah_grasp_options.fingertip_link_names.begin(), ah_grasp_options.fingertip_link_names.end());

  try {
    if (params_.device == "v4") {
      ah_grasp_options.device = AllegroHandGrasp::Options::HandDevice::HAND_V4;
      ah_grasp_ = AllegroHandGrasp::Create(ah_grasp_options);
    } else if (params_.device == "aph") {
      // FIXME
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Failed to create AllegroHandGrasp instance: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!ah_grasp_) {
    return controller_interface::CallbackReturn::ERROR;
  } else {
    auto info = ah_grasp_->get_info();
    RCLCPP_INFO(get_node()->get_logger(), "Create AllegroHandGrasp. %s hand configuration", info.hand_side ? "right" : "left");
  }

  for (size_t i = 0; i < joint_names_.size(); i++) {
    moving_average_filter_map_.emplace(joint_names_[i], std::make_unique<ExponentialMovingAverageFilter>(100));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AllegroHandGraspEffortController::on_activate(const rclcpp_lifecycle::State&) {
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint names are not set.");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();

  for (const auto& joint_name : joint_names_) {
    auto cmd_it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface& iface) {
          return iface.get_prefix_name() == joint_name && iface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
        });
    if (cmd_it == command_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Effort command interface not found for joint %s", joint_name.c_str());
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
  }

  pre_alloc_result_ = std::make_shared<CommandAction::Result>();
  pre_alloc_result_->state.name = joint_names_;
  pre_alloc_result_->state.position.clear();
  for (size_t i = 0; i < joint_names_.size(); i++) {
    pre_alloc_result_->state.position.push_back(joint_position_state_interfaces_.at(joint_names_[i]).get().get_value());
  }
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  action_server_ = rclcpp_action::create_server<CommandAction>(
      get_node(), "~/grasp_cmd",
      std::bind(&AllegroHandGraspEffortController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AllegroHandGraspEffortController::cancel_callback, this, std::placeholders::_1),
      std::bind(&AllegroHandGraspEffortController::accepted_callback, this, std::placeholders::_1));

  ah_grasp_->set_motion_type(AllegroHandGrasp::MotionType::MOTION_TYPE_NONE);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AllegroHandGraspEffortController::on_deactivate(const rclcpp_lifecycle::State&) {
  joint_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();

  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AllegroHandGraspEffortController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

controller_interface::InterfaceConfiguration AllegroHandGraspEffortController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

} // namespace grasp_action_controller
