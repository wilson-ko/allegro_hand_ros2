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

#pragma once

// C++ standard
#include <cassert>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_server_goal_handle.hpp>

#include <allegro_hand_grasp_controllers/grasp_action_controller_parameters.hpp>
#include <allegro_hand_grasp_controllers/visibility_control.hpp>

#include "ema_filter.hpp"

namespace grasp_action_controller {
/**
 * \brief Controller for executing a gripper command action for simple
 * multi-dof grippers.
 */
class AllegroHandGraspPositionController : public controller_interface::ControllerInterface {
public:
  /**
   * \brief Store position and max effort in struct to allow easier realtime
   * buffer usage
   */

  /// Maximum number of joints to support various Allegro Hand models.
  /// This is used to pre-allocate a fixed-size array for commands, which is
  /// essential for real-time safety by avoiding dynamic memory allocation in
  /// the control loop.
  static constexpr int JOINT_MAX = 32;

  struct Commands {
    double positions_[JOINT_MAX];
    double efforts_[JOINT_MAX];
  };

  GRASP_ACTION_CONTROLLER_PUBLIC AllegroHandGraspPositionController();

  GRASP_ACTION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GRASP_ACTION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GRASP_ACTION_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  GRASP_ACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  GRASP_ACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  GRASP_ACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  GRASP_ACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  realtime_tools::RealtimeBuffer<Commands> command_;
  // pre-allocated memory that is reused to set the realtime buffer
  Commands command_struct_, command_struct_rt_;

protected:
  using CommandAction = control_msgs::action::ParallelGripperCommand;
  using ActionServer = rclcpp_action::Server<CommandAction>;
  using ActionServerPtr = ActionServer::SharedPtr;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CommandAction>;

  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<CommandAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  bool update_hold_position_;
  bool verbose_ = false; ///< Hard coded verbose flag to help in debugging

  std::vector<std::string> joint_names_;
  std::map<std::string, int> joint_name_map_;

  std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interfaces_;
  std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interfaces_;
  std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interfaces_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  RealtimeGoalHandleBuffer rt_active_goal_;

  CommandAction::Result::SharedPtr pre_alloc_result_;
  rclcpp::Duration action_monitor_period_;

  ActionServerPtr action_server_;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;

  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CommandAction::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle> goal_handle);
  void accepted_callback(std::shared_ptr<GoalHandle> goal_handle);
  void preempt_active_goal();
  void set_hold_position();

  rclcpp::Time last_movement_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  std::map<std::string, double> computed_commands_;
  std::unordered_map<std::string, std::unique_ptr<ExponentialMovingAverageFilter>> moving_average_filter_map_;

  void check_for_success(const rclcpp::Time& time, const std::vector<double>& error_positions,
                         const std::vector<double>& current_velocities);
};

} // namespace grasp_action_controller
