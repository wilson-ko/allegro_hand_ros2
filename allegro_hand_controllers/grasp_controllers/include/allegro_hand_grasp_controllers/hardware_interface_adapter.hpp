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

#ifndef GRIPPER_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER_HPP_
#define GRIPPER_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER_HPP_

#include <algorithm>
#include <cassert>
#include <memory>
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/**
 * \brief Helper class to simplify integrating the AllegroHandGraspController with
 * different hardware interfaces.
 *
 * The AllegroHandGraspController outputs position while
 * it is supposed to work with either position or effort commands.
 *
 */
template <const char* HardwareInterface>
class HardwareInterfaceAdapter {
public:
  bool init(std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> /* joint_handle */,
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& /* node */) {
    return false;
  }

  void starting(const rclcpp::Time& /* time */) {}
  void stopping(const rclcpp::Time& /* time */) {}

  double updateCommand(double /* desired_position */, double /* desired_velocity */, double /* error_position */,
                       double /* error_velocity */, double /* max_allowed_effort */) {
    return 0.0;
  }
};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired
 * positions as commands.
 */
template <>
class HardwareInterfaceAdapter<hardware_interface::HW_IF_POSITION> {
public:
  bool init(std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_handle,
            const rclcpp_lifecycle::LifecycleNode::SharedPtr& /* node */) {
    joint_handle_ = joint_handle;
    return true;
  }

  void starting(const rclcpp::Time& /* time */) {}
  void stopping(const rclcpp::Time& /* time */) {}

  double updateCommand(double desired_position, double /* desired_velocity */, double /* error_position */, double /* error_velocity */,
                       double max_allowed_effort) {
    // Forward desired position to command
    joint_handle_->get().set_value(desired_position);
    return max_allowed_effort;
  }

private:
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_handle_;
};

/**
 * \brief Adapter for an effort-controlled hardware interface. Maps position and
 * velocity errors to effort commands through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this
 * adapter. Notice the \p gains entry: \code gripper_controller: type:
 * "grasp_action_controller/AllegroHandGraspController" joints: gripper_joint
 *   goal_tolerance: 0.01
 *   stalled_velocity_threshold: 0.01
 *   stall_timeout: 0.2
 *   gains:
 *     gripper_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 * \endcode
 */
template <>
class HardwareInterfaceAdapter<hardware_interface::HW_IF_EFFORT> {
public:
  template <typename ParameterT>
  auto auto_declare(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, const std::string& name,
                    const ParameterT& default_value) {
    if (!node->has_parameter(name)) {
      return node->declare_parameter<ParameterT>(name, default_value);
    } else {
      return node->get_parameter(name).get_value<ParameterT>();
    }
  }

  bool init(std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_handle,
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node) {
    joint_handle_ = joint_handle;
    // Init PID gains from ROS parameter server
    const std::string prefix = "gains." + joint_handle_->get().get_prefix_name();
    const auto k_p = auto_declare<double>(node, prefix + ".p", 0.0);
    const auto k_i = auto_declare<double>(node, prefix + ".i", 0.0);
    const auto k_d = auto_declare<double>(node, prefix + ".d", 0.0);
    const auto i_clamp = auto_declare<double>(node, prefix + ".i_clamp", 0.0);
    // Initialize PID
    pid_ = std::make_shared<control_toolbox::Pid>(k_p, k_i, k_d, i_clamp, -i_clamp);
    return true;
  }

  void starting(const rclcpp::Time& /* time */) {
    if (!joint_handle_) {
      return;
    }
    // Reset PIDs, zero effort commands
    pid_->reset();
    joint_handle_->get().set_value(0.0);
  }

  void stopping(const rclcpp::Time& /* time */) {}

  double updateCommand(double /* desired_position */, double /* desired_velocity */, double error_position, double error_velocity,
                       double max_allowed_effort) {
    // Preconditions
    if (!joint_handle_) {
      return 0.0;
    }
    // Time since the last call to update
    const auto period = std::chrono::steady_clock::now() - last_update_time_;
    // Update PIDs
    double command = pid_->computeCommand(error_position, error_velocity, static_cast<uint64_t>(period.count()));
    command = std::min<double>(fabs(max_allowed_effort), std::max<double>(-fabs(max_allowed_effort), command));
    joint_handle_->get().set_value(command);
    last_update_time_ = std::chrono::steady_clock::now();
    return command;
  }

private:
  using PidPtr = std::shared_ptr<control_toolbox::Pid>;
  PidPtr pid_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_handle_;
  std::chrono::steady_clock::time_point last_update_time_;
};

#endif // GRIPPER_CONTROLLERS__HARDWARE_INTERFACE_ADAPTER_HPP_
