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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <allegro_hand_grasp_library/ah_grasp.hpp>

/**
 * @brief Specifies the side of the hand, which is crucial for kinematic and
 * dynamic calculations that differ between the left and right hands.
 */
enum HandSide {
  LEFT = 0, ///< Represents the left hand.
  RIGHT,    ///< Represents the right hand.
};

/**
 * @brief A data structure that encapsulates all input/output information for a single finger.
 *
 * This context serves as a central point for accessing a finger's state (like
 * joint positions and velocities) and for setting its command (torque). It is
 * used by the grasp algorithm to process data for each finger independently.
 */
struct FingerIoContext {
  /// The kinematic chain of joint names for this finger, ordered from the base to the tip.
  std::vector<std::string> joint_chain;
  /**
   * @brief Holds the current state of the finger.
   */
  struct State {
    /// Fingertip position in the reference frame (typically the palm).
    Eigen::Vector3d fpos;
    /// Current joint angles (q).
    Eigen::VectorXd q;
    /// Current joint velocities (q_dot).
    Eigen::VectorXd q_dot;
    /// The Jacobian matrix mapping joint velocities to fingertip linear velocity.
    /// @note This Jacobian is expressed with respect to the reference frame.
    Eigen::MatrixXd jacobian;
    /**
     * @brief Constructs a State object, initializing all vectors and matrices to zero.
     * @param joint_num The number of joints in the finger.
     */
    explicit State(size_t joint_num) {
      fpos = Eigen::Vector3d::Zero(3);
      q = Eigen::VectorXd::Zero(joint_num);
      q_dot = Eigen::VectorXd::Zero(joint_num);
      jacobian = Eigen::MatrixXd::Zero(3, joint_num);
    }
  } state;

  /**
   * @brief Holds the command to be sent to the finger.
   */
  struct Command {
    /// The desired torque to be applied to each joint.
    /// @note The unit of torque is hardware-dependent and may not be in physical units like Nm.
    Eigen::VectorXd torque;
    /**
     * @brief Constructs a Command object, initializing the torque vector to zero.
     * @param joint_num The number of joints in the finger.
     */
    explicit Command(size_t joint_num) { torque = Eigen::VectorXd::Zero(joint_num); }
  } command;

  /**
   * @brief Constructs a FingerIoContext.
   * @param joints_ The kinematic chain of joint names for the finger.
   */
  explicit FingerIoContext(const std::vector<std::string>& joints_)
      : joint_chain(joints_), state(joint_chain.size()), command(joint_chain.size()) {}

  /**
   * @brief Gets the number of joints in the finger.
   * @return The number of joints.
   */
  inline size_t joint_num() const { return joint_chain.size(); }
};

/**
 * @brief Abstract base class for grasp control algorithms.
 *
 * This class defines the interface that all grasp algorithms must implement.
 * It provides a standardized way to set motions, update control loops, and
 * manage gains, regardless of the underlying control strategy.
 */
class GraspAlgorithm {
protected:
  /// A reference to the vector of I/O contexts for all fingers.
  std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts_;
  /// The side of the hand (LEFT or RIGHT).
  HandSide hand_side_;
  /// The currently active motion type.
  AllegroHandGrasp::MotionType motion_type_;

public:
  // Helper maps for YAML serialization/deserialization
  /// Helper map to convert a motion type string to its corresponding enum value.
  static const std::map<std::string, AllegroHandGrasp::MotionType> MOTION_TYPE_FROM_STRING;
  /// Helper map to convert a motion type enum to its corresponding string representation.
  static const std::map<AllegroHandGrasp::MotionType, std::string> MOTION_TYPE_TO_STRING;

  /// A smart pointer type for GraspAlgorithm objects.
  using Ptr = std::unique_ptr<GraspAlgorithm>;
  /**
   * @brief Constructs a GraspAlgorithm.
   * @param finger_io_contexts A reference to the vector of finger I/O contexts.
   * @param hand_side The side of the hand (LEFT or RIGHT).
   */
  explicit GraspAlgorithm(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts, HandSide hand_side)
      : finger_io_contexts_(finger_io_contexts), hand_side_(hand_side), motion_type_(AllegroHandGrasp::MotionType::MOTION_TYPE_NONE) {}
  /**
   * @brief Virtual destructor.
   */
  virtual ~GraspAlgorithm() {}

  /**
   * @brief Sets the entire gain table from a YAML formatted string.
   * @param gain_yaml The YAML string containing the gain configuration.
   * @return True on success, false otherwise.
   */
  virtual bool set_gain_table(std::string gain_yaml) = 0;
  /**
   * @brief Gets the entire gain table as a YAML formatted string.
   * @param[out] gain_yaml A string reference to be populated with the YAML data.
   * @return True on success, false otherwise.
   */
  virtual bool get_gain_table(std::string& gain_yaml) = 0;

  /**
   * @brief Sets the gains for a specific motion type from a YAML string.
   * @param motion_type The motion type to update.
   * @param gain_yaml The YAML string with the gain data.
   * @return True on success, false otherwise.
   */
  virtual bool set_gain_table_item(AllegroHandGrasp::MotionType motion_type, std::string gain_yaml) = 0;
  /**
   * @brief Gets the gains for a specific motion type as a YAML string.
   * @param motion_type The motion type to retrieve.
   * @param[out] gain_yaml A string reference to be populated with the YAML data.
   * @return True on success, false otherwise.
   */
  virtual bool get_gain_table_item(AllegroHandGrasp::MotionType motion_type, std::string& gain_yaml) = 0;

  /**
   * @brief Sets the active motion type for the grasp controller.
   * @param motion_type The motion to activate.
   * @return True if the motion type is supported and set, false otherwise.
   */
  virtual bool set_motion(AllegroHandGrasp::MotionType motion_type) = 0;
  /**
   * @brief Sets a scalar for the enveloping grasp torque.
   * @param envelop_torque The scalar value to apply.
   */
  virtual void set_envelop_torque(double envelop_torque) = 0;
  /**
   * @brief Updates the control algorithm for one time step.
   * @param dt The time delta since the last update, in seconds.
   */
  virtual void update(double dt) = 0;

  /**
   * @brief A factory method to create a specific grasp algorithm instance.
   * @param device The type of hand device, which determines the algorithm to create.
   * @param finger_io_contexts A reference to the vector of finger I/O contexts.
   * @param hand_side The side of the hand (LEFT or RIGHT).
   * @return A unique pointer to the created GraspAlgorithm instance.
   */
  static Ptr Create(AllegroHandGrasp::Options::HandDevice device, std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts,
                    HandSide hand_side);
};
