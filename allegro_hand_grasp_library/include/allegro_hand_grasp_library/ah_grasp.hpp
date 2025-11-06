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

/**
 * @class AllegroHandGrasp
 * @brief An abstract base class defining the interface for Allegro Hand grasp algorithms.
 *
 * This class provides a standardized API for various grasp control strategies.
 * It encapsulates functionalities such as setting motion types, managing control gains,
 * and computing desired joint torques. Implementations of this class will provide
 * the specific logic for different hand models or control approaches.
 */
class AllegroHandGrasp {
public:
  /**
   * @brief Enumerates the available pre-defined motion and grasp types.
   */
  enum MotionType {
    MOTION_TYPE_NONE,             ///< Deactivates motors, allowing free motion.
    MOTION_TYPE_HOME,             ///< Commands the hand to a fixed, predefined home posture.
    MOTION_TYPE_READY,            ///< Moves the hand to a ready-to-grasp posture.
    MOTION_TYPE_GRAVITY_COMP,     ///< Activates gravity compensation to hold the current pose.
    MOTION_TYPE_PRE_SHAPE,        ///< A pre-shaping motion for grasping (legacy).
    MOTION_TYPE_GRASP_3,          ///< Executes a three-finger (thumb, index, middle) grasp.
    MOTION_TYPE_GRASP_4,          ///< Executes a four-finger grasp.
    MOTION_TYPE_PINCH_IT,         ///< Executes a pinch grasp with the index finger and thumb.
    MOTION_TYPE_PINCH_MT,         ///< Executes a pinch grasp with the middle finger and thumb.
    MOTION_TYPE_OBJECT_MOVING,    ///< Motion for moving a held object (legacy).
    MOTION_TYPE_ENVELOP,          ///< Executes a power grasp by wrapping fingers around an object.
    MOTION_TYPE_JOINT_PD,         ///< Simple joint-space PD control to a desired position.
    MOTION_TYPE_MOVE_OBJ,         ///< Legacy motion for moving an object.
    MOTION_TYPE_FINGERTIP_MOVING, ///< Legacy motion for moving fingertips to a target.
  };

protected:
  /**
   * @brief Protected default constructor to prevent direct instantiation.
   *
   * This class is abstract and should be instantiated via the static `Create` factory method.
   */
  AllegroHandGrasp() {}

public:
  /// A shared pointer to an AllegroHandGrasp instance.
  using Ptr = std::shared_ptr<AllegroHandGrasp>;
  /**
   * @brief Configuration options for creating a grasp algorithm instance.
   */
  struct Options {
    /**
     * @brief Specifies the target hand hardware version.
     */
    enum HandDevice {
      HAND_V4 = 0,       ///< Allegro Hand Version 4.
      HAND_V4_REF_DEBUG, ///< A reference implementation of V4 for debugging.
      HAND_PLEXUS,       ///< Allegro Hand Plexus.
    };
    /// The specific hand device model to use.
    HandDevice device;
    /// The URDF model of the robot as a string.
    std::string urdf_string;
    /// The name of the reference frame for kinematics (e.g., "palm_link").
    std::string reference_frame_name;
    /**
     * @brief A list of link names corresponding to the fingertips.
     * The order must be: Thumb, Index, Middle, Ring.
     */
    std::vector<std::string> fingertip_link_names;
  };

  struct Info {
    bool hand_side; // right := true
  };

  /**
   * @brief Virtual destructor.
   */
  virtual ~AllegroHandGrasp() {}
  /**
   * @brief A factory method to create a specific grasp algorithm instance.
   * @param options Configuration options specifying the device and model.
   * @return A shared pointer to the created AllegroHandGrasp instance.
   */
  static Ptr Create(const Options& options);

  virtual Info get_info() = 0;

  /**
   * @brief Sets the entire gain table from a YAML formatted string.
   * @param gain_yaml The YAML string containing the gain configuration for all motions.
   * @return True on success, false if parsing fails or the format is invalid.
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
   * @param gain_yaml The YAML string with the gain data for the specified motion.
   * @return True on success, false otherwise.
   */
  virtual bool set_gain_table_item(MotionType motion_type, std::string gain_yaml) = 0;

  /**
   * @brief Gets the gains for a specific motion type as a YAML string.
   * @param motion_type The motion type to retrieve.
   * @param[out] gain_yaml A string reference to be populated with the YAML data.
   * @return True on success, false otherwise.
   */
  virtual bool get_gain_table_item(MotionType motion_type, std::string& gain_yaml) = 0;

  /**
   * @brief Sets the active motion type for the grasp controller.
   * @param motion_type The motion to activate.
   * @return True if the motion type is supported and set, false otherwise.
   */
  virtual bool set_motion_type(MotionType motion_type) = 0;

  /**
   * @brief Sets a scalar for the enveloping grasp torque.
   * @param envelop_torque The scalar value to apply (e.g., 1.0 for default torque).
   */
  virtual void set_envelop_torque(double envelop_torque) = 0;

  /**
   * @brief Updates the internal state of the hand with new joint data.
   * @param joint_state A map from joint names to pairs of {position, velocity}.
   */
  virtual void set_joint_state(const std::map<std::string, std::pair<double, double>>& joint_state) = 0;

  /**
   * @brief Retrieves the computed desired joint torques.
   * @param[out] joint_torque A map from joint names to their computed torque values.
   */
  virtual void get_joint_torque(std::map<std::string, double>& joint_torque) = 0;

  /**
   * @brief Executes one step of the control loop.
   * @param dt The time elapsed since the last update, in seconds.
   */
  virtual void update(double dt) = 0;

  /**
   * @brief Retrieves the current positions of all fingertips.
   * @return A map from fingertip link names to their 3D position vectors.
   */
  virtual std::map<std::string, Eigen::Vector3d> get_fingertip_positions() = 0;

}; // class AllegroHandGrasp

/*
 *
 */
