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

/*
 * FIXME. 2025.09.03 ('c')void
 * The Allegro Hand Plexus has a kinematic structure very similar to the V4,
 * so this is designed with a structure similar to GraspAlgorithmV4Legacy.
 * Note: This file has not yet been modified for the Plexus and will not operate correctly.
 */

#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "ah_grasp_interface.hpp"

/**
 * @brief Defines the unique identifiers for each finger.
 * @note The order is intentionally set to match the legacy BHand library's
 *       internal indexing for easier comparison and debugging.
 */
enum FingerID {
  THUMB = 0, ///< Identifier for the Thumb.
  INDEX,     ///< Identifier for the Index finger.
  MIDDLE,    ///< Identifier for the Middle finger.
  RING,      ///< Identifier for the Ring finger.
  NOF,       ///< Total number of fingers.
};

static const std::map<FingerID, std::string> finger_to_string = {
    {THUMB, "thumb"},
    {INDEX, "index"},
    {MIDDLE, "middle"},
    {RING, "ring"},
};

/// The number of joints per finger.
static constexpr int NOJ = 4;
/// Conversion factor from degrees to radians.
static constexpr double DEG2RAD = M_PI / 180;
/// Conversion ratio from PWM command values to torque.
static constexpr double PWM_TO_TORQUE_RATIO = 800.0;

/// Default home position joint angles (in radians) for the left hand.
static constexpr double Q_HOME_LEFT[NOF][NOJ] = {{50 * DEG2RAD, 25 * DEG2RAD, 15 * DEG2RAD, 45 * DEG2RAD},  // THUMB
                                                 {0 * DEG2RAD, -10 * DEG2RAD, 45 * DEG2RAD, 45 * DEG2RAD},  // INDEX
                                                 {0 * DEG2RAD, -10 * DEG2RAD, 45 * DEG2RAD, 45 * DEG2RAD},  // MIDDLE
                                                 {-5 * DEG2RAD, -5 * DEG2RAD, 50 * DEG2RAD, 45 * DEG2RAD}}; // RING

/// Default home position joint angles (in radians) for the right hand.
static constexpr double Q_HOME_RIGHT[NOF][NOJ] = {{50 * DEG2RAD, 25 * DEG2RAD, 15 * DEG2RAD, 45 * DEG2RAD}, // THUMB
                                                  {0 * DEG2RAD, -10 * DEG2RAD, 45 * DEG2RAD, 45 * DEG2RAD}, // INDEX
                                                  {0 * DEG2RAD, -10 * DEG2RAD, 45 * DEG2RAD, 45 * DEG2RAD}, // MIDDLE
                                                  {5 * DEG2RAD, -5 * DEG2RAD, 50 * DEG2RAD, 45 * DEG2RAD}}; // RING

/// Default Coulomb friction coefficients.
static constexpr double FC[NOF][NOJ] = {
    // THUMB
    {0.0f / 800.0f * 0.5f, 30.0f / 800.0f * 0.5f, 30.0f / 800.0f * 0.5f, 30.0f / 800.0f * 0.5f},
    // INDEX, MIDDLE, RING
    {2.0f / 800.0f * 0.5f, 30.0f / 800.0f * 0.5f, 20.0f / 800.0f * 0.5f, 10.0f / 800.0f * 0.5f},
    {2.0f / 800.0f * 0.5f, 30.0f / 800.0f * 0.5f, 20.0f / 800.0f * 0.5f, 10.0f / 800.0f * 0.5f},
    {2.0f / 800.0f * 0.5f, 30.0f / 800.0f * 0.5f, 20.0f / 800.0f * 0.5f, 10.0f / 800.0f * 0.5f}};

/// Default viscous friction coefficients.
static constexpr double FV[NOF][NOJ] = {
    // THUMB
    {2.0f / 800.0f * 1.0f, 1.0f / 800.0f * 1.0f, 2.0f / 800.0f * 1.0f, 2.0f / 800.0f * 1.0f},
    // INDEX, MIDDLE, RING
    {2.0f / 800.0f * 1.0f, 3.0f / 800.0f * 1.0f, 3.0f / 800.0f * 1.0f, 0.5f / 800.0f * 1.0f},
    {2.0f / 800.0f * 1.0f, 3.0f / 800.0f * 1.0f, 3.0f / 800.0f * 1.0f, 0.5f / 800.0f * 1.0f},
    {2.0f / 800.0f * 1.0f, 3.0f / 800.0f * 1.0f, 3.0f / 800.0f * 1.0f, 0.5f / 800.0f * 1.0f}};

/// Multipliers for the tanh function used in friction compensation.
static constexpr double TANH_MULTIPLIERS[NOJ] = {50.0, 40.0, 50.0, 10.0};

/**
 * @brief A structure to hold a complete set of control gains for a finger.
 *
 * This structure groups all gain-related parameters, making it easy to manage
 * and switch between different control behaviors.
 */
struct GainSet {
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd kp_task;
  Eigen::VectorXd kd_task;
  double f_des; // desired force

  /// @brief Default constructor for std::map compatibility.
  GainSet() : f_des(0.0) {}

  /**
   * @brief Constructs a GainSet, initializing vectors to a specified size.
   */
  explicit GainSet(size_t joint_num) {
    kp = Eigen::VectorXd::Zero(joint_num);
    kd = Eigen::VectorXd::Zero(joint_num);
    kp_task = Eigen::VectorXd::Zero(joint_num);
    kd_task = Eigen::VectorXd::Zero(joint_num);
    f_des = 0.0;
  }

  /// @brief Copy constructor.
  GainSet(const GainSet& other) : kp(other.kp), kd(other.kd), kp_task(other.kp_task), kd_task(other.kd_task), f_des(other.f_des) {}

  /// @brief Copy assignment operator to resolve -Wdeprecated-copy warning.
  GainSet& operator=(const GainSet& other) {
    if (this != &other) {
      kp = other.kp;
      kd = other.kd;
      kp_task = other.kp_task;
      kd_task = other.kd_task;
      f_des = other.f_des;
    }
    return *this;
  }

  /**
   * @brief Resets all gain values to zero.
   */
  inline void reset_gains() {
    kp.setZero();
    kd.setZero();
    kp_task.setZero();
    kd_task.setZero();
    f_des = 0;
  }

  /**
   * @brief Sets the proportional gains from a vector.
   */
  inline void set_kp(const std::vector<double>& values) {
    assert(kp.size() == (int)values.size());
    memcpy(kp.data(), values.data(), sizeof(double) * values.size());
  }

  /**
   * @brief Sets the derivative gains from a vector.
   */
  inline void set_kd(const std::vector<double>& values) {
    assert(kd.size() == (int)values.size());
    memcpy(kd.data(), values.data(), sizeof(double) * values.size());
  }

  /**
   * @brief Sets the task-space proportional gains from a vector.
   */
  inline void set_kp_task(const std::vector<double>& values) {
    assert(kp_task.size() == (int)values.size());
    memcpy(kp_task.data(), values.data(), sizeof(double) * values.size());
  }

  /**
   * @brief Sets the task-space derivative gains from a vector.
   */
  inline void set_kd_task(const std::vector<double>& values) {
    assert(kd_task.size() == (int)values.size());
    memcpy(kd_task.data(), values.data(), sizeof(double) * values.size());
  }
};

/**
 * @brief Holds all control-related data for a single finger.
 *
 * This structure encapsulates the I/O context, constants, current gains,
 * desired values, and outputs for one finger, serving as the primary data
 * object for per-finger control calculations.
 */
struct ControlContext {
  /// Pointer to the I/O context for this finger, providing state information.
  FingerIoContext* io_ctx;
  /// Home position for the finger's joints.
  Eigen::VectorXd home_position;
  /// Coulomb friction coefficients for the finger's joints.
  Eigen::VectorXd fc;
  /// Viscous friction coefficients for the finger's joints.
  Eigen::VectorXd fv;

  /// The currently active set of control gains.
  GainSet gain;

  /// A map storing pre-configured gain sets for different motion types.
  std::map<AllegroHandGrasp::MotionType, GainSet> gain_table;

  /// Desired fingertip position in the palm frame.
  Eigen::Vector3d fp_des;

  /// The final computed desired torque to be sent to the joints.
  /// @note The Allegro Hand V4 SDK uses a normalized torque value in the range [-1.0, 1.0].
  Eigen::VectorXd tau_des;

  /// @brief Resets the current gains to zero.
  inline void reset_gains() { gain.reset_gains(); }

  /// @brief Adds or updates a gain set in the gain table for a specific motion type.
  inline void set_gain_table_item(AllegroHandGrasp::MotionType motion_type, const GainSet& gain_set) { gain_table[motion_type] = gain_set; }

  /// @brief Resets all desired values (position, torque) to zero.
  inline void reset_desired_values() {
    fp_des.setZero();
    tau_des.setZero();
  }

  /// @brief Switches the active gain set to the one specified by the motion type.
  inline void switch_gain(AllegroHandGrasp::MotionType motion_type) {
    auto it = this->gain_table.find(motion_type);
    if (it != this->gain_table.end()) {
      this->gain = it->second;
    } else {
      std::string str_motion_type;
      try {
        str_motion_type = GraspAlgorithm::MOTION_TYPE_TO_STRING.at(motion_type);
      } catch (const std::out_of_range& e) {
        str_motion_type = fmt::format("UNKNOWN_MOTION_TYPE({})", int(motion_type));
      }
      throw std::runtime_error(fmt::format("Gains for motion type {} not found.", str_motion_type));
    }
  }

  /**
   * @brief Calculates the desired torque for a single joint using PD control.
   * @param j_idx The index of the joint.
   * @param q_des_j The desired position for that joint.
   */
  void calc_tau_des_pd_control(int j_idx, double q_des_j) {
    const auto& state = io_ctx->state;
    tau_des(j_idx) = gain.kp(j_idx) * (q_des_j - state.q(j_idx)) - gain.kd(j_idx) * state.q_dot(j_idx);
    tau_des(j_idx) /= PWM_TO_TORQUE_RATIO;
  }

  /**
   * @brief Calculates the desired torque for all joints based on task-space control.
   *
   * This function implements the core control law, combining position control,
   * force control, damping, and friction compensation to compute the final torque.
   * @param e_object_vec The error vector from the object's current position to its desired position.
   */
  void calc_tau_des(const Eigen::Vector3d& e_object_vec) {
    const auto& state = io_ctx->state;
    const auto& q_dot = state.q_dot;
    Eigen::MatrixXd JT = state.jacobian.transpose();

    // Fingertip position error
    Eigen::Vector3d e_vec = fp_des - state.fpos;

    // Jacobian transpose multiplied by error vector
    Eigen::VectorXd JTe = JT * e_vec;

    // Position-based torque
    Eigen::VectorXd tau_pos = gain.kp.array() * JTe.array();

    // Pinching force torque
    Eigen::VectorXd tau_pinching = gain.f_des * JTe;

    // Object position correction torque
    Eigen::VectorXd tau_pos_obj = gain.kp_task.array() * (JT * e_object_vec).array();

    // Joint damping torque
    Eigen::VectorXd tau_joint_damping = gain.kd.array() * q_dot.array();

    // Friction compensation torque
    Eigen::VectorXd tau_friction = Eigen::VectorXd::Zero(NOJ);
    for (int j_idx = 0; j_idx < NOJ; ++j_idx) {
      tau_friction(j_idx) = (fv(j_idx) * q_dot(j_idx)) + (fc(j_idx) * std::tanh(q_dot(j_idx) * TANH_MULTIPLIERS[j_idx]));
    }
    tau_des = tau_pos + tau_pos_obj - tau_joint_damping + tau_pinching + tau_friction;
  }

  /**
   * @brief Constructs a ControlContext for a finger.
   * @param io_ctx_ A pointer to the finger's I/O context.
   */
  explicit ControlContext(FingerIoContext* io_ctx_) : io_ctx(io_ctx_), gain(io_ctx->joint_num()) {
    auto joint_num = io_ctx->joint_num();

    home_position = Eigen::VectorXd::Zero(joint_num);
    fc = Eigen::VectorXd::Zero(joint_num);
    fv = Eigen::VectorXd::Zero(joint_num);

    fp_des.setZero();

    tau_des = Eigen::VectorXd::Zero(joint_num);
  }
}; // ControlContext

/**
 * @class GraspAlgorithmPlexusLegacy
 * @brief A refactored implementation of the legacy Allegro Hand grasp algorithm.
 *
 * This class modernizes the structure of the original BHand library code while
 * preserving its core control logic. It manages the control state for each
 * finger and computes the desired torques based on the selected motion type.
 * It supports dynamic gain loading via YAML.
 */
class GraspAlgorithmPlexusLegacy : public GraspAlgorithm {
  /// Vector of control contexts, one for each finger.
  std::vector<ControlContext> control_context_;

  /// The elapsed time since the current motion was started.
  double current_time_;
  /// A scalar value applied to the torque during an enveloping grasp.
  double _envelop_torque_scalar;

  /// @copydoc GraspAlgorithm::set_gain_table
  bool set_gain_table(std::string gain_yaml) override;
  /// @copydoc GraspAlgorithm::get_gain_table
  bool get_gain_table(std::string& gain_yaml) override;

  /// @copydoc GraspAlgorithm::set_gain_table_item
  bool set_gain_table_item(AllegroHandGrasp::MotionType motion_type, std::string gain_yaml) override;
  /// @copydoc GraspAlgorithm::get_gain_table_item
  bool get_gain_table_item(AllegroHandGrasp::MotionType motion_type, std::string& gain_yaml) override;

  /// @copydoc GraspAlgorithm::set_envelop_torque
  void set_envelop_torque(double envelop_torque) override { _envelop_torque_scalar = envelop_torque; }

  /// @copydoc GraspAlgorithm::update
  void update(double dt) override;
  /// @copydoc GraspAlgorithm::set_motion
  bool set_motion(AllegroHandGrasp::MotionType motion_type) override;

  /// @brief Initializes the gain table with default values for all motion types.
  void _initialize_default_gain_table();
  /// @brief Main torque calculation routine, called in each update cycle.
  void _update_torque();

  // Private methods for specific motion control logic.
  void _motion_home();
  void _motion_ready();
  void _motion_grasp3();
  void _motion_grasp4();
  void _motion_pinch_it();
  void _motion_pinch_mt();
  void _motion_envelop();

public:
  /**
   * @brief Constructs the GraspAlgorithmPlexusLegacy object.
   * @param finger_io_contexts A reference to the vector of finger I/O contexts.
   * @param hand_side The side of the hand (LEFT or RIGHT).
   */
  explicit GraspAlgorithmPlexusLegacy(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts, HandSide hand_side);
  /// @brief Virtual destructor.
  virtual ~GraspAlgorithmPlexusLegacy() = default;
}; // class GraspAlgorithmPlexusLegacy

/**
 * @brief Factory function to create an instance of GraspAlgorithmPlexusLegacy.
 * @param finger_io_contexts A reference to the vector of finger I/O contexts.
 * @param hand_side The side of the hand (LEFT or RIGHT).
 * @return A unique pointer to the created GraspAlgorithmPlexusLegacy instance.
 */
GraspAlgorithm::Ptr CreateGraspAlgorithmPlexusLegacy(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts,
                                                     HandSide hand_side) {
  return std::make_unique<GraspAlgorithmPlexusLegacy>(finger_io_contexts, hand_side);
}

GraspAlgorithmPlexusLegacy::GraspAlgorithmPlexusLegacy(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts,
                                                       HandSide hand_side)
    : GraspAlgorithm(finger_io_contexts, hand_side) {
  current_time_ = 0;
  _envelop_torque_scalar = 1.0;

  for (size_t idx = 0; idx < finger_io_contexts_.size(); idx++) {
    auto& fc = finger_io_contexts_.at(idx);
    control_context_.emplace_back(fc.get());

    const double* home = (hand_side == HandSide::LEFT) ? Q_HOME_LEFT[idx] : Q_HOME_RIGHT[idx];
    control_context_.back().home_position = Eigen::Map<const Eigen::VectorXd>(home, NOJ);
    control_context_.back().fc = Eigen::Map<const Eigen::VectorXd>(FC[idx], NOJ);
    control_context_.back().fv = Eigen::Map<const Eigen::VectorXd>(FV[idx], NOJ);
  }

  _initialize_default_gain_table();
}

bool GraspAlgorithmPlexusLegacy::set_motion(AllegroHandGrasp::MotionType motion_type) {
  current_time_ = 0;

  // Check Supported Motion Types
  switch (motion_type) {
  case AllegroHandGrasp::MotionType::MOTION_TYPE_NONE:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_HOME:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_READY:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP:
    break;
  default:
    return false;
  }

  motion_type_ = motion_type;

  // Update Gain
  for (auto& cc : control_context_) {
    cc.switch_gain(motion_type_);
  }

  return true;
}

void GraspAlgorithmPlexusLegacy::update(double dt) {
  current_time_ += dt;
  _update_torque();

  // Set Output Torque
  for (size_t i = 0; i < finger_io_contexts_.size(); i++) {
    auto& finger_context = finger_io_contexts_[i];
    const auto& control = control_context_[i];
    finger_context->command.torque = control.tau_des;
  }
}

/**
 * @brief Main torque calculation routine.
 *
 * This function orchestrates the torque calculation by calling the appropriate motion-specific function.
 */
void GraspAlgorithmPlexusLegacy::_update_torque() {
  bool torque_set = false;

  for (auto& cc : control_context_) {
    cc.reset_desired_values();
  }

  switch (motion_type_) {
  case AllegroHandGrasp::MotionType::MOTION_TYPE_NONE:
    torque_set = true;
    break;
  case AllegroHandGrasp::MotionType::MOTION_TYPE_HOME:
    _motion_home();
    torque_set = true;
    break;
  case AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP:
    _motion_envelop();
    torque_set = true;
    break;
  default:
    break;
  }

  if (torque_set) {
    return;
  }

  switch (motion_type_) {
  case AllegroHandGrasp::MotionType::MOTION_TYPE_READY:
    _motion_ready();
    break;
  case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3:
    _motion_grasp3();
    break;
  case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4:
    _motion_grasp4();
    break;
  case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT:
    _motion_pinch_it();
    break;
  case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT:
    _motion_pinch_mt();
    break;
  default:
    return;
  }

  // Object-level control variables
  Eigen::Vector3d xc_object =
      (finger_io_contexts_[INDEX]->state.fpos + finger_io_contexts_[MIDDLE]->state.fpos + finger_io_contexts_[THUMB]->state.fpos) / 3.0;
  Eigen::Vector3d xd_object = Eigen::Vector3d::Zero(); // Desired object position
  Eigen::Vector3d e_object_vec = xd_object - xc_object;

  for (int f_idx = 0; f_idx < NOF; ++f_idx) {
    control_context_[f_idx].calc_tau_des(e_object_vec);
  }
}

/**
 * @brief Computes torque to move the hand to its home position using PD control.
 */
void GraspAlgorithmPlexusLegacy::_motion_home() {
  // f_idx: finger index (0:THUMB, 1:INDEX, 2:MIDDLE, 3:RING)
  for (int f_idx = 0; f_idx < NOF; ++f_idx) {
    auto& control = control_context_[f_idx];
    const auto& state = finger_io_contexts_[f_idx]->state;

    control.tau_des = control.gain.kp.array() * (control.home_position - state.q).array() - control.gain.kd.array() * state.q_dot.array();
    control.tau_des /= PWM_TO_TORQUE_RATIO;
  }
}

/**
 * @brief Sets the desired fingertip positions for a "ready" or pre-grasp posture.
 */
void GraspAlgorithmPlexusLegacy::_motion_ready() {
  if (hand_side_ == LEFT) {
    control_context_[INDEX].fp_des = {0.08, -0.048, 0.10};
    control_context_[MIDDLE].fp_des = {0.08, 0.0, 0.10};
    control_context_[RING].fp_des = {0.08, 0.045, 0.08};
    control_context_[THUMB].fp_des = {0.11, -0.040, -0.04};
  } else { // RIGHT
    control_context_[INDEX].fp_des = {0.08, 0.048, 0.10};
    control_context_[MIDDLE].fp_des = {0.08, 0.0, 0.10};
    control_context_[RING].fp_des = {0.08, -0.045, 0.08};
    control_context_[THUMB].fp_des = {0.11, 0.040, -0.04};
  }
}

/**
 * @brief Implements a three-finger grasp (thumb, index, middle).
 *
 * The index and middle fingers move towards the thumb, and the thumb moves
 * towards the geometric center of the other two fingertips.
 */
void GraspAlgorithmPlexusLegacy::_motion_grasp3() {
  const auto& p_index = finger_io_contexts_[INDEX]->state.fpos;
  const auto& p_middle = finger_io_contexts_[MIDDLE]->state.fpos;
  const auto& p_thumb = finger_io_contexts_[THUMB]->state.fpos;

  Eigen::Vector3d center_geo = (p_index + p_middle) / 2.0;

  control_context_[INDEX].fp_des = p_thumb;
  control_context_[MIDDLE].fp_des = p_thumb;
  control_context_[THUMB].fp_des = center_geo;

  // Direction vectors
  Eigen::Vector3d dir_index = (control_context_[INDEX].fp_des - p_index).normalized();
  Eigen::Vector3d dir_middle = (control_context_[MIDDLE].fp_des - p_middle).normalized();
  Eigen::Vector3d dir_thumb = (control_context_[THUMB].fp_des - p_thumb).normalized();

  // Update desired positions to be one unit vector away
  control_context_[INDEX].fp_des = p_index + dir_index;
  control_context_[MIDDLE].fp_des = p_middle + dir_middle;
  control_context_[THUMB].fp_des = p_thumb + dir_thumb;

  // Force balancing (simplified from legacy)
  double alpha0 = control_context_[INDEX].gain.f_des;
  double alpha1 = (-1) * alpha0 * (dir_index.x() - dir_thumb.x() * dir_index.y() / dir_thumb.y()) /
                  (dir_middle.x() - dir_thumb.x() * dir_middle.y() / dir_thumb.y());
  double alpha3 = (-1) * alpha0 * dir_index.z() - alpha1 * dir_middle.z() + 0.003 * 9.81;

  control_context_[MIDDLE].gain.f_des = alpha1;
  control_context_[THUMB].gain.f_des = alpha3;
}

/**
 * @brief Implements a four-finger grasp (all fingers).
 *
 * The three fingers (index, middle, ring) move towards the thumb, and the thumb
 * moves towards the geometric center of the other three fingertips.
 */
void GraspAlgorithmPlexusLegacy::_motion_grasp4() {
  const auto& p_index = finger_io_contexts_[INDEX]->state.fpos;
  const auto& p_middle = finger_io_contexts_[MIDDLE]->state.fpos;
  const auto& p_ring = finger_io_contexts_[RING]->state.fpos;
  const auto& p_thumb = finger_io_contexts_[THUMB]->state.fpos;

  Eigen::Vector3d center_geo = (p_index + p_middle + p_ring) / 3.0;

  control_context_[INDEX].fp_des = p_thumb;
  control_context_[INDEX].fp_des.z() += 0.02;
  control_context_[MIDDLE].fp_des = p_thumb;
  control_context_[MIDDLE].fp_des.z() += 0.02;
  control_context_[RING].fp_des = p_thumb;
  control_context_[RING].fp_des.z() += 0.02;
  control_context_[THUMB].fp_des = center_geo;

  // Direction vectors
  Eigen::Vector3d dir_index = (control_context_[INDEX].fp_des - p_index).normalized();
  Eigen::Vector3d dir_middle = (control_context_[MIDDLE].fp_des - p_middle).normalized();
  Eigen::Vector3d dir_ring = (control_context_[RING].fp_des - p_ring).normalized();
  Eigen::Vector3d dir_thumb = (control_context_[THUMB].fp_des - p_thumb).normalized();

  // Update desired positions
  control_context_[INDEX].fp_des = p_index + dir_index;
  control_context_[MIDDLE].fp_des = p_middle + dir_middle;
  control_context_[RING].fp_des = p_ring + dir_ring;
  control_context_[THUMB].fp_des = p_thumb + dir_thumb;

  // Force balancing
  double alpha0 = control_context_[INDEX].gain.f_des;
  double alpha1 = (p_index - control_context_[INDEX].fp_des).norm() / (p_middle - control_context_[MIDDLE].fp_des).norm() * alpha0;
  double alpha2 = alpha0;

  Eigen::Vector3d force_vec0 = alpha0 * dir_index;
  Eigen::Vector3d force_vec1 = alpha1 * dir_middle;
  Eigen::Vector3d force_vec2 = alpha2 * dir_ring;

  double alpha3 = (force_vec0 + force_vec1 + force_vec2).norm();

  control_context_[MIDDLE].gain.f_des = alpha1;
  control_context_[RING].gain.f_des = alpha2;
  control_context_[THUMB].gain.f_des = alpha3;
}

/**
 * @brief Implements a pinch grasp between the index finger and the thumb.
 *
 * The other two fingers are held in a passive, curled position.
 */
void GraspAlgorithmPlexusLegacy::_motion_pinch_it() {
  const auto& p_index = finger_io_contexts_[INDEX]->state.fpos;
  const auto& p_thumb = finger_io_contexts_[THUMB]->state.fpos;

  Eigen::Vector3d center_geo = (p_index + p_thumb) / 2.0;
  double pinch_offset_y = 0.000;

  control_context_[INDEX].fp_des = center_geo;
  control_context_[INDEX].fp_des.y() += pinch_offset_y / 2.0;
  control_context_[THUMB].fp_des = center_geo;
  control_context_[THUMB].fp_des.y() -= pinch_offset_y / 2.0;

  Eigen::Vector3d dir_index = (control_context_[INDEX].fp_des - p_index).normalized();
  Eigen::Vector3d dir_thumb = (control_context_[THUMB].fp_des - p_thumb).normalized();

  control_context_[INDEX].fp_des = p_index + dir_index;
  control_context_[THUMB].fp_des = p_thumb + dir_thumb;

  if (hand_side_ == LEFT) {
    control_context_[MIDDLE].fp_des = {0.08, 0.0, 0.095};
    control_context_[RING].fp_des = {0.08, 0.040, 0.095};
  } else {
    control_context_[MIDDLE].fp_des = {0.08, 0.0, 0.095};
    control_context_[RING].fp_des = {0.08, -0.040, 0.095};
  }

  double alpha0 = control_context_[INDEX].gain.f_des;
  double alpha3 = (p_index - control_context_[INDEX].fp_des).norm() / (p_thumb - control_context_[THUMB].fp_des).norm() * alpha0;
  control_context_[THUMB].gain.f_des = alpha3;
}

/**
 * @brief Implements a pinch grasp between the middle finger and the thumb.
 *
 * The other two fingers are held in a passive, curled position.
 */
void GraspAlgorithmPlexusLegacy::_motion_pinch_mt() {
  const auto& p_middle = finger_io_contexts_[MIDDLE]->state.fpos;
  const auto& p_thumb = finger_io_contexts_[THUMB]->state.fpos;

  Eigen::Vector3d center_geo = (p_middle + p_thumb) / 2.0;

  control_context_[MIDDLE].fp_des = center_geo;
  control_context_[THUMB].fp_des = center_geo;

  Eigen::Vector3d dir_middle = (control_context_[MIDDLE].fp_des - p_middle).normalized();
  Eigen::Vector3d dir_thumb = (control_context_[THUMB].fp_des - p_thumb).normalized();

  control_context_[MIDDLE].fp_des = p_middle + dir_middle;
  control_context_[THUMB].fp_des = p_thumb + dir_thumb;

  if (hand_side_ == LEFT) {
    control_context_[INDEX].fp_des = {0.08, -0.05, 0.095};
    control_context_[RING].fp_des = {0.08, 0.040, 0.095};
  } else {
    control_context_[INDEX].fp_des = {0.08, 0.05, 0.095};
    control_context_[RING].fp_des = {0.08, -0.040, 0.095};
  }

  double alpha1 = control_context_[MIDDLE].gain.f_des;
  double alpha3 = (p_middle - control_context_[MIDDLE].fp_des).norm() / (p_thumb - control_context_[THUMB].fp_des).norm() * alpha1;
  control_context_[THUMB].gain.f_des = alpha3;
}

/**
 * @brief Implements an enveloping grasp.
 *
 * This is a pre-scripted motion where fingers close in a specific sequence to wrap around an object.
 */
void GraspAlgorithmPlexusLegacy::_motion_envelop() {
  double q_des_left[NOF][NOJ] = {{80 * DEG2RAD, 30 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},  // THUMB
                                 {10 * DEG2RAD, 60 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},  // INDEX
                                 {0 * DEG2RAD, 60 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},   // MIDDLE
                                 {-20 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD}}; // RING

  double q_des_right[NOF][NOJ] = {{80 * DEG2RAD, 30 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},  // THUMB
                                  {-10 * DEG2RAD, 60 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD}, // INDEX
                                  {0 * DEG2RAD, 60 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},   // MIDDLE
                                  {20 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD}};  // RING

  const double (*q_des)[NOJ] = (hand_side_ == LEFT) ? q_des_left : q_des_right;

  if (current_time_ > 0.15) {
    control_context_[THUMB].calc_tau_des_pd_control(0, q_des[THUMB][0]);
    control_context_[THUMB].calc_tau_des_pd_control(1, q_des[THUMB][1]);
  }

  control_context_[INDEX].calc_tau_des_pd_control(0, q_des[INDEX][0]);
  control_context_[MIDDLE].calc_tau_des_pd_control(0, q_des[MIDDLE][0]);
  control_context_[RING].calc_tau_des_pd_control(0, q_des[RING][0]);

  auto set_torque = [this](int f_id, int j_idx, double val) { control_context_[f_id].tau_des(j_idx) = val / PWM_TO_TORQUE_RATIO; };

  if (current_time_ > 0 && current_time_ <= 0.05) {
    set_torque(RING, 1, 600);
  } else if (current_time_ > 0.05 && current_time_ <= 0.1) {
    set_torque(MIDDLE, 1, 600);
    set_torque(RING, 1, 600);
    set_torque(RING, 2, 360);
  } else if (current_time_ > 0.1 && current_time_ <= 0.2) {
    set_torque(INDEX, 1, 600);
    set_torque(MIDDLE, 1, 600);
    set_torque(MIDDLE, 2, 360);
    set_torque(RING, 1, 600);
    set_torque(RING, 2, 360);
    set_torque(RING, 3, 180);
    set_torque(THUMB, 2, 600);
  } else if (current_time_ > 0.2) {
    set_torque(INDEX, 1, 600);
    set_torque(INDEX, 2, 360);
    set_torque(INDEX, 3, 180);
    set_torque(MIDDLE, 1, 600);
    set_torque(MIDDLE, 2, 360);
    set_torque(MIDDLE, 3, 180);
    set_torque(RING, 1, 600);
    set_torque(RING, 2, 360);
    set_torque(RING, 3, 180);
    set_torque(THUMB, 2, 600);
    set_torque(THUMB, 3, 420);
  }

  for (auto& cc : control_context_) {
    cc.tau_des *= _envelop_torque_scalar;
  }
}

/**
 * @brief Initializes the gain table with default values for all supported motion types.
 */
void GraspAlgorithmPlexusLegacy::_initialize_default_gain_table() {
  // MOTION_TYPE_HOME, ENVELOP, JOINT_PD
  {
    GainSet imr_gains(NOJ);
    imr_gains.set_kp({500, 800, 900, 500});
    imr_gains.set_kd({25, 50, 55, 40});

    GainSet thumb_gains(NOJ);
    thumb_gains.set_kp({1000, 700, 600, 600});
    thumb_gains.set_kd({50, 50, 50, 40});

    for (auto motion_type : {AllegroHandGrasp::MotionType::MOTION_TYPE_HOME, AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP,
                             AllegroHandGrasp::MotionType::MOTION_TYPE_JOINT_PD}) {
      control_context_[INDEX].set_gain_table_item(motion_type, imr_gains);
      control_context_[MIDDLE].set_gain_table_item(motion_type, imr_gains);
      control_context_[RING].set_gain_table_item(motion_type, imr_gains);
      control_context_[THUMB].set_gain_table_item(motion_type, thumb_gains);
    }
  }

  // MOTION_TYPE_READY, GRAVITY_COMP
  {
    const double sqrt_kp_80 = std::sqrt(80);
    const double sqrt_kp_90 = std::sqrt(90);

    GainSet imr_gains(NOJ);
    imr_gains.set_kp({80, 90, 90, 80});
    imr_gains.set_kd({sqrt_kp_80 * 0.004 * 1.0, sqrt_kp_90 * 0.004 * 2.0, sqrt_kp_90 * 0.004 * 2.0, sqrt_kp_80 * 0.004 * 1.0});

    GainSet thumb_gains(NOJ);
    thumb_gains.set_kp({90, 90, 90, 90});
    thumb_gains.set_kd({sqrt_kp_90 * 0.004 * 4.0, sqrt_kp_90 * 0.004 * 2.0, sqrt_kp_90 * 0.004 * 1.0, sqrt_kp_90 * 0.004 * 1.0});

    for (auto motion_type : {AllegroHandGrasp::MotionType::MOTION_TYPE_READY, AllegroHandGrasp::MotionType::MOTION_TYPE_GRAVITY_COMP}) {
      control_context_[INDEX].set_gain_table_item(motion_type, imr_gains);
      control_context_[MIDDLE].set_gain_table_item(motion_type, imr_gains);
      control_context_[RING].set_gain_table_item(motion_type, imr_gains);
      control_context_[THUMB].set_gain_table_item(motion_type, thumb_gains);
    }
  }

  // MOTION_TYPE_PRE_SHAPE
  {
    const double f_des_index = 40.0, f_des_middle = 3.0, f_des_ring = 0.0, f_des_thumb = 3.0;
    const double kd_index = std::sqrt(f_des_index) * 0.01;
    const double kd_middle = std::sqrt(f_des_middle) * 0.01;
    const double kd_ring = std::sqrt(f_des_ring) * 0.01;
    const double sqrt_f_des_thumb = std::sqrt(f_des_thumb);

    GainSet index_gains(NOJ);
    index_gains.f_des = f_des_index;
    index_gains.set_kd({kd_index, kd_index, kd_index, kd_index});
    control_context_[INDEX].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE, index_gains);

    GainSet middle_gains(NOJ);
    middle_gains.f_des = f_des_middle;
    middle_gains.set_kd({kd_middle, kd_middle, kd_middle, kd_middle});
    control_context_[MIDDLE].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE, middle_gains);

    GainSet ring_gains(NOJ);
    ring_gains.f_des = f_des_ring;
    ring_gains.set_kd({kd_ring, kd_ring, kd_ring, kd_ring});
    control_context_[RING].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE, ring_gains);

    GainSet thumb_gains(NOJ);
    thumb_gains.f_des = f_des_thumb;
    thumb_gains.set_kd({sqrt_f_des_thumb * 0.03, sqrt_f_des_thumb * 0.03, sqrt_f_des_thumb * 0.001, sqrt_f_des_thumb * 0.001});
    control_context_[THUMB].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE, thumb_gains);
  }

  // MOTION_TYPE_GRASP_3
  {
    const double f_des_index = 4.0, f_des_middle = 3.0, f_des_ring = 0.0, f_des_thumb = 3.0;
    const double kd_index = std::sqrt(f_des_index) * 0.01;
    const double kd_middle = std::sqrt(f_des_middle) * 0.01;
    const double sqrt_f_des_thumb = std::sqrt(f_des_thumb);

    GainSet index_gains(NOJ);
    index_gains.f_des = f_des_index;
    index_gains.set_kd({kd_index, kd_index, kd_index, kd_index});
    control_context_[INDEX].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3, index_gains);

    GainSet middle_gains(NOJ);
    middle_gains.f_des = f_des_middle;
    middle_gains.set_kd({kd_middle, kd_middle, kd_middle, kd_middle});
    control_context_[MIDDLE].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3, middle_gains);

    GainSet ring_gains(NOJ);
    const double sqrt_kp_120 = std::sqrt(120);
    const double kd_ring = sqrt_kp_120 * 0.005;
    const double kd_task = sqrt_kp_120 * 0.04;
    ring_gains.f_des = f_des_ring;
    ring_gains.set_kp({120, 120, 120, 120});
    ring_gains.set_kd({kd_ring * 1.0, kd_ring * 3.0, kd_ring * 2.0, kd_ring * 1.0});
    ring_gains.set_kd_task({kd_task, kd_task, kd_task, kd_task});
    control_context_[RING].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3, ring_gains);

    GainSet thumb_gains(NOJ);
    thumb_gains.f_des = f_des_thumb;
    thumb_gains.set_kd({sqrt_f_des_thumb * 0.03, sqrt_f_des_thumb * 0.03, sqrt_f_des_thumb * 0.001, sqrt_f_des_thumb * 0.001});
    control_context_[THUMB].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3, thumb_gains);
  }

  // MOTION_TYPE_GRASP_4
  {
    const double f_des_index = 4.0, f_des_middle = 3.0, f_des_ring = 3.0, f_des_thumb = 3.0;
    const double kd_index = std::sqrt(f_des_index) * 0.01;
    const double kd_middle = std::sqrt(f_des_middle) * 0.01;
    const double kd_ring = std::sqrt(f_des_ring) * 0.01;
    const double sqrt_f_des_thumb = std::sqrt(f_des_thumb);

    GainSet index_gains(NOJ);
    index_gains.f_des = f_des_index;
    index_gains.set_kd({kd_index, kd_index, kd_index, kd_index});
    control_context_[INDEX].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4, index_gains);

    GainSet middle_gains(NOJ);
    middle_gains.f_des = f_des_middle;
    middle_gains.set_kd({kd_middle, kd_middle, kd_middle, kd_middle});
    control_context_[MIDDLE].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4, middle_gains);

    GainSet ring_gains(NOJ);
    ring_gains.f_des = f_des_ring;
    ring_gains.set_kd({kd_ring, kd_ring, kd_ring, kd_ring});
    control_context_[RING].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4, ring_gains);

    GainSet thumb_gains(NOJ);
    thumb_gains.f_des = f_des_thumb;
    thumb_gains.set_kd({sqrt_f_des_thumb * 0.03, sqrt_f_des_thumb * 0.03, sqrt_f_des_thumb * 0.001, sqrt_f_des_thumb * 0.001});
    control_context_[THUMB].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4, thumb_gains);
  }

  // MOTION_TYPE_PINCH_IT
  {
    const double f_des_index = 5.0, f_des_thumb = 4.0;
    const double sqrt_f_des_index = std::sqrt(f_des_index);
    const double kd_index = sqrt_f_des_index * 0.005;
    const double kd_task_index = sqrt_f_des_index * 0.001;
    GainSet index_gains(NOJ);
    index_gains.f_des = f_des_index;
    index_gains.set_kd({kd_index, kd_index, kd_index, kd_index});
    index_gains.set_kd_task({kd_task_index, kd_task_index, kd_task_index, kd_task_index});
    control_context_[INDEX].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT, index_gains);

    const double sqrt_kp_120 = std::sqrt(120);
    const double kd_passive = sqrt_kp_120 * 0.005;
    const double kd_task_passive = sqrt_kp_120 * 0.04;
    GainSet passive_gains(NOJ);
    passive_gains.set_kp({120, 120, 120, 120});
    passive_gains.set_kd({kd_passive * 1.0, kd_passive * 3.0, kd_passive * 2.0, kd_passive * 1.0});
    passive_gains.set_kd_task({kd_task_passive, kd_task_passive, kd_task_passive, kd_task_passive});
    control_context_[MIDDLE].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT, passive_gains);
    control_context_[RING].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT, passive_gains);

    const double sqrt_f_des_thumb = std::sqrt(f_des_thumb);
    const double kd_thumb = sqrt_f_des_thumb * 0.03;
    const double kd_task_thumb = sqrt_f_des_thumb * 0.001;
    GainSet thumb_gains(NOJ);
    thumb_gains.f_des = f_des_thumb;
    thumb_gains.set_kd({kd_thumb, kd_thumb, kd_thumb, kd_thumb});
    thumb_gains.set_kd_task({kd_task_thumb, kd_task_thumb, kd_task_thumb, kd_task_thumb});
    control_context_[THUMB].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT, thumb_gains);
  }

  // MOTION_TYPE_PINCH_MT
  {
    const double f_des_middle = 5.0, f_des_thumb = 4.0;
    const double sqrt_kp_120 = std::sqrt(120);
    const double kd_passive = sqrt_kp_120 * 0.005;
    const double kd_task_passive = sqrt_kp_120 * 0.04;
    GainSet passive_gains(NOJ);
    passive_gains.set_kp({120, 120, 120, 120});
    passive_gains.set_kd({kd_passive * 1.0, kd_passive * 3.0, kd_passive * 2.0, kd_passive * 1.0});
    passive_gains.set_kd_task({kd_task_passive, kd_task_passive, kd_task_passive, kd_task_passive});
    control_context_[INDEX].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT, passive_gains);
    control_context_[RING].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT, passive_gains);

    const double sqrt_f_des_middle = std::sqrt(f_des_middle);
    const double kd_middle = sqrt_f_des_middle * 0.005;
    const double kd_task_middle = sqrt_f_des_middle * 0.001;
    GainSet middle_gains(NOJ);
    middle_gains.f_des = f_des_middle;
    middle_gains.set_kd({kd_middle, kd_middle, kd_middle, kd_middle});
    middle_gains.set_kd_task({kd_task_middle, kd_task_middle, kd_task_middle, kd_task_middle});
    control_context_[MIDDLE].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT, middle_gains);

    const double sqrt_f_des_thumb = std::sqrt(f_des_thumb);
    const double kd_thumb = sqrt_f_des_thumb * 0.03;
    const double kd_task_thumb = sqrt_f_des_thumb * 0.001;
    GainSet thumb_gains(NOJ);
    thumb_gains.f_des = f_des_thumb;
    thumb_gains.set_kd({kd_thumb, kd_thumb, kd_thumb, kd_thumb});
    thumb_gains.set_kd_task({kd_task_thumb, kd_task_thumb, kd_task_thumb, kd_task_thumb});
    control_context_[THUMB].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT, thumb_gains);
  }

  // MOTION_TYPE_NONE
  {
    GainSet zero_gains(NOJ);
    for (int f_idx = 0; f_idx < NOF; ++f_idx) {
      control_context_[f_idx].set_gain_table_item(AllegroHandGrasp::MotionType::MOTION_TYPE_NONE, zero_gains);
    }
  }

#if 0 // Note: It seems better not to scale the gains. (Why? Possibly due to noise?)
  // The original gains were tuned for a 333Hz control loop.
  // If the update frequency is different, the derivative gains (kd, kd_task) must be scaled.
  const double original_frequency = 333.0;
  const double target_frequency = 100.0; // Assuming the new target is 100Hz

  if (std::abs(original_frequency - target_frequency) > 1e-6) {
    
    const double kd_scaling_factor = original_frequency / target_frequency;
    for (auto& cc : control_context_) {
      for (auto& [motion_type, gain_set] : cc.gain_table) {
        gain_set.kd *= kd_scaling_factor;
        gain_set.kd_task *= kd_scaling_factor;
      }
    }
  }
#endif
}

/**
 * @brief Updates the entire gain table for all fingers from a YAML formatted string.
 *
 * This function parses a YAML string to configure the control gains (kp, kd, etc.)
 * for various motion types and fingers. It overwrites existing gain values in the
 * internal gain table. If the YAML is malformed or contains unknown motion/finger names,
 * an error is logged, and the function may return false.
 *
 * @param gain_yaml A string containing the gains in YAML format.
 * @return True if the gain table was successfully updated, false otherwise.
 *
 * @par YAML Format Example:
 * @code{.yaml}
 * home:
 *   thumb:
 *     kp: [1000, 700, 600, 600]
 *     kd: [50, 50, 50, 40]
 *     kp_task: [0, 0, 0, 0]
 *     kd_task: [0, 0, 0, 0]
 *     f_des: 0.0
 *   # ... other fingers and motions
 * @endcode
 */
bool GraspAlgorithmPlexusLegacy::set_gain_table(std::string gain_yaml) {
  try {
    YAML::Node root = YAML::Load(gain_yaml);
    if (!root.IsMap()) {
      SPDLOG_ERROR("YAML root is not a map.");
      return false;
    }

    // Create a reverse map for finger lookup
    std::map<std::string, FingerID> finger_from_string;
    for (const auto& pair : finger_to_string) {
      finger_from_string[pair.second] = pair.first;
    }

    // Iterate over motions in the YAML (e.g., "home", "ready")
    for (const auto& motion_it : root) {
      std::string motion_name = motion_it.first.as<std::string>();
      if (GraspAlgorithm::MOTION_TYPE_FROM_STRING.count(motion_name) == 0) {
        SPDLOG_WARN("Unknown motion type '{}' in YAML, skipping.", motion_name);
        continue;
      }
      AllegroHandGrasp::MotionType motion_type = GraspAlgorithm::MOTION_TYPE_FROM_STRING.at(motion_name);
      YAML::Node motion_node = motion_it.second;

      // Iterate over fingers for the current motion (e.g., "thumb", "index")
      for (const auto& finger_it : motion_node) {
        std::string finger_name = finger_it.first.as<std::string>();
        if (finger_from_string.count(finger_name) == 0) {
          SPDLOG_WARN("Unknown finger name '{}' for motion type '{}' in YAML, skipping.", finger_name, motion_name);
          continue;
        }
        FingerID f_idx = finger_from_string.at(finger_name);
        auto& cc = control_context_[f_idx];

        YAML::Node gain_node = finger_it.second;

        GainSet gain_set(NOJ);
        gain_set.set_kp(gain_node["kp"].as<std::vector<double>>());
        gain_set.set_kd(gain_node["kd"].as<std::vector<double>>());
        gain_set.set_kp_task(gain_node["kp_task"].as<std::vector<double>>());
        gain_set.set_kd_task(gain_node["kd_task"].as<std::vector<double>>());
        gain_set.f_des = gain_node["f_des"].as<double>();

        cc.gain_table[motion_type] = gain_set;
      }
    }
  } catch (const YAML::Exception& e) {
    SPDLOG_ERROR("Failed to parse gain table YAML: {}", e.what());
    return false;
  }
  return true;
}

/**
 * @brief Exports the entire internal gain table to a YAML formatted string.
 *
 * This function serializes the control gains for all configured motion types and
 * fingers into a YAML string. This is useful for saving the current gain configuration
 * or for debugging purposes.
 *
 * @param[out] gain_yaml A string reference that will be populated with the YAML
 *                       representation of the gain table.
 * @return True on successful export, false otherwise.
 *
 * @par YAML Format Example:
 * The output format is identical to the one expected by set_gain_table().
 * @code{.yaml}
 * home:
 *   thumb:
 *     kp: [1000, 700, 600, 600]
 *     # ... other gain values and fingers
 * @endcode
 */
bool GraspAlgorithmPlexusLegacy::get_gain_table(std::string& gain_yaml) {
  YAML::Node root;

  // Iterate over all possible motion types
  for (const auto& pair : GraspAlgorithm::MOTION_TYPE_TO_STRING) {
    AllegroHandGrasp::MotionType motion_type = pair.first;
    const std::string& motion_name = pair.second;

    YAML::Node motion_node;
    bool motion_has_gains = false;

    // For each motion, iterate over all fingers
    for (int f_idx = 0; f_idx < NOF; ++f_idx) {
      const auto& cc = control_context_[f_idx];
      const std::string finger_name = finger_to_string.at(static_cast<FingerID>(f_idx));

      auto it = cc.gain_table.find(motion_type);
      if (it != cc.gain_table.end()) {
        motion_has_gains = true;
        const GainSet& gain_set = it->second;

        YAML::Node gain_node;
        gain_node["kp"] = std::vector<double>(gain_set.kp.data(), gain_set.kp.data() + gain_set.kp.size());
        gain_node["kd"] = std::vector<double>(gain_set.kd.data(), gain_set.kd.data() + gain_set.kd.size());
        gain_node["kp_task"] = std::vector<double>(gain_set.kp_task.data(), gain_set.kp_task.data() + gain_set.kp_task.size());
        gain_node["kd_task"] = std::vector<double>(gain_set.kd_task.data(), gain_set.kd_task.data() + gain_set.kd_task.size());
        gain_node["f_des"] = gain_set.f_des;

        motion_node[finger_name] = gain_node;
      }
    }

    if (motion_has_gains) {
      root[motion_name] = motion_node;
    }
  }
  std::stringstream ss;
  ss << root;
  gain_yaml = ss.str();
  return true;
}

/**
 * @brief Updates the gains for a specific motion type from a YAML formatted string.
 *
 * This function parses a YAML string to configure the control gains for a single,
 * specified motion type. The YAML should contain the gain data for one or more
 * fingers. This allows for targeted updates to the gain table without needing to
 * provide the entire configuration.
 *
 * @param motion_type The motion type for which to update the gains.
 * @param gain_yaml A YAML formatted string containing the gains for the fingers.
 * @return True if the gains were successfully updated, false otherwise.
 *
 * @par YAML Format Example:
 * @code{.yaml}
 * thumb:
 *   kp: [1000, 700, 600, 600]
 *   # ... other gain values
 * index:
 *   # ... other gain values
 * @endcode
 */
bool GraspAlgorithmPlexusLegacy::set_gain_table_item(AllegroHandGrasp::MotionType motion_type, std::string gain_yaml) {
  try {
    YAML::Node root = YAML::Load(gain_yaml);
    if (!root.IsMap()) {
      SPDLOG_ERROR("YAML root is not a map for set_gain_table_item.");
      return false;
    }

    // Create a reverse map for finger lookup
    std::map<std::string, FingerID> finger_from_string;
    for (const auto& pair : finger_to_string) {
      finger_from_string[pair.second] = pair.first;
    }

    // Iterate over fingers in the YAML (e.g., "thumb", "index")
    for (const auto& finger_it : root) {
      std::string finger_name = finger_it.first.as<std::string>();
      if (finger_from_string.count(finger_name) == 0) {
        SPDLOG_WARN("Unknown finger name '{}' in YAML, skipping.", finger_name);
        continue;
      }
      FingerID f_idx = finger_from_string.at(finger_name);
      auto& cc = control_context_[f_idx];

      YAML::Node gain_node = finger_it.second;

      GainSet gain_set(NOJ);
      gain_set.set_kp(gain_node["kp"].as<std::vector<double>>());
      gain_set.set_kd(gain_node["kd"].as<std::vector<double>>());
      gain_set.set_kp_task(gain_node["kp_task"].as<std::vector<double>>());
      gain_set.set_kd_task(gain_node["kd_task"].as<std::vector<double>>());
      gain_set.f_des = gain_node["f_des"].as<double>();

      cc.gain_table[motion_type] = gain_set;
    }
  } catch (const YAML::Exception& e) {
    SPDLOG_ERROR("Failed to parse gain YAML for motion type '{}': {}", GraspAlgorithm::MOTION_TYPE_TO_STRING.at(motion_type), e.what());
    return false;
  }
  return true;
}

/**
 * @brief Exports the gains for a specific motion type to a YAML formatted string.
 *
 * This function serializes the control gains for a single, specified motion type
 * into a YAML string. This is useful for retrieving the configuration of a
 * particular motion for inspection or backup.
 *
 * @param motion_type The motion type for which to export the gains.
 * @param[out] gain_yaml A string reference that will be populated with the YAML data.
 * @return True on successful export, false otherwise.
 *
 * @par YAML Format Example:
 * The output format is identical to the one expected by set_gain_table_item().
 * @code{.yaml}
 * thumb:
 *   kp: [1000, 700, 600, 600]
 *   # ... other gain values and fingers
 * @endcode
 */
bool GraspAlgorithmPlexusLegacy::get_gain_table_item(AllegroHandGrasp::MotionType motion_type, std::string& gain_yaml) {
  if (GraspAlgorithm::MOTION_TYPE_TO_STRING.count(motion_type) == 0)
    return false;

  YAML::Node root;

  for (int f_idx = 0; f_idx < NOF; ++f_idx) {
    const auto& cc = control_context_[f_idx];
    const std::string finger_name = finger_to_string.at(static_cast<FingerID>(f_idx));

    auto it = cc.gain_table.find(motion_type);
    if (it == cc.gain_table.end()) {
      SPDLOG_WARN("Gain for motion type '{}' not found for finger '{}'.", GraspAlgorithm::MOTION_TYPE_TO_STRING.at(motion_type),
                  finger_name);
      continue;
    }
    const GainSet& gain_set = it->second;

    YAML::Node gain_node;
    std::vector<double> kp(gain_set.kp.data(), gain_set.kp.data() + gain_set.kp.size());
    std::vector<double> kd(gain_set.kd.data(), gain_set.kd.data() + gain_set.kd.size());
    std::vector<double> kp_task(gain_set.kp_task.data(), gain_set.kp_task.data() + gain_set.kp_task.size());
    std::vector<double> kd_task(gain_set.kd_task.data(), gain_set.kd_task.data() + gain_set.kd_task.size());
    gain_node["kp"] = kp;
    gain_node["kd"] = kd;
    gain_node["kp_task"] = kp_task;
    gain_node["kd_task"] = kd_task;
    gain_node["f_des"] = gain_set.f_des;
    root[finger_name] = gain_node;
  }
  std::stringstream ss;
  ss << root;
  gain_yaml = ss.str();
  return true;
}
