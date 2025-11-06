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

#include <functional>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <spdlog/spdlog.h>
#include <unordered_map>

/*
 * install pinocchio
 *  - sudo apt install ros-$ROS_DISTRO-pinocchio
 */
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <boost/algorithm/string/join.hpp>

#include <allegro_hand_grasp_library/ah_grasp.hpp>

#include "ah_grasp_interface.hpp"

/*
 * @brief A namespace for utility functions.
 */
namespace util {
template <typename Type>
static std::string to_string(const Type& v) {
  std::stringstream ss;
  ss << v;
  return ss.str();
}

template <typename MatType>
static std::string mat2str(const MatType& mat) {
  return to_string(mat);
}
} // namespace util

/**
 * @class AllegroHandGraspImpl
 * @brief Concrete implementation of the AllegroHandGrasp interface.
 *
 * This class manages the kinematic model of the Allegro Hand using Pinocchio,
 * handles the state updates, and delegates the grasp control logic to a
 * specific GraspAlgorithm instance. It serves as the main backend for the
 * grasp effort controller.
 */
class AllegroHandGraspImpl : public AllegroHandGrasp {
  /// Pinocchio model of the robot, built from URDF.
  std::unique_ptr<pinocchio::Model> model_;
  /// Pinocchio data associated with the model, used for computations.
  std::unique_ptr<pinocchio::Data> data_;
  /// Configuration options provided at initialization.
  Options options_;

  /// The detected side of the hand (LEFT or RIGHT).
  HandSide hand_side_;
  /// Transformation from the palm frame to the world frame.
  Eigen::Isometry3d T_palm2world_;
  /// Transformation from the world frame to the palm frame.
  Eigen::Isometry3d T_world2palm_;
  /// Adjoint transformation matrix from the world frame to the palm frame.
  Eigen::MatrixXd Adj_world2palm_;

  /// The currently active motion type.
  MotionType motion_type_;

  /// Vector of I/O contexts, one for each finger (thumb, index, middle, ring).
  std::vector<std::unique_ptr<FingerIoContext>> finger_io_contexts_;

  /// The active grasp control algorithm instance.
  GraspAlgorithm::Ptr grasp_algorithm_;

  /**
   * @brief Verifies that the provided options are valid and consistent.
   *
   * Checks for the presence of URDF data, frame names, and their existence
   * within the Pinocchio model. Throws a `std::runtime_error` if any
   * verification step fails.
   */

  void verifiy_options() {
    bool ok = true;
    std::string exception_message;

    do {
      if (options_.urdf_string.empty()) {
        ok = false;
        exception_message = "urdf_string is empty.";
        break;
      }

      if (options_.fingertip_link_names.empty()) {
        ok = false;
        exception_message = "fingertip_link_names is empty.";
        break;
      }

      if (options_.fingertip_link_names.size() < 3) {
        ok = false;
        exception_message = "fingertip_link_names.size() < 3";
        break;
      }

      if (options_.reference_frame_name.empty()) {
        ok = false;
        exception_message = "reference_frame_name is empty.";
        break;
      }

      if (model_) {
        if (!model_->existFrame(options_.reference_frame_name)) {
          ok = false;
          exception_message = fmt::format("reference_frame_name({}) is not exit", options_.reference_frame_name);
          break;
        }

        for (const auto& fingertip_link_name : options_.fingertip_link_names) {
          if (!model_->existFrame(fingertip_link_name)) {
            ok = false;
            exception_message = fmt::format("fingertip_link_name({}) is not exit", fingertip_link_name);
            break;
          }
        }
      }

    } while (0);

    if (!ok) {
      throw std::runtime_error(fmt::format("Options verification failed. {}", exception_message));
    }

    return;
  }

  /**
   * @brief Automatically detects whether the hand is a left or right hand.
   *
   * This method analyzes the kinematic structure from the URDF. It determines
   * the hand's side by observing the relative position of the thumb with respect
   * to a temporary coordinate frame defined by the palm and middle finger.
   *
   * @param palm_link_name The name of the palm link, used as the reference frame.
   * @throws std::runtime_error if the hand side cannot be determined.
   */
  void detect_handside(const std::string& palm_link_name) {
    // --- 1. Preparation Step: Get all necessary frame IDs ---
    assert(options_.fingertip_link_names.size() >= 3);
    auto thumb_tip_name = options_.fingertip_link_names[0];
    auto middle_tip_name = options_.fingertip_link_names[2];

    assert(model_->existFrame(palm_link_name));
    assert(model_->existFrame(thumb_tip_name));
    assert(model_->existFrame(middle_tip_name));

    const pinocchio::FrameIndex palm_id = model_->getFrameId(palm_link_name);
    const pinocchio::FrameIndex thumb_id = model_->getFrameId(thumb_tip_name);
    const pinocchio::FrameIndex middle_id = model_->getFrameId(middle_tip_name);

    std::vector<pinocchio::FrameIndex> all_fingertip_ids;
    for (const auto& name : options_.fingertip_link_names) {
      all_fingertip_ids.push_back(model_->getFrameId(name));
    }
    const size_t num_fingertips = all_fingertip_ids.size();

    // --- 2. Calculate positions in 'open hand pose' (q=0) and set the origin ---
    Eigen::VectorXd q_neutral = pinocchio::neutral(*model_);
    pinocchio::forwardKinematics(*model_, *data_, q_neutral);
    pinocchio::updateFramePlacements(*model_, *data_);

    // (Final revision) Explicitly set the temporary origin to the position of the given palm_link.
    const Eigen::Vector3d temp_origin = data_->oMf[palm_id].translation();

    std::vector<Eigen::Vector3d> p_neutral(num_fingertips);
    for (size_t i = 0; i < num_fingertips; ++i) {
      p_neutral[i] = data_->oMf[all_fingertip_ids[i]].translation();
    }

    // --- 3. Calculate positions in 'slightly flexed pose' (q=alpha) ---
    Eigen::VectorXd q_flex = q_neutral;
    const double alpha = 0.1;
    std::set<pinocchio::JointIndex> joints_to_flex;
    for (const auto& frame_id : all_fingertip_ids) {
      const pinocchio::JointIndex joint_id = model_->frames[frame_id].parentJoint;
      if (joint_id > 0)
        joints_to_flex.insert(joint_id);
    }
    for (const auto& joint_id : joints_to_flex) {
      const auto& joint = model_->joints[joint_id];
      const int q_idx = joint.idx_q();
      if (q_idx >= 0)
        q_flex.segment(q_idx, joint.nq()).array() += alpha;
    }
    pinocchio::forwardKinematics(*model_, *data_, q_flex);
    pinocchio::updateFramePlacements(*model_, *data_);

    // --- 4. Calculate Z and Y axes ---
    Eigen::Vector3d avg_displacement = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < num_fingertips; ++i) {
      avg_displacement += (data_->oMf[all_fingertip_ids[i]].translation() - p_neutral[i]);
    }
    Eigen::Vector3d temp_z_axis = -(avg_displacement.normalized());

    // Set the Y-axis (forward direction) based on the middle finger's direction.
    const Eigen::Vector3d P_middle_neutral = data_->oMf[middle_id].translation();
    Eigen::Vector3d middle_dir = P_middle_neutral - temp_origin;
    Eigen::Vector3d temp_y_axis = (middle_dir - middle_dir.dot(temp_z_axis) * temp_z_axis).normalized();

    // --- 5. Calculate X-axis ---
    Eigen::Vector3d temp_x_axis = temp_y_axis.cross(temp_z_axis);

    /*
     * Temporary Palm Frame
     *  Y-axis: Direction of the middle finger.
     *  Z-axis: Direction of the back of the hand.
     */

    // --- 6. Final Determination ---
    const Eigen::Vector3d P_thumb_neutral = data_->oMf[thumb_id].translation();
    Eigen::Vector3d thumb_vec = P_thumb_neutral - temp_origin; // relative position with respect to palm_link
    const double thumb_x_component = thumb_vec.dot(temp_x_axis);
    const double threshold = 1e-9;

    if (thumb_x_component > threshold) {
      hand_side_ = LEFT;
    } else if (thumb_x_component < -threshold) {
      hand_side_ = RIGHT;
    } else {
      throw std::runtime_error("Handside detection failed: thumb is aligned with the YZ plane.");
    }

    SPDLOG_TRACE("Handside : {}", hand_side_ == RIGHT ? "RIGHT" : "LEFT");
  }

public:
  /**
   * @brief Constructs the AllegroHandGraspImpl object.
   *
   * @param options The configuration options for the grasp controller.
   */
  AllegroHandGraspImpl(const Options& options) : options_(options) {
    verifiy_options();

    model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModelFromXML(options_.urdf_string, *model_);
    data_ = std::make_unique<pinocchio::Data>(*model_);

    verifiy_options();
    detect_handside(options_.reference_frame_name);

    pinocchio::framesForwardKinematics(*model_, *data_, pinocchio::neutral(*model_));

    for (const auto& fingertip_link_name : options_.fingertip_link_names) {
      auto joints = get_joints(fingertip_link_name);
      finger_io_contexts_.push_back(std::make_unique<FingerIoContext>(joints));
      SPDLOG_TRACE("joints[{}] : {}", fingertip_link_name, boost::algorithm::join(joints, " -> "));
    }

    /*
     *
     */
    {
      if (!model_->existFrame(options_.reference_frame_name)) {
        throw std::invalid_argument("Error: Root frame '" + options_.reference_frame_name + "' does not exist in the model.");
      }
      const pinocchio::FrameIndex reference_frame_id = model_->getFrameId(options_.reference_frame_name);
      const auto& oMf_ref = data_->oMf[reference_frame_id];
      T_palm2world_.linear() = oMf_ref.rotation();
      T_palm2world_.translation() = oMf_ref.translation();
      T_world2palm_ = T_palm2world_.inverse();

#if 0
      Adj_world2palm_ = pinocchio::SE3(T_world2palm_.matrix()).toActionMatrix(); // Adjoint Matrix
#else
      /*
       * Pinocchio bug?
       * The Jacobian obtained with pinocchio::getFrameJacobian is [J_v, J_w].
       *  - This is weird in the first place. The documentation seems to describe it as [J_w, J_v],
       *  - but the actual values clearly show the order is [J_v, J_w].
       * However, the Adjoint Matrix obtained with pinocchio::SE3 is Ad(T) = [[R, p_hat R]; [0, R]].
       * W.T.F.
       */
      Eigen::MatrixXd Adj = pinocchio::SE3(T_world2palm_.matrix()).toActionMatrix(); // Adjoint Matrix
      Adj_world2palm_ = Eigen::MatrixXd(6, 6);
      Adj_world2palm_.block<3, 3>(0, 0) = Adj.block<3, 3>(0, 0).transpose();
      Adj_world2palm_.block<3, 3>(0, 3) = Adj.block<3, 3>(3, 0).transpose();
      Adj_world2palm_.block<3, 3>(3, 0) = Adj.block<3, 3>(0, 3).transpose();
      Adj_world2palm_.block<3, 3>(3, 3) = Adj.block<3, 3>(3, 3).transpose();
#endif

      // SPDLOG_TRACE("T_palm2world_\n{}", util::mat2str(T_palm2world_.matrix()));
      // SPDLOG_TRACE("T_world2palm_\n{}", util::mat2str(T_world2palm_.matrix()));
      // SPDLOG_TRACE("Adj_world2palm_\n{}", util::mat2str(Adj_world2palm_.matrix()));
    }

    grasp_algorithm_ = GraspAlgorithm::Create(options_.device, finger_io_contexts_, hand_side_);

    set_motion_type(MOTION_TYPE_NONE);
  }

  virtual ~AllegroHandGraspImpl() {}

  /**
   * @brief Retrieves the chain of joint names for a given finger.
   *
   * Traverses the kinematic tree from the specified fingertip link up to the
   * base, collecting all parent joint names in order.
   * @param fingertip_link_name The name of the fingertip link.
   * @return A vector of joint names, ordered from base to tip.
   */
  std::vector<std::string> get_joints(std::string fingertip_link_name) {
    if (!model_->existFrame(fingertip_link_name)) {
      throw std::invalid_argument("Error: Frame '" + fingertip_link_name + "' does not exist in the model.");
    }

    auto fingertip_frame_id = model_->getFrameId(fingertip_link_name);
    auto current_joint_id = model_->frames[fingertip_frame_id].parentJoint;

    std::vector<std::string> joints;
    while (current_joint_id != 0) {
      joints.push_back(model_->names[current_joint_id]);
      current_joint_id = model_->parents[current_joint_id];
    }

    std::reverse(joints.begin(), joints.end());
    return joints;
  }

  Info get_info() override {
    Info info;
    info.hand_side = (hand_side_ == RIGHT);
    return info;
  }

  bool set_gain_table(std::string gain_yaml) override { return grasp_algorithm_->set_gain_table(gain_yaml); }
  bool get_gain_table(std::string& gain_yaml) override { return grasp_algorithm_->get_gain_table(gain_yaml); }

  bool set_gain_table_item(MotionType motion_type, std::string gain_yaml) override {
    return grasp_algorithm_->set_gain_table_item(motion_type, gain_yaml);
  }
  bool get_gain_table_item(MotionType motion_type, std::string& gain_yaml) override {
    return grasp_algorithm_->get_gain_table_item(motion_type, gain_yaml);
  }

  /// @copydoc AllegroHandGrasp::set_motion_type
  bool set_motion_type(MotionType motion_type) override {
    motion_type_ = motion_type;
    grasp_algorithm_->set_motion(motion_type);
    return true;
  }

  void set_envelop_torque(double envelop_torque) override { grasp_algorithm_->set_envelop_torque(envelop_torque); }

  /// @copydoc AllegroHandGrasp::get_joint_torque
  void get_joint_torque(std::map<std::string, double>& joint_torque) override {
    joint_torque.clear();
    for (const auto& joint_context : finger_io_contexts_) {
      for (size_t i = 0; i < joint_context->joint_num(); i++) {
        const auto& joint_name = joint_context->joint_chain[i];
        joint_torque[joint_name] = joint_context->command.torque(i);
      }
    }
  }

  /**
   * @brief Sets the current joint states and updates the kinematic model.
   *
   * This is a critical function called in each control cycle. It takes the
   * current joint positions and velocities, updates the Pinocchio model's
   * forward kinematics, computes the Jacobian for each fingertip, and populates
   * the `FingerIoContext` for each finger with the latest state information.
   *
   * @param joint_state A map from joint names to pairs of (position, velocity).
   */
  void set_joint_state(const std::map<std::string, std::pair<double, double>>& joint_state) override {
    auto cur_q = pinocchio::neutral(*model_);

    for (const auto& [joint_name, state] : joint_state) {
      const auto& q = state.first;
      if (model_->existJointName(joint_name)) {
        auto joint_id = model_->getJointId(joint_name);
        int q_index = model_->joints[joint_id].idx_q();
        cur_q[q_index] = q;
      }
    }

    pinocchio::framesForwardKinematics(*model_, *data_, cur_q);
    pinocchio::updateFramePlacements(*model_, *data_);

    for (size_t finger_idx = 0; finger_idx < options_.fingertip_link_names.size(); finger_idx++) {
      const auto& fingertip_name = options_.fingertip_link_names.at(finger_idx);
      auto fingertip_frame_id = model_->getFrameId(fingertip_name);
      auto& joint_context = finger_io_contexts_.at(finger_idx);

      // fingertip pose w.r.t palm link frame
      joint_context->state.fpos = T_world2palm_ * data_->oMf[fingertip_frame_id].translation();

      for (size_t joint_idx = 0; joint_idx < joint_context->joint_num(); joint_idx++) {
        const auto& joint_name = joint_context->joint_chain[joint_idx];
        const auto& state = joint_state.at(joint_name);
        const auto& q = state.first;
        const auto& q_dot = state.second;
        joint_context->state.q(joint_idx) = q;
        joint_context->state.q_dot(joint_idx) = q_dot;
      }
    }

    pinocchio::computeJointJacobians(*model_, *data_, cur_q);

    for (size_t finger_idx = 0; finger_idx < options_.fingertip_link_names.size(); finger_idx++) {
      auto& joint_context = finger_io_contexts_.at(finger_idx);

      const auto& fingertip_link_name = options_.fingertip_link_names.at(finger_idx);
      auto fingertip_frame_id = model_->getFrameId(fingertip_link_name);

      Eigen::MatrixXd J_ftip_w(6, model_->nv);
      J_ftip_w.setZero();
      pinocchio::getFrameJacobian(*model_, *data_, fingertip_frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_ftip_w);
      Eigen::MatrixXd J_ftip = Adj_world2palm_ * J_ftip_w;

      // SPDLOG_TRACE("J_ftip_w({})\n{}", fingertip_link_name, util::mat2str(J_ftip_w));
      // SPDLOG_TRACE("J_ftip({})\n{}", fingertip_link_name, util::mat2str(J_ftip));

      // Extract the translational part of the Jacobian
      Eigen::MatrixXd finger_jacobian(3, joint_context->joint_num());
      const Eigen::MatrixXd J_trans = J_ftip.topRows(3);
      for (size_t i = 0; i < joint_context->joint_num(); ++i) {
        const auto& joint_name = joint_context->joint_chain[i];
        auto joint_id = model_->getJointId(joint_name);
        if (model_->joints[joint_id].nv() > 0) {
          // Get the column index in the full Jacobian corresponding to this joint
          int v_index = model_->joints[joint_id].idx_v();
          finger_jacobian.col(i) = J_trans.col(v_index);
        } else {
          // This joint is fixed, its Jacobian column is zero.
          finger_jacobian.col(i).setZero();
        }
      }

      joint_context->state.jacobian = finger_jacobian;
    }
  }

  /**
   * @brief Retrieves the current positions of all fingertips.
   * @return A map from fingertip link names to their 3D positions.
   */
  std::map<std::string, Eigen::Vector3d> get_fingertip_positions() override {
    std::map<std::string, Eigen::Vector3d> fingertip_positions;
    for (size_t finger_idx = 0; finger_idx < options_.fingertip_link_names.size(); finger_idx++) {
      const auto& fingertip_name = options_.fingertip_link_names.at(finger_idx);
      fingertip_positions[fingertip_name] = finger_io_contexts_[finger_idx]->state.fpos;
    }
    return fingertip_positions;
  }

  /// @copydoc AllegroHandGrasp::update
  void update(double dt) override { grasp_algorithm_->update(dt); }
}; // class AllegroHandGrasp

/// @copydoc GraspAlgorithm::MOTION_TYPE_FROM_STRING
const std::map<std::string, AllegroHandGrasp::MotionType> GraspAlgorithm::MOTION_TYPE_FROM_STRING = {
    {"none", AllegroHandGrasp::MotionType::MOTION_TYPE_NONE},
    {"home", AllegroHandGrasp::MotionType::MOTION_TYPE_HOME},
    {"ready", AllegroHandGrasp::MotionType::MOTION_TYPE_READY},
    {"gravity_comp", AllegroHandGrasp::MotionType::MOTION_TYPE_GRAVITY_COMP},
    {"pre_shape", AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE},
    {"grasp_3", AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3},
    {"grasp_4", AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4},
    {"pinch_it", AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT},
    {"pinch_mt", AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT},
    {"object_moving", AllegroHandGrasp::MotionType::MOTION_TYPE_OBJECT_MOVING},
    {"envelop", AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP},
    {"joint_pd", AllegroHandGrasp::MotionType::MOTION_TYPE_JOINT_PD},
    {"move_obj", AllegroHandGrasp::MotionType::MOTION_TYPE_MOVE_OBJ},
    {"fingertip_moving", AllegroHandGrasp::MotionType::MOTION_TYPE_FINGERTIP_MOVING},
};

/// @copydoc GraspAlgorithm::MOTION_TYPE_TO_STRING
const std::map<AllegroHandGrasp::MotionType, std::string> GraspAlgorithm::MOTION_TYPE_TO_STRING = {
    {AllegroHandGrasp::MotionType::MOTION_TYPE_NONE, "none"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_HOME, "home"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_READY, "ready"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_GRAVITY_COMP, "gravity_comp"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE, "pre_shape"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3, "grasp_3"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4, "grasp_4"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT, "pinch_it"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT, "pinch_mt"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_OBJECT_MOVING, "object_moving"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP, "envelop"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_JOINT_PD, "joint_pd"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_MOVE_OBJ, "move_obj"},
    {AllegroHandGrasp::MotionType::MOTION_TYPE_FINGERTIP_MOVING, "fingertip_moving"},
};

/// @copydoc AllegroHandGrasp::Create
AllegroHandGrasp::Ptr AllegroHandGrasp::Create(const Options& options) { return std::make_shared<AllegroHandGraspImpl>(options); }

// Forward declarations for factory functions in other files.
extern GraspAlgorithm::Ptr CreateGraspAlgorithmV4LegacyReference(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts,
                                                                 HandSide hand_side);

extern GraspAlgorithm::Ptr CreateGraspAlgorithmV4Legacy(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts,
                                                        HandSide hand_side);

extern GraspAlgorithm::Ptr CreateGraspAlgorithmPlexusLegacy(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts,
                                                            HandSide hand_side);

/// @copydoc GraspAlgorithm::Create
GraspAlgorithm::Ptr GraspAlgorithm::Create(AllegroHandGrasp::Options::HandDevice device,
                                           std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts, HandSide hand_side) {

  spdlog::set_level(spdlog::level::trace);

  switch (device) {
  case AllegroHandGrasp::Options::HandDevice::HAND_V4_REF_DEBUG:
    return CreateGraspAlgorithmV4LegacyReference(finger_io_contexts, hand_side); // Obsolete. REMOVE ME
  case AllegroHandGrasp::Options::HandDevice::HAND_V4:
    return CreateGraspAlgorithmV4Legacy(finger_io_contexts, hand_side);
  case AllegroHandGrasp::Options::HandDevice::HAND_PLEXUS:
    return CreateGraspAlgorithmPlexusLegacy(finger_io_contexts, hand_side);
  default:
    return nullptr;
  }
}
