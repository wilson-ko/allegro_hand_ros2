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

#include <spdlog/spdlog.h>

#include "ah_grasp_interface.hpp"

/*
 * legacy code를 테스트하기 위해 인터페이스만 맞추고 Isolation 한다.
 *
 * Legacy Code 에서의 Matrix 형식
 *
 *         LINK0, LINK1, LINK2, LINK3
 *   INDEX     X,     X,     X,     X
 *  MIDDLE     X,     X,     X,     X
 *    RING     X,     X,     X,     X
 *   THUMB     X,     X,     X,     X
 */

/*
 *
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

class LowpassFilter {
  double v_prv;
  double v_filtered;

public:
  LowpassFilter() : v_prv(0), v_filtered(0) {}
  inline double filtering(double v) {
    v_filtered = (0.6 * v_filtered) + (0.198 * v_prv) + (0.198 * v);
    v_prv = v;
    return v_filtered;
  }
};

/*
 * 개발을 위한 중간형태. Legacy Code 의 형태 유지
 */
class GraspAlgorithmV4LegacyReference : public GraspAlgorithm {
  static constexpr int NOF = 4; // number of fingers. [INDEX, MIDDLE, RING, THUMB]
  static constexpr int NOJ = 4; // number of joints in each finger
  static constexpr double DEG2RAD = M_PI / 180;

  /*
   * Index of Finger Context
   */
  enum FingerID {
    THUMB = 0,
    INDEX,
    MIDDLE,
    RING,
  };

  // -- GAIN ----
  double _kp[NOF][NOJ];      ///< proportional control gain for each joint
  double _kd[NOF][NOJ];      ///< derivative control gain for each joint
  double _kp_task[NOF][NOJ]; ///<
  double _kd_task[NOF][NOJ]; ///<

  // -- TEMP ----
  double _f_des[NOF]; ///< desired force
  double _x_des[NOF]; ///< desired x position in cartesian coordinate for the center of each finger tip
  double _y_des[NOF]; ///< desired y position in cartesian coordinate for the center of each finger tip
  double _z_des[NOF]; ///< desired z position in cartesian coordinate for the center of each finger tip

  // -- STAMP ----
  double _current_time;

  // -- STATE ----
  double _x[NOF]; ///< x position of finger tip along with cartesian space
  double _y[NOF]; ///< y position of finger tip along with cartesian space
  double _z[NOF]; ///< z position of finger tip along with cartesian space

  double _q_filtered[NOF][NOJ];    ///< current joint angle (radian, low pass filtered)
  double _qdot_filtered[NOF][NOJ]; ///< joint velocity (radian/sec, low pass filtered)

  double _J[NOF][3][NOJ]; ///< Jacobian
  double _G[NOF][NOJ];    ///< gravitational vector

  double _J_p[NOF][3][NOJ]; ///< Jacobian. from pinocchio.

  // -- OUTPUT TORQUE ----
  double _tau_des[NOF][NOJ]; ///< desired joint torque

  //
  double _envelop_torque_scalar;

  //
  LowpassFilter _q_filters[NOF][NOJ];
  LowpassFilter _qdot_filters[NOF][NOJ];

#if 0
  // -- Debug Jocobian ----
  double S1_C[NOJ], S2_C[NOJ], S3_C[NOJ], S4_C[NOJ], S23_C[NOJ], S234_C[NOJ], S34_C[NOJ];
  double C1_C[NOJ], C2_C[NOJ], C3_C[NOJ], C4_C[NOJ], C23_C[NOJ], C234_C[NOJ], C34_C[NOJ];
  void LegacyCalculateJacobian(); // For Debugging
#endif

  void LegacySetGains(AllegroHandGrasp::MotionType motion_type);

  void LegacyMotion_HomePosition();
  void LegacyMotion_ReadyToMove();
  void LegacyMotion_Ready();
  void LegacyMotion_GravityComp();
  void LegacyMotion_Grasp3();
  void LegacyMotion_Grasp4();
  void LegacyMotion_PinchIT();
  void LegacyMotion_PinchMT();
  void LegacyMotion_Envelop();

  bool set_gain_table(std::string gain_yaml) override {
    (void)gain_yaml;
    return false;
  }

  bool get_gain_table(std::string& gain_yaml) override {
    (void)gain_yaml;
    return false;
  }

  bool set_gain_table_item(AllegroHandGrasp::MotionType motion_type, std::string gain_yaml) override {
    // nothing to do
    (void)gain_yaml;
    (void)motion_type;
    return false;
  }

  bool get_gain_table_item(AllegroHandGrasp::MotionType motion_type, std::string& gain_yaml) override {
    (void)motion_type;
    (void)gain_yaml;
    return false;
  }

  void set_envelop_torque(double envelop_torque) override { _envelop_torque_scalar = envelop_torque; }

  void update(double dt) override {
    _current_time += dt;

    // Legacy Finger Sequence
    std::vector<int> conv_idx = {
        INDEX,
        MIDDLE,
        RING,
        THUMB,
    };

    // Copy current state
    // l_finger_idx := legacy finger index. [INDEX, MIDDLE, RING, THUMB]
    for (int l_finger_idx = 0; l_finger_idx < NOF; l_finger_idx++) {
      const auto& joint_context = this->finger_io_contexts_.at(conv_idx[l_finger_idx]);

      _x[l_finger_idx] = joint_context->state.fpos.x();
      _y[l_finger_idx] = joint_context->state.fpos.y();
      _z[l_finger_idx] = joint_context->state.fpos.z();

      auto J_pinv = joint_context->state.jacobian.completeOrthogonalDecomposition().pseudoInverse();

      for (int joint_idx = 0; joint_idx < NOJ; joint_idx++) {

#if 1
        _q_filtered[l_finger_idx][joint_idx] = joint_context->state.q[joint_idx];
        _qdot_filtered[l_finger_idx][joint_idx] = joint_context->state.q_dot[joint_idx];
#else
        // 굳이 필요 없음
        _q_filtered[l_finger_idx][joint_idx] = _q_filters[l_finger_idx][joint_idx].filtering(joint_context->state.q[joint_idx]);
        _qdot_filtered[l_finger_idx][joint_idx] = _qdot_filters[l_finger_idx][joint_idx].filtering(joint_context->state.q_dot[joint_idx]);
#endif

        for (int dim_idx = 0; dim_idx < 3; dim_idx++) { // (x, y, z)
          auto j = joint_context->state.jacobian(dim_idx, joint_idx);
          _J[l_finger_idx][dim_idx][joint_idx] = j;
          _J_p[l_finger_idx][dim_idx][joint_idx] = j;
        }
      }
    } // for (int l_finger_idx = 0; l_finger_idx < NOF; l_finger_idx++)

#if 0
    /*
     * For Jacobian Debugging
     * Legacy Code 의 Jacobian 을 적용하고 joint_context 의 값과 비교함. 
     */
    {
      for (int l_finger_idx = 0; l_finger_idx < NOF; l_finger_idx++) {
        const auto& _q = _q_filtered;
        const auto& i = l_finger_idx;
        S1_C[i] = sin(_q[i][0]);
        C1_C[i] = cos(_q[i][0]);
        S2_C[i] = sin(_q[i][1]);
        C2_C[i] = cos(_q[i][1]);
        S3_C[i] = sin(_q[i][2]);
        C3_C[i] = cos(_q[i][2]);
        S4_C[i] = sin(_q[i][3]);
        C4_C[i] = cos(_q[i][3]);
        S23_C[i] = sin(_q[i][1] + _q[i][2]);
        C23_C[i] = cos(_q[i][1] + _q[i][2]);
        S34_C[i] = sin(_q[i][2] + _q[i][3]);
        C34_C[i] = cos(_q[i][2] + _q[i][3]);
        S234_C[i] = sin(_q[i][1] + _q[i][2] + _q[i][3]);
        C234_C[i] = cos(_q[i][1] + _q[i][2] + _q[i][3]);
      }

      LegacyCalculateJacobian();

      for (int l_finger_idx = 0; l_finger_idx < NOF; l_finger_idx++) {
        Eigen::MatrixXd J_l(3, 4); // legacy
        Eigen::MatrixXd J_p(3, 4); // legacy
        for (int joint_idx = 0; joint_idx < NOJ; joint_idx++) {
          for (int dim_idx = 0; dim_idx < 3; dim_idx++) {
            J_l(dim_idx, joint_idx) = _J[l_finger_idx][dim_idx][joint_idx];
            J_p(dim_idx, joint_idx) = _J_p[l_finger_idx][dim_idx][joint_idx];
          }
        }

        SPDLOG_TRACE("J_l[{}]\n{}", l_finger_idx, util::mat2str(J_l));
        SPDLOG_TRACE("J_p[{}]\n{}", l_finger_idx, util::mat2str(J_p));
      } 
    }
#endif

    memset(_G, 0, sizeof(_G));

    LegacyUpdate();

    // Set Output Torque
    for (int l_finger_idx = 0; l_finger_idx < NOF; l_finger_idx++) {
      auto& joint_context = this->finger_io_contexts_.at(conv_idx[l_finger_idx]);
      for (int joint_idx = 0; joint_idx < NOJ; joint_idx++) {
        joint_context->command.torque(joint_idx) = _tau_des[l_finger_idx][joint_idx];
      }
    }
  } // update

  bool set_motion(AllegroHandGrasp::MotionType motion_type) override {
    _current_time = 0;

    switch (motion_type) {
    case AllegroHandGrasp::MotionType::MOTION_TYPE_NONE:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_HOME:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_READY:
    // case AllegroHandGrasp::MotionType::MOTION_TYPE_GRAVITY_COMP:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT:
    // case AllegroHandGrasp::MotionType::MOTION_TYPE_OBJECT_MOVING:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP:
    // case AllegroHandGrasp::MotionType::MOTION_TYPE_JOINT_PD:
    case AllegroHandGrasp::MotionType::MOTION_TYPE_MOVE_OBJ:
      // case AllegroHandGrasp::MotionType::MOTION_TYPE_FINGERTIP_MOVING:
      break;
    default:
      return false;
    }

    motion_type_ = motion_type;
    LegacySetGains(motion_type_);
    return true;
  }

  void LegacyUpdate() {

    bool _tau_des_set = false;

    switch (motion_type_) {
    case AllegroHandGrasp::MotionType::MOTION_TYPE_NONE:
      memset(_tau_des, 0, sizeof(_tau_des));
      _tau_des_set = true;
      break;
    case AllegroHandGrasp::MotionType::MOTION_TYPE_HOME:
      LegacyMotion_HomePosition();
      _tau_des_set = true;
      break;
    case AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP:
      LegacyMotion_Envelop();
      _tau_des_set = true;
      break;
    default:
      // nothing todo
      break;
    }

    if (_tau_des_set) {
      return;
    }

    switch (motion_type_) {
    case AllegroHandGrasp::MotionType::MOTION_TYPE_READY:
      LegacyMotion_Ready();
      break;
    case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3:
      LegacyMotion_Grasp3();
      break;
    case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4:
      LegacyMotion_Grasp4();
      break;
    case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT:
      LegacyMotion_PinchIT();
      break;
    case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT:
      LegacyMotion_PinchMT();
      break;
    // case AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE:
    //   break;
    // case AllegroHandGrasp::MotionType::MOTION_TYPE_MOVE_OBJ:
    //   break;
    // case AllegroHandGrasp::MotionType::MOTION_TYPE_GRAVITY_COMP:
    //   break;
    // case AllegroHandGrasp::MotionType::MOTION_TYPE_OBJECT_MOVING:
    //   break;
    // case AllegroHandGrasp::MotionType::MOTION_TYPE_FINGERTIP_MOVING:
    //   break;
    default:
      memset(_tau_des, 0, sizeof(_tau_des));
      return;
    }

    calc_torque();
  } // update

  void calc_torque() {
    memset(_tau_des, 0, sizeof(_tau_des));

    int i = 0;
    double o = 0.5f;
    double c = 1.0f;

    double fv[NOF][NOJ];
    double fc[NOF][NOJ];

    double Scalef = 800.0f;

    double e_x[4];
    double e_y[4];
    double e_z[4];

    double e_x_object;
    double e_y_object;
    double e_z_object;

    double xc_object;
    double yc_object;
    double zc_object;

    double x_d_object;
    double y_d_object;
    double z_d_object;

    double t_JointDamping[NOF][NOJ];
    double t_TaskDamping[NOF][NOJ];
    double t_Friction[NOF][NOJ];
    double t_Position[NOF][NOJ];
    double t_Pinching[NOF][NOJ];
    double t_Position_Object[NOF][NOJ];
    double t_Gravity[NOF][NOJ];

    ////////// ALLEGRO HAND 2.1 ////////////////////
    // coulomb friction coefficient
    // Tune these for gravity compensation in Allegro Hand 2.0
    fc[0][0] = fc[1][0] = fc[2][0] = 2.0f / Scalef * o;
    fc[0][1] = fc[1][1] = fc[2][1] = 30.0f / Scalef * o;
    fc[0][2] = fc[1][2] = fc[2][2] = 20.0f / Scalef * o;
    fc[0][3] = fc[1][3] = fc[2][3] = 10.0f / Scalef * o; // maybe make this higher for all?

    fc[3][0] = 0.0f / Scalef * o;
    fc[3][1] = 30.0f / Scalef * o;
    fc[3][2] = 30.0f / Scalef * o;
    fc[3][3] = 30.0f / Scalef * o;

    ////////// ALLEGRO HAND 2.1 ////////////////////
    // viscous friction coefficient
    fv[0][0] = fv[1][0] = fv[2][0] = 2.0f / Scalef * c; // 20.0f / Scalef * c;
    fv[0][1] = fv[1][1] = fv[2][1] = 3.0f / Scalef * c;
    fv[0][2] = fv[1][2] = fv[2][2] = 3.0f / Scalef * c;
    fv[0][3] = fv[1][3] = fv[2][3] = 0.5f / Scalef * c;

    fv[3][0] = 2.0f / Scalef * c;
    fv[3][1] = 1.0f / Scalef * c;
    fv[3][2] = 2.0f / Scalef * c;
    fv[3][3] = 2.0f / Scalef * c;
    ///////////////////////////////////////////

    //
    xc_object = (_x[0] + _x[1] + _x[3]) / 3;
    yc_object = (_y[0] + _y[1] + _y[3]) / 3;
    zc_object = (_z[0] + _z[1] + _z[3]) / 3;

    x_d_object = 0.0f;
    y_d_object = 0.0f; // 10.0f*sinf(5*_current_time);
    z_d_object = 0.0f;

    e_x_object = x_d_object - xc_object;
    e_y_object = y_d_object - yc_object;
    e_z_object = z_d_object - zc_object;

    for (i = 0; i < NOF; i++) {
      e_x[i] = (_x_des[i] - _x[i]);
      e_y[i] = (_y_des[i] - _y[i]);
      e_z[i] = (_z_des[i] - _z[i]);

      ///////////////////////////////////////////
      // joint damping
      t_JointDamping[i][0] = 1.0f * _kd[i][0] * _qdot_filtered[i][0];
      t_JointDamping[i][1] = 1.0f * _kd[i][1] * _qdot_filtered[i][1];
      t_JointDamping[i][2] = 1.0f * _kd[i][2] * _qdot_filtered[i][2];
      t_JointDamping[i][3] = 1.0f * _kd[i][3] * _qdot_filtered[i][3];

      ///////////////////////////////////////////
      // task damping
      t_TaskDamping[i][0] = 0;
      t_TaskDamping[i][1] = 0;
      t_TaskDamping[i][2] = 0;
      t_TaskDamping[i][3] = 0;

      ///////////////////////////////////////////
      // friction
      t_Friction[i][0] = (fv[i][0] * _qdot_filtered[i][0]) + (fc[i][0] * (double)tanh(_qdot_filtered[i][0] * 50.0f));
      t_Friction[i][1] = (fv[i][1] * _qdot_filtered[i][1]) + (fc[i][1] * (double)tanh(_qdot_filtered[i][1] * 40.0f));
      t_Friction[i][2] = (fv[i][2] * _qdot_filtered[i][2]) + (fc[i][2] * (double)tanh(_qdot_filtered[i][2] * 50.0f));
      t_Friction[i][3] = (fv[i][3] * _qdot_filtered[i][3]) + (fc[i][3] * (double)tanh(_qdot_filtered[i][3] * 10.0f));

      ///////////////////////////////////////////
      // task position
      t_Position[i][0] = (_J[i][0][0] * _kp[i][0] * (e_x[i]) + _J[i][1][0] * _kp[i][0] * (e_y[i]) + _J[i][2][0] * _kp[i][0] * (e_z[i]));
      t_Position[i][1] = (_J[i][0][1] * _kp[i][1] * (e_x[i]) + _J[i][1][1] * _kp[i][1] * (e_y[i]) + _J[i][2][1] * _kp[i][1] * (e_z[i]));
      t_Position[i][2] = (_J[i][0][2] * _kp[i][2] * (e_x[i]) + _J[i][1][2] * _kp[i][2] * (e_y[i]) + _J[i][2][2] * _kp[i][2] * (e_z[i]));
      t_Position[i][3] = (_J[i][0][3] * _kp[i][3] * (e_x[i]) + _J[i][1][3] * _kp[i][3] * (e_y[i]) + _J[i][2][3] * _kp[i][3] * (e_z[i]));

      ///////////////////////////////////////////
      // desired pinching force
      t_Pinching[i][0] = (_J[i][0][0] * _f_des[i] * (e_x[i]) + _J[i][1][0] * _f_des[i] * (e_y[i]) + _J[i][2][0] * _f_des[i] * (e_z[i]));
      t_Pinching[i][1] = (_J[i][0][1] * _f_des[i] * (e_x[i]) + _J[i][1][1] * _f_des[i] * (e_y[i]) + _J[i][2][1] * _f_des[i] * (e_z[i]));
      t_Pinching[i][2] = (_J[i][0][2] * _f_des[i] * (e_x[i]) + _J[i][1][2] * _f_des[i] * (e_y[i]) + _J[i][2][2] * _f_des[i] * (e_z[i]));
      t_Pinching[i][3] = (_J[i][0][3] * _f_des[i] * (e_x[i]) + _J[i][1][3] * _f_des[i] * (e_y[i]) + _J[i][2][3] * _f_des[i] * (e_z[i]));

      ///////////////////////////////////////////
      // object position
      t_Position_Object[i][0] = (_J[i][0][0] * _kp_task[i][0] * (e_x_object) + _J[i][1][0] * _kp_task[i][0] * (e_y_object) +
                                 _J[i][2][0] * _kp_task[i][0] * (e_z_object));
      t_Position_Object[i][1] = (_J[i][0][1] * _kp_task[i][1] * (e_x_object) + _J[i][1][1] * _kp_task[i][1] * (e_y_object) +
                                 _J[i][2][1] * _kp_task[i][1] * (e_z_object));
      t_Position_Object[i][2] = (_J[i][0][2] * _kp_task[i][2] * (e_x_object) + _J[i][1][2] * _kp_task[i][2] * (e_y_object) +
                                 _J[i][2][2] * _kp_task[i][2] * (e_z_object));
      t_Position_Object[i][3] = (_J[i][0][3] * _kp_task[i][3] * (e_x_object) + _J[i][1][3] * _kp_task[i][3] * (e_y_object) +
                                 _J[i][2][3] * _kp_task[i][3] * (e_z_object));

      ///////////////////////////////////////////
      // gravity
      t_Gravity[i][0] = _G[i][0];
      t_Gravity[i][1] = _G[i][1];
      t_Gravity[i][2] = _G[i][2];
      t_Gravity[i][3] = _G[i][3];

      ///////////////////////////////////////////
      // total
      _tau_des[i][0] = ((t_Position[i][0] + t_Position_Object[i][0] - t_TaskDamping[i][0] - t_JointDamping[i][0] + t_Pinching[i][0] +
                         t_Gravity[i][0] + t_Friction[i][0]) *
                        (1));
      _tau_des[i][1] = ((t_Position[i][1] + t_Position_Object[i][1] - t_TaskDamping[i][1] - t_JointDamping[i][1] + t_Pinching[i][1] +
                         t_Gravity[i][1] + t_Friction[i][1]) *
                        (1));
      _tau_des[i][2] = ((t_Position[i][2] + t_Position_Object[i][2] - t_TaskDamping[i][2] - t_JointDamping[i][2] + t_Pinching[i][2] +
                         t_Gravity[i][2] + t_Friction[i][2]) *
                        (1));
      _tau_des[i][3] = ((t_Position[i][3] + t_Position_Object[i][3] - t_TaskDamping[i][3] - t_JointDamping[i][3] + t_Pinching[i][3] +
                         t_Gravity[i][3] + t_Friction[i][3]) *
                        (1));
    }
  }

public:
  explicit GraspAlgorithmV4LegacyReference(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts, HandSide hand_side)
      : GraspAlgorithm(finger_io_contexts, hand_side) {
    _current_time = 0;
    _envelop_torque_scalar = 1.0;
  }
  virtual ~GraspAlgorithmV4LegacyReference() = default;
}; // class GraspAlgorithmV4LegacyReference

/*
 *
 */
void GraspAlgorithmV4LegacyReference::LegacySetGains(AllegroHandGrasp::MotionType motion_type) {

  switch (motion_type) {
  case AllegroHandGrasp::MotionType::MOTION_TYPE_HOME:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_ENVELOP:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_JOINT_PD: {

    // ALLEGRO HAND 3.0
    // Finger 1,2 ,3 (2014 01 11)
    _kp[0][0] = _kp[1][0] = _kp[2][0] = 500; // 500 40
    _kp[0][1] = _kp[1][1] = _kp[2][1] = 800;
    _kp[0][2] = _kp[1][2] = _kp[2][2] = 900;
    _kp[0][3] = _kp[1][3] = _kp[2][3] = 500;
    _kd[0][0] = _kd[1][0] = _kd[2][0] = 25; // this finger has really low friction at the first joint on BR014
    _kd[0][1] = _kd[1][1] = _kd[2][1] = 50; // make sure these values look good on all hands. likely change them
    _kd[0][2] = _kd[1][2] = _kd[2][2] = 55; // the to match the ones below.
    _kd[0][3] = _kd[1][3] = _kd[2][3] = 40;

    // Finger 4
    _kp[3][0] = 1000;
    _kp[3][1] = 700;
    _kp[3][2] = 600;
    _kp[3][3] = 600;
    _kd[3][0] = 50;
    _kd[3][1] = 50;
    _kd[3][2] = 50;
    _kd[3][3] = 40;

    // Added by Alex to null force for Arrow display
    _f_des[0] = 0;
    _f_des[1] = 0;
    _f_des[2] = 0;
    _f_des[3] = 0;
  } break;

  case AllegroHandGrasp::MotionType::MOTION_TYPE_MOVE_OBJ:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_GRAVITY_COMP:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_OBJECT_MOVING:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_FINGERTIP_MOVING:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_READY: {

    // ALLEGRO HAND 3.0
    // Fingers 1,2 ,3 (2014 01 10)
    _kp[0][0] = _kp[1][0] = _kp[2][0] = 80; // was 120 orignially
    _kp[0][1] = _kp[1][1] = _kp[2][1] = 90;
    _kp[0][2] = _kp[1][2] = _kp[2][2] = 90;
    _kp[0][3] = _kp[1][3] = _kp[2][3] = 80;
    _kd[0][0] = _kd[1][0] = _kd[2][0] = std::sqrt(_kp[0][0]) * 0.004 * 1.0;
    _kd[0][1] = _kd[1][1] = _kd[2][1] = std::sqrt(_kp[0][1]) * 0.004 * 2.0;
    _kd[0][2] = _kd[1][2] = _kd[2][2] = std::sqrt(_kp[0][2]) * 0.004 * 2.0;
    _kd[0][3] = _kd[1][3] = _kd[2][3] = std::sqrt(_kp[0][3]) * 0.004 * 1.0;
    _kp_task[0][0] = _kp_task[1][0] = _kp_task[2][0] = 0;
    _kp_task[0][1] = _kp_task[1][1] = _kp_task[2][1] = 0;
    _kp_task[0][2] = _kp_task[1][2] = _kp_task[2][2] = 0;
    _kp_task[0][3] = _kp_task[1][3] = _kp_task[2][3] = 0;
    _kd_task[0][0] = _kd_task[1][0] = _kd_task[2][0] = 0;
    _kd_task[0][1] = _kd_task[1][1] = _kd_task[2][1] = 0;
    _kd_task[0][2] = _kd_task[1][2] = _kd_task[2][2] = 0;
    _kd_task[0][3] = _kd_task[1][3] = _kd_task[2][3] = 0;

    // Finger 4
    _kp[3][0] = 90; // 120;
    _kp[3][1] = 90; // 120;
    _kp[3][2] = 90; // 120;
    _kp[3][3] = 90; // 120;
    _kd[3][0] = std::sqrt(_kp[3][0]) * 0.004 * 4.0;
    _kd[3][1] = std::sqrt(_kp[3][1]) * 0.004 * 2.0;
    _kd[3][2] = std::sqrt(_kp[3][2]) * 0.004 * 1.0;
    _kd[3][3] = std::sqrt(_kp[3][3]) * 0.004 * 1.0;

    _kp_task[3][0] = 0;
    _kp_task[3][1] = 0;
    _kp_task[3][2] = 0;
    _kp_task[3][3] = 0;
    _kd_task[3][0] = 0;
    _kd_task[3][1] = 0;
    _kd_task[3][2] = 0;
    _kd_task[3][3] = 0;

    // Etc
    _f_des[0] = 0;
    _f_des[1] = 0;
    _f_des[2] = 0;
    _f_des[3] = 0;
  } break;

  case AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3:
  case AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4: {
    _f_des[0] = (AllegroHandGrasp::MotionType::MOTION_TYPE_PRE_SHAPE == motion_type ? 40.0 : 4.0);
    _f_des[1] = 3.0;
    _f_des[2] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_4 == motion_type ? 3.0 : 0.0);
    _f_des[3] = 3.0;

    // Finger 1
    _kp[0][0] = 0;
    _kp[0][1] = 0;
    _kp[0][2] = 0;
    _kp[0][3] = 0;
    _kd[0][0] = std::sqrt(_f_des[0]) * 0.01;
    _kd[0][1] = std::sqrt(_f_des[0]) * 0.01;
    _kd[0][2] = std::sqrt(_f_des[0]) * 0.01;
    _kd[0][3] = std::sqrt(_f_des[0]) * 0.01;
    _kp_task[0][0] = 0;
    _kp_task[0][1] = 0;
    _kp_task[0][2] = 0;
    _kp_task[0][3] = 0;
    _kd_task[0][0] = std::sqrt(_f_des[0]) * 0.0;
    _kd_task[0][1] = std::sqrt(_f_des[0]) * 0.0;
    _kd_task[0][2] = std::sqrt(_f_des[0]) * 0.0;
    _kd_task[0][3] = std::sqrt(_f_des[0]) * 0.0;

    // Finger 2
    _kp[1][0] = 0;
    _kp[1][1] = 0;
    _kp[1][2] = 0;
    _kp[1][3] = 0;
    _kd[1][0] = std::sqrt(_f_des[1]) * 0.01;
    _kd[1][1] = std::sqrt(_f_des[1]) * 0.01;
    _kd[1][2] = std::sqrt(_f_des[1]) * 0.01;
    _kd[1][3] = std::sqrt(_f_des[1]) * 0.01;
    _kp_task[1][0] = 0;
    _kp_task[1][1] = 0;
    _kp_task[1][2] = 0;
    _kp_task[1][3] = 0;
    _kd_task[1][0] = std::sqrt(_f_des[1]) * 0.0;
    _kd_task[1][1] = std::sqrt(_f_des[1]) * 0.0;
    _kd_task[1][2] = std::sqrt(_f_des[1]) * 0.0;
    _kd_task[1][3] = std::sqrt(_f_des[1]) * 0.0;

    // Finger 3
    _kp[2][0] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? 120.0 : 0.0);
    _kp[2][1] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? 120.0 : 0.0);
    _kp[2][2] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? 120.0 : 0.0);
    _kp[2][3] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? 120.0 : 0.0);
    _kd[2][0] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? std::sqrt(_kp[2][0]) * 0.005 * 1.0
                                                                                  : std::sqrt(_f_des[2]) * 0.01);
    _kd[2][1] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? std::sqrt(_kp[2][1]) * 0.005 * 3.0
                                                                                  : std::sqrt(_f_des[2]) * 0.01);
    _kd[2][2] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? std::sqrt(_kp[2][2]) * 0.005 * 2.0
                                                                                  : std::sqrt(_f_des[2]) * 0.01);
    _kd[2][3] = (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? std::sqrt(_kp[2][3]) * 0.005 * 1.0
                                                                                  : std::sqrt(_f_des[2]) * 0.01);
    _kp_task[2][0] = 0;
    _kp_task[2][1] = 0;
    _kp_task[2][2] = 0;
    _kp_task[2][3] = 0;
    _kd_task[2][0] =
        (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? std::sqrt(_kp[2][0]) * 0.04 : std::sqrt(_f_des[2]) * 0.0);
    _kd_task[2][1] =
        (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? std::sqrt(_kp[2][1]) * 0.04 : std::sqrt(_f_des[2]) * 0.0);
    _kd_task[2][2] =
        (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? std::sqrt(_kp[2][2]) * 0.04 : std::sqrt(_f_des[2]) * 0.0);
    _kd_task[2][3] =
        (AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3 == motion_type ? std::sqrt(_kp[2][3]) * 0.04 : std::sqrt(_f_des[2]) * 0.0);

    // Finger 4
    _kp[3][0] = 0;
    _kp[3][1] = 0;
    _kp[3][2] = 0;
    _kp[3][3] = 0;
    _kd[3][0] = std::sqrt(_f_des[3]) * 0.03;
    _kd[3][1] = std::sqrt(_f_des[3]) * 0.03;
    _kd[3][2] = std::sqrt(_f_des[3]) * 0.001;
    _kd[3][3] = std::sqrt(_f_des[3]) * 0.001;
    _kp_task[3][0] = 0;
    _kp_task[3][1] = 0;
    _kp_task[3][2] = 0;
    _kp_task[3][3] = 0;
    _kd_task[3][0] = std::sqrt(_f_des[3]) * 0.0;
    _kd_task[3][1] = std::sqrt(_f_des[3]) * 0.0;
    _kd_task[3][2] = std::sqrt(_f_des[3]) * 0.0;
    _kd_task[3][3] = std::sqrt(_f_des[3]) * 0.0;
  } break;

  case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_IT: {
    _x_des[1] = _x[1] - 0.01; // set desired position for middle finger
    _y_des[1] = _y[1];
    _z_des[1] = _z[1] + 0.02;

    _f_des[0] = 5.0;
    _f_des[1] = 0;
    _f_des[2] = 0;
    _f_des[3] = 4;

    _kp[0][0] = 0;
    _kp[0][1] = 0;
    _kp[0][2] = 0;
    _kp[0][3] = 0;

    _kp[1][0] = 120;
    _kp[1][1] = 120;
    _kp[1][2] = 120;
    _kp[1][3] = 120;

    _kp[2][0] = 120;
    _kp[2][1] = 120;
    _kp[2][2] = 120;
    _kp[2][3] = 120;

    _kp[3][0] = 0;
    _kp[3][1] = 0;
    _kp[3][2] = 0;
    _kp[3][3] = 0;

    _kp_task[0][0] = 0;
    _kp_task[0][1] = 0;
    _kp_task[0][2] = 0;
    _kp_task[0][3] = 0;

    _kp_task[1][0] = 0;
    _kp_task[1][1] = 0;
    _kp_task[1][2] = 0;
    _kp_task[1][3] = 0;

    _kp_task[2][0] = 0;
    _kp_task[2][1] = 0;
    _kp_task[2][2] = 0;
    _kp_task[2][3] = 0;

    _kp_task[3][0] = 0;
    _kp_task[3][1] = 0;
    _kp_task[3][2] = 0;
    _kp_task[3][3] = 0;

    _kd[0][0] = std::sqrt(_f_des[0]) * 0.005f;
    _kd[0][1] = std::sqrt(_f_des[0]) * 0.005f;
    _kd[0][2] = std::sqrt(_f_des[0]) * 0.005f;
    _kd[0][3] = std::sqrt(_f_des[0]) * 0.005f;

    _kd[1][0] = std::sqrt(_kp[1][0]) * 0.005f * 1.0f;
    _kd[1][1] = std::sqrt(_kp[1][1]) * 0.005f * 3.0f;
    _kd[1][2] = std::sqrt(_kp[1][2]) * 0.005f * 2.0f;
    _kd[1][3] = std::sqrt(_kp[1][3]) * 0.005f * 1.0f;

    _kd[2][0] = std::sqrt(_kp[2][0]) * 0.005f * 1.0f;
    _kd[2][1] = std::sqrt(_kp[2][1]) * 0.005f * 3.0f;
    _kd[2][2] = std::sqrt(_kp[2][2]) * 0.005f * 2.0f;
    _kd[2][3] = std::sqrt(_kp[2][3]) * 0.005f * 1.0f;

    _kd[3][0] = std::sqrt(_f_des[3]) * 0.03f;
    _kd[3][1] = std::sqrt(_f_des[3]) * 0.03f;
    _kd[3][2] = std::sqrt(_f_des[3]) * 0.03f;
    _kd[3][3] = std::sqrt(_f_des[3]) * 0.03f;

    _kd_task[0][0] = std::sqrt(_f_des[0]) * 0.001f;
    _kd_task[0][1] = std::sqrt(_f_des[0]) * 0.001f;
    _kd_task[0][2] = std::sqrt(_f_des[0]) * 0.001f;
    _kd_task[0][3] = std::sqrt(_f_des[0]) * 0.001f;

    _kd_task[1][0] = std::sqrt(_kp[1][0]) * 0.04f;
    _kd_task[1][1] = std::sqrt(_kp[1][1]) * 0.04f;
    _kd_task[1][2] = std::sqrt(_kp[1][2]) * 0.04f;
    _kd_task[1][3] = std::sqrt(_kp[1][3]) * 0.04f;

    _kd_task[2][0] = std::sqrt(_kp[2][0]) * 0.04f;
    _kd_task[2][1] = std::sqrt(_kp[2][1]) * 0.04f;
    _kd_task[2][2] = std::sqrt(_kp[2][2]) * 0.04f;
    _kd_task[2][3] = std::sqrt(_kp[2][3]) * 0.04f;

    _kd_task[3][0] = std::sqrt(_f_des[3]) * 0.001f;
    _kd_task[3][1] = std::sqrt(_f_des[3]) * 0.001f;
    _kd_task[3][2] = std::sqrt(_f_des[3]) * 0.001f;
    _kd_task[3][3] = std::sqrt(_f_des[3]) * 0.001f;
  } break;

  case AllegroHandGrasp::MotionType::MOTION_TYPE_PINCH_MT: {
    _x_des[0] = _x[0] - 0.01; // set desired position for index finger
    _y_des[0] = _y[0];
    _z_des[0] = _z[0] + 0.02;

    _f_des[0] = 0;
    _f_des[1] = 5.0;
    _f_des[2] = 0;
    _f_des[3] = 4.0;

    _kp[0][0] = 120;
    _kp[0][1] = 120;
    _kp[0][2] = 120;
    _kp[0][3] = 120;

    _kp[1][0] = 0;
    _kp[1][1] = 0;
    _kp[1][2] = 0;
    _kp[1][3] = 0;

    _kp[2][0] = 120;
    _kp[2][1] = 120;
    _kp[2][2] = 120;
    _kp[2][3] = 120;

    _kp[3][0] = 0;
    _kp[3][1] = 0;
    _kp[3][2] = 0;
    _kp[3][3] = 0;

    _kp_task[0][0] = 0;
    _kp_task[0][1] = 0;
    _kp_task[0][2] = 0;
    _kp_task[0][3] = 0;

    _kp_task[1][0] = 0;
    _kp_task[1][1] = 0;
    _kp_task[1][2] = 0;
    _kp_task[1][3] = 0;

    _kp_task[2][0] = 0;
    _kp_task[2][1] = 0;
    _kp_task[2][2] = 0;
    _kp_task[2][3] = 0;

    _kp_task[3][0] = 0;
    _kp_task[3][1] = 0;
    _kp_task[3][2] = 0;
    _kp_task[3][3] = 0;

    _kd[0][0] = std::sqrt(_kp[0][0]) * 0.005f * 1.0f;
    _kd[0][1] = std::sqrt(_kp[0][1]) * 0.005f * 3.0f;
    _kd[0][2] = std::sqrt(_kp[0][2]) * 0.005f * 2.0f;
    _kd[0][3] = std::sqrt(_kp[0][3]) * 0.005f * 1.0f;

    _kd[1][0] = std::sqrt(_f_des[1]) * 0.005f;
    _kd[1][1] = std::sqrt(_f_des[1]) * 0.005f;
    _kd[1][2] = std::sqrt(_f_des[1]) * 0.005f;
    _kd[1][3] = std::sqrt(_f_des[1]) * 0.005f;

    _kd[2][0] = std::sqrt(_kp[2][0]) * 0.005f * 1.0f;
    _kd[2][1] = std::sqrt(_kp[2][1]) * 0.005f * 3.0f;
    _kd[2][2] = std::sqrt(_kp[2][2]) * 0.005f * 2.0f;
    _kd[2][3] = std::sqrt(_kp[2][3]) * 0.005f * 1.0f;

    _kd[3][0] = std::sqrt(_f_des[3]) * 0.03f;
    _kd[3][1] = std::sqrt(_f_des[3]) * 0.03f;
    _kd[3][2] = std::sqrt(_f_des[3]) * 0.03f;
    _kd[3][3] = std::sqrt(_f_des[3]) * 0.03f;

    _kd_task[0][0] = std::sqrt(_kp[0][0]) * 0.04f;
    _kd_task[0][1] = std::sqrt(_kp[0][1]) * 0.04f;
    _kd_task[0][2] = std::sqrt(_kp[0][2]) * 0.04f;
    _kd_task[0][3] = std::sqrt(_kp[0][3]) * 0.04f;

    _kd_task[1][0] = std::sqrt(_f_des[1]) * 0.001f;
    _kd_task[1][1] = std::sqrt(_f_des[1]) * 0.001f;
    _kd_task[1][2] = std::sqrt(_f_des[1]) * 0.001f;
    _kd_task[1][3] = std::sqrt(_f_des[1]) * 0.001f;

    _kd_task[2][0] = std::sqrt(_kp[2][0]) * 0.04f;
    _kd_task[2][1] = std::sqrt(_kp[2][1]) * 0.04f;
    _kd_task[2][2] = std::sqrt(_kp[2][2]) * 0.04f;
    _kd_task[2][3] = std::sqrt(_kp[2][3]) * 0.04f;

    _kd_task[3][0] = std::sqrt(_f_des[3]) * 0.001f;
    _kd_task[3][1] = std::sqrt(_f_des[3]) * 0.001f;
    _kd_task[3][2] = std::sqrt(_f_des[3]) * 0.001f;
    _kd_task[3][3] = std::sqrt(_f_des[3]) * 0.001f;
  } break;

  case AllegroHandGrasp::MotionType::MOTION_TYPE_NONE:
  default: {
    memset(_kp, 0, sizeof(_kp));
    memset(_kd, 0, sizeof(_kd));
    memset(_kp_task, 0, sizeof(_kp_task));
    memset(_kd_task, 0, sizeof(_kd_task));
  } break;
  }
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_HomePosition() {
  static double q_home_left[NOF][NOJ] = {{0 * DEG2RAD, -10 * DEG2RAD, 45 * DEG2RAD, 45 * DEG2RAD},
                                         {0 * DEG2RAD, -10 * DEG2RAD, 45 * DEG2RAD, 45 * DEG2RAD},
                                         {-5 * DEG2RAD, -5 * DEG2RAD, 50 * DEG2RAD, 45 * DEG2RAD},
                                         {50 * DEG2RAD, 25 * DEG2RAD, 15 * DEG2RAD, 45 * DEG2RAD}};

  static double q_home_right[NOF][NOJ] = {{0 * DEG2RAD, -10 * DEG2RAD, 45 * DEG2RAD, 45 * DEG2RAD},
                                          {0 * DEG2RAD, -10 * DEG2RAD, 45 * DEG2RAD, 45 * DEG2RAD},
                                          {5 * DEG2RAD, -5 * DEG2RAD, 50 * DEG2RAD, 45 * DEG2RAD},
                                          {50 * DEG2RAD, 25 * DEG2RAD, 15 * DEG2RAD, 45 * DEG2RAD}};

  for (int i = 0; i < NOF; i++) {
    for (int j = 0; j < NOJ; j++) {
      if (hand_side_ == LEFT)
        _tau_des[i][j] = _kp[i][j] * (q_home_left[i][j] - _q_filtered[i][j]) - _kd[i][j] * _qdot_filtered[i][j];
      else
        _tau_des[i][j] = _kp[i][j] * (q_home_right[i][j] - _q_filtered[i][j]) - _kd[i][j] * _qdot_filtered[i][j];
      _tau_des[i][j] /= 800.0; // pwm to torque
    }
  }
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_ReadyToMove() // Same as GraspAlgorithmV4LegacyReference::LegacyMotion_GravityComp()
{
  _x_des[0] = _x[0];
  _y_des[0] = _y[0];
  _z_des[0] = _z[0];

  _x_des[1] = _x[1];
  _y_des[1] = _y[1];
  _z_des[1] = _z[1];

  _x_des[2] = _x[2];
  _y_des[2] = _y[2];
  _z_des[2] = _z[2];

  _x_des[3] = _x[3];
  _y_des[3] = _y[3];
  _z_des[3] = _z[3];
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_Ready() {
  if (hand_side_ == LEFT) {
    _x_des[0] = 0.08;
    _y_des[0] = -0.048;
    _z_des[0] = 0.10;

    _x_des[1] = 0.08;
    _y_des[1] = 0.0;
    _z_des[1] = 0.10;

    _x_des[2] = 0.08;
    _y_des[2] = 0.045;
    _z_des[2] = 0.08;

    _x_des[3] = 0.11;
    _y_des[3] = -0.040;
    _z_des[3] = -0.04;
  } else {
    _x_des[0] = 0.08;
    _y_des[0] = 0.048;
    _z_des[0] = 0.10;

    _x_des[1] = 0.08;
    _y_des[1] = 0.0;
    _z_des[1] = 0.10;

    _x_des[2] = 0.08;
    _y_des[2] = -0.045;
    _z_des[2] = 0.08;

    _x_des[3] = 0.11;
    _y_des[3] = 0.040;
    _z_des[3] = -0.04;
  }
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_GravityComp() {

  _x_des[0] = _x[0];
  _y_des[0] = _y[0];
  _z_des[0] = _z[0];

  _x_des[1] = _x[1];
  _y_des[1] = _y[1];
  _z_des[1] = _z[1];

  _x_des[2] = _x[2];
  _y_des[2] = _y[2];
  _z_des[2] = _z[2];

  _x_des[3] = _x[3];
  _y_des[3] = _y[3];
  _z_des[3] = _z[3];
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_Grasp3() {
  double distance[4];
  double delta_x[4];
  double delta_y[4];
  double delta_z[4];
  double alpha[4];

  double center_x_geo, center_y_geo, center_z_geo;

  alpha[0] = _f_des[0];

  center_x_geo = (_x[0] + _x[1]) / 2.0f;
  center_y_geo = (_y[0] + _y[1]) / 2.0f;
  center_z_geo = (_z[0] + _z[1]) / 2.0f;

  _x_des[0] = _x[3];        // index
  _x_des[1] = _x[3];        // middle
  _x_des[3] = center_x_geo; // thumb

  _y_des[0] = _y[3];        //+y_move;
  _y_des[1] = _y[3];        //+y_move;
  _y_des[3] = center_y_geo; //+y_move;

  _z_des[0] = _z[3]; //+0.01;
  _z_des[1] = _z[3]; //+0.01;
  _z_des[3] = center_z_geo;

  distance[0] = std::sqrt((_x[0] - _x_des[0]) * (_x[0] - _x_des[0]) + (_y[0] - _y_des[0]) * (_y[0] - _y_des[0]) +
                          (_z[0] - _z_des[0]) * (_z[0] - _z_des[0]));
  distance[1] = std::sqrt((_x[1] - _x_des[1]) * (_x[1] - _x_des[1]) + (_y[1] - _y_des[1]) * (_y[1] - _y_des[1]) +
                          (_z[1] - _z_des[1]) * (_z[1] - _z_des[1]));
  distance[3] = std::sqrt((_x[3] - _x_des[3]) * (_x[3] - _x_des[3]) + (_y[3] - _y_des[3]) * (_y[3] - _y_des[3]) +
                          (_z[3] - _z_des[3]) * (_z[3] - _z_des[3]));

  delta_x[0] = (_x_des[0] - _x[0]) / (distance[0]);
  delta_y[0] = (_y_des[0] - _y[0]) / (distance[0]);
  delta_z[0] = (_z_des[0] - _z[0]) / (distance[0]);

  delta_x[1] = (_x_des[1] - _x[1]) / (distance[1]);
  delta_y[1] = (_y_des[1] - _y[1]) / (distance[1]);
  delta_z[1] = (_z_des[1] - _z[1]) / (distance[1]);

  delta_x[3] = (_x_des[3] - _x[3]) / (distance[3]);
  delta_y[3] = (_y_des[3] - _y[3]) / (distance[3]);
  delta_z[3] = (_z_des[3] - _z[3]) / (distance[3]);

  _x_des[0] = _x[0] + delta_x[0];
  _y_des[0] = _y[0] + delta_y[0];
  _z_des[0] = _z[0] + delta_z[0];

  _x_des[1] = _x[1] + delta_x[1];
  _y_des[1] = _y[1] + delta_y[1];
  _z_des[1] = _z[1] + delta_z[1];

  _x_des[3] = _x[3] + delta_x[3];
  _y_des[3] = _y[3] + delta_y[3];
  _z_des[3] = _z[3] + delta_z[3];

  alpha[1] = (-1) * alpha[0] * (delta_x[0] - delta_x[3] * delta_y[0] / delta_y[3]) / (delta_x[1] - delta_x[3] * delta_y[1] / delta_y[3]);
  alpha[3] = (-1) * alpha[0] * delta_z[0] - alpha[1] * delta_z[1] + 0.003f * 9.81f;

  _f_des[0] = alpha[0]; // + 0.03f*sinf(_current_time);
  _f_des[1] = alpha[1]; // + 0.03f*sinf(_current_time);
  _f_des[3] = alpha[3]; // + 0.03f*sinf(_current_time);
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_Grasp4() {
  double distance[4];
  double delta_x[4];
  double delta_y[4];
  double delta_z[4];
  double alpha[4];

  double center_x_geo, center_y_geo, center_z_geo;

  alpha[0] = _f_des[0];

  center_x_geo = (_x[0] + _x[1] + _x[2]) / 3.0f;
  center_y_geo = (_y[0] + _y[1] + _y[2]) / 3.0f;
  center_z_geo = (_z[0] + _z[1] + _z[2]) / 3.0f;

  _x_des[0] = _x[3];
  _x_des[1] = _x[3];
  _x_des[2] = _x[3];
  _x_des[3] = center_x_geo;

  _y_des[0] = _y[3];
  _y_des[1] = _y[3];
  _y_des[2] = _y[3];
  _y_des[3] = center_y_geo;

  _z_des[0] = _z[3] + 0.02;
  _z_des[1] = _z[3] + 0.02;
  _z_des[2] = _z[3] + 0.02;
  _z_des[3] = center_z_geo;

  distance[0] = std::sqrt((_x[0] - _x_des[0]) * (_x[0] - _x_des[0]) + (_y[0] - _y_des[0]) * (_y[0] - _y_des[0]) +
                          (_z[0] - _z_des[0]) * (_z[0] - _z_des[0]));
  distance[1] = std::sqrt((_x[1] - _x_des[1]) * (_x[1] - _x_des[1]) + (_y[1] - _y_des[1]) * (_y[1] - _y_des[1]) +
                          (_z[1] - _z_des[1]) * (_z[1] - _z_des[1]));
  distance[2] = std::sqrt((_x[2] - _x_des[2]) * (_x[2] - _x_des[2]) + (_y[2] - _y_des[2]) * (_y[2] - _y_des[2]) +
                          (_z[2] - _z_des[2]) * (_z[2] - _z_des[2]));
  distance[3] = std::sqrt((_x[3] - _x_des[3]) * (_x[3] - _x_des[3]) + (_y[3] - _y_des[3]) * (_y[3] - _y_des[3]) +
                          (_z[3] - _z_des[3]) * (_z[3] - _z_des[3]));

  delta_x[0] = (_x_des[0] - _x[0]) / (distance[0]);
  delta_y[0] = (_y_des[0] - _y[0]) / (distance[0]);
  delta_z[0] = (_z_des[0] - _z[0]) / (distance[0]);

  delta_x[1] = (_x_des[1] - _x[1]) / (distance[1]);
  delta_y[1] = (_y_des[1] - _y[1]) / (distance[1]);
  delta_z[1] = (_z_des[1] - _z[1]) / (distance[1]);

  delta_x[2] = (_x_des[2] - _x[2]) / (distance[2]);
  delta_y[2] = (_y_des[2] - _y[2]) / (distance[2]);
  delta_z[2] = (_z_des[2] - _z[2]) / (distance[2]);

  delta_x[3] = (_x_des[3] - _x[3]) / (distance[3]);
  delta_y[3] = (_y_des[3] - _y[3]) / (distance[3]);
  delta_z[3] = (_z_des[3] - _z[3]) / (distance[3]);

  _x_des[0] = _x[0] + delta_x[0];
  _y_des[0] = _y[0] + delta_y[0];
  _z_des[0] = _z[0] + delta_z[0];

  _x_des[1] = _x[1] + delta_x[1];
  _y_des[1] = _y[1] + delta_y[1];
  _z_des[1] = _z[1] + delta_z[1];

  _x_des[2] = _x[2] + delta_x[2];
  _y_des[2] = _y[2] + delta_y[2];
  _z_des[2] = _z[2] + delta_z[2];

  _x_des[3] = _x[3] + delta_x[3];
  _y_des[3] = _y[3] + delta_y[3];
  _z_des[3] = _z[3] + delta_z[3];

  alpha[1] = (distance[0] / distance[1]) * alpha[0];
  alpha[2] = alpha[0];

  delta_x[0] = alpha[0] * delta_x[0];
  delta_y[0] = alpha[0] * delta_y[0];
  delta_z[0] = alpha[0] * delta_z[0];

  delta_x[1] = alpha[1] * delta_x[1];
  delta_y[1] = alpha[1] * delta_y[1];
  delta_z[1] = alpha[1] * delta_z[1];

  delta_x[2] = alpha[2] * delta_x[2];
  delta_y[2] = alpha[2] * delta_y[2];
  delta_z[2] = alpha[2] * delta_z[2];

  alpha[3] = std::sqrt((delta_x[0] + delta_x[1] + delta_x[2]) * (delta_x[0] + delta_x[1] + delta_x[2]) +
                       (delta_y[0] + delta_y[1] + delta_y[2]) * (delta_y[0] + delta_y[1] + delta_y[2]) +
                       (delta_z[0] + delta_z[1] + delta_z[2]) * (delta_z[0] + delta_z[1] + delta_z[2]));

  _f_des[1] = alpha[1];
  _f_des[2] = alpha[2];
  _f_des[3] = alpha[3];
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_PinchIT() {
  double distance[4];
  double delta_x[4];
  double delta_y[4];
  double delta_z[4];
  double alpha[4];

  double center_x_geo, center_y_geo, center_z_geo;

  double pinchOffset_y = 0.000; // 0.002;

  alpha[0] = _f_des[0];

  center_x_geo = (_x[0] + _x[3]) / 2.0f;
  center_y_geo = (_y[0] + _y[3]) / 2.0f;
  center_z_geo = (_z[0] + _z[3]) / 2.0f;

  _x_des[0] = center_x_geo;
  _x_des[3] = center_x_geo;

  _y_des[0] = center_y_geo + pinchOffset_y / 2.0;
  _y_des[3] = center_y_geo - pinchOffset_y / 2.0;

  _z_des[0] = center_z_geo;
  _z_des[3] = center_z_geo;

  distance[0] = std::sqrt((_x[0] - _x_des[0]) * (_x[0] - _x_des[0]) + (_y[0] - _y_des[0]) * (_y[0] - _y_des[0]) +
                          (_z[0] - _z_des[0]) * (_z[0] - _z_des[0]));
  distance[3] = std::sqrt((_x[3] - _x_des[3]) * (_x[3] - _x_des[3]) + (_y[3] - _y_des[3]) * (_y[3] - _y_des[3]) +
                          (_z[3] - _z_des[3]) * (_z[3] - _z_des[3]));

  delta_x[0] = (_x_des[0] - _x[0]) / (distance[0]);
  delta_y[0] = (_y_des[0] - _y[0]) / (distance[0]);
  delta_z[0] = (_z_des[0] - _z[0]) / (distance[0]);

  delta_x[3] = (_x_des[3] - _x[3]) / (distance[3]);
  delta_y[3] = (_y_des[3] - _y[3]) / (distance[3]);
  delta_z[3] = (_z_des[3] - _z[3]) / (distance[3]);

  _x_des[0] = _x[0] + delta_x[0];
  _y_des[0] = _y[0] + delta_y[0];
  _z_des[0] = _z[0] + delta_z[0];

  if (hand_side_ == LEFT) {
    _x_des[1] = 0.08f;
    _y_des[1] = 0.0f;
    _z_des[1] = 0.095f;

    _x_des[2] = 0.08f;
    _y_des[2] = 0.040f;
    _z_des[2] = 0.095f;
  } else {
    _x_des[1] = 0.08f;
    _y_des[1] = 0.0f;
    _z_des[1] = 0.095f;

    _x_des[2] = 0.08f;
    _y_des[2] = -0.040f;
    _z_des[2] = 0.095f;
  }

  _x_des[3] = _x[3] + delta_x[3];
  _y_des[3] = _y[3] + delta_y[3];
  _z_des[3] = _z[3] + delta_z[3];

  alpha[3] = (distance[0] / distance[3]) * alpha[0];

  _f_des[3] = alpha[3];
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_PinchMT() {
  double distance[4];
  double delta_x[4];
  double delta_y[4];
  double delta_z[4];
  double alpha[4];

  double center_x_geo, center_y_geo, center_z_geo;

  alpha[1] = _f_des[1];

  center_x_geo = (_x[1] + _x[3]) / 2.0f;
  center_y_geo = (_y[1] + _y[3]) / 2.0f;
  center_z_geo = (_z[1] + _z[3]) / 2.0f;

  _x_des[1] = center_x_geo;
  _x_des[3] = center_x_geo;

  _y_des[1] = center_y_geo;
  _y_des[3] = center_y_geo;

  _z_des[1] = center_z_geo;
  _z_des[3] = center_z_geo;

  distance[1] = std::sqrt((_x[1] - _x_des[1]) * (_x[1] - _x_des[1]) + (_y[1] - _y_des[1]) * (_y[1] - _y_des[1]) +
                          (_z[1] - _z_des[1]) * (_z[1] - _z_des[1]));
  distance[3] = std::sqrt((_x[3] - _x_des[3]) * (_x[3] - _x_des[3]) + (_y[3] - _y_des[3]) * (_y[3] - _y_des[3]) +
                          (_z[3] - _z_des[3]) * (_z[3] - _z_des[3]));

  delta_x[1] = (_x_des[1] - _x[1]) / (distance[1]);
  delta_y[1] = (_y_des[1] - _y[1]) / (distance[1]);
  delta_z[1] = (_z_des[1] - _z[1]) / (distance[1]);

  delta_x[3] = (_x_des[3] - _x[3]) / (distance[3]);
  delta_y[3] = (_y_des[3] - _y[3]) / (distance[3]);
  delta_z[3] = (_z_des[3] - _z[3]) / (distance[3]);

  _x_des[1] = _x[1] + delta_x[1];
  _y_des[1] = _y[1] + delta_y[1];
  _z_des[1] = _z[1] + delta_z[1];

  if (hand_side_ == LEFT) {
    _x_des[0] = 0.08f;
    _y_des[0] = -0.05f;
    _z_des[0] = 0.095f;

    _x_des[2] = 0.08f;
    _y_des[2] = 0.040f;
    _z_des[2] = 0.095f;
  } else {
    _x_des[0] = 0.08f;
    _y_des[0] = 0.05f;
    _z_des[0] = 0.095f;

    _x_des[2] = 0.08f;
    _y_des[2] = -0.040f;
    _z_des[2] = 0.095f;
  }

  _x_des[3] = _x[3] + delta_x[3];
  _y_des[3] = _y[3] + delta_y[3];
  _z_des[3] = _z[3] + delta_z[3];

  alpha[3] = (distance[1] / distance[3]) * alpha[1];

  _f_des[3] = alpha[3];
}

void GraspAlgorithmV4LegacyReference::LegacyMotion_Envelop() {
  static double q_des_left[NOF][NOJ] = {{10 * DEG2RAD, 60 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},
                                        {0 * DEG2RAD, 60 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},
                                        {-20 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},
                                        {80 * DEG2RAD, 30 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD}};

  static double q_des_right[NOF][NOJ] = {{-10 * DEG2RAD, 60 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},
                                         {0 * DEG2RAD, 60 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},
                                         {20 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD},
                                         {80 * DEG2RAD, 30 * DEG2RAD, 0 * DEG2RAD, 0 * DEG2RAD}};

  if (_current_time > 0.15) {
    if (hand_side_ == LEFT) {
      _tau_des[3][0] = _kp[3][0] * (q_des_left[3][0] - _q_filtered[3][0]) - _kd[3][0] * _qdot_filtered[3][0];
      _tau_des[3][0] /= 800.0; // pwm to torque
      _tau_des[3][1] = _kp[3][1] * (q_des_left[3][1] - _q_filtered[3][1]) - _kd[3][1] * _qdot_filtered[3][1];
      _tau_des[3][1] /= 800.0; // pwm to torque
    } else {
      _tau_des[3][0] = _kp[3][0] * (q_des_right[3][0] - _q_filtered[3][0]) - _kd[3][0] * _qdot_filtered[3][0];
      _tau_des[3][0] /= 800.0; // pwm to torque
      _tau_des[3][1] = _kp[3][1] * (q_des_right[3][1] - _q_filtered[3][1]) - _kd[3][1] * _qdot_filtered[3][1];
      _tau_des[3][1] /= 800.0; // pwm to torque
    }
  }

  if (hand_side_ == LEFT) {
    _tau_des[0][0] = _kp[0][0] * (q_des_left[0][0] - _q_filtered[0][0]) - _kd[0][0] * _qdot_filtered[0][0];
    _tau_des[0][0] /= 800.0; // pwm to torque
    _tau_des[1][0] = _kp[1][0] * (q_des_left[1][0] - _q_filtered[1][0]) - _kd[1][0] * _qdot_filtered[1][0];
    _tau_des[1][0] /= 800.0; // pwm to torque
    _tau_des[2][0] = _kp[2][0] * (q_des_left[2][0] - _q_filtered[2][0]) - _kd[2][0] * _qdot_filtered[2][0];
    _tau_des[2][0] /= 800.0; // pwm to torque
  } else {
    _tau_des[0][0] = _kp[0][0] * (q_des_right[0][0] - _q_filtered[0][0]) - _kd[0][0] * _qdot_filtered[0][0];
    _tau_des[0][0] /= 800.0; // pwm to torque
    _tau_des[1][0] = _kp[1][0] * (q_des_right[1][0] - _q_filtered[1][0]) - _kd[1][0] * _qdot_filtered[1][0];
    _tau_des[1][0] /= 800.0; // pwm to torque
    _tau_des[2][0] = _kp[2][0] * (q_des_right[2][0] - _q_filtered[2][0]) - _kd[2][0] * _qdot_filtered[2][0];
    _tau_des[2][0] /= 800.0; // pwm to torque
  }

  if (_current_time > 0 && _current_time <= 0.05) {
    _tau_des[0][1] = 0;
    _tau_des[0][2] = 0;
    _tau_des[0][3] = 0;

    _tau_des[1][1] = 0;
    _tau_des[1][2] = 0;
    _tau_des[1][3] = 0;

    _tau_des[2][1] = (double)600 / 800.0;
    _tau_des[2][2] = 0;
    _tau_des[2][3] = 0;

    _tau_des[3][2] = 0;
    _tau_des[3][3] = 0;
  } else if (_current_time > 0.05 && _current_time <= 0.1) {
    _tau_des[0][1] = 0;
    _tau_des[0][2] = 0;
    _tau_des[0][3] = 0;

    _tau_des[1][1] = (double)600 / 800.0;
    _tau_des[1][2] = 0;
    _tau_des[1][3] = 0;

    _tau_des[2][1] = (double)600 / 800.0;
    _tau_des[2][2] = (double)360 / 800.0;
    _tau_des[2][3] = 0;

    _tau_des[3][2] = 0;
    _tau_des[3][3] = 0;
  } else if (_current_time > 0.1 && _current_time <= 0.2) {
    _tau_des[0][1] = (double)600 / 800.0;
    _tau_des[0][2] = 0;
    _tau_des[0][3] = 0;

    _tau_des[1][1] = (double)600 / 800.0;
    _tau_des[1][2] = (double)360 / 800.0;
    _tau_des[1][3] = 0;

    _tau_des[2][1] = (double)600 / 800.0;
    _tau_des[2][2] = (double)360 / 800.0;
    _tau_des[2][3] = (double)180 / 800.0;

    _tau_des[3][2] = (double)600 / 800.0;
    _tau_des[3][3] = 0;
  } else if (_current_time > 0.2) {
    _tau_des[0][1] = (double)600 / 800.0;
    _tau_des[0][2] = (double)360 / 800.0;
    _tau_des[0][3] = (double)180 / 800.0;

    _tau_des[1][1] = (double)600 / 800.0;
    _tau_des[1][2] = (double)360 / 800.0;
    _tau_des[1][3] = (double)180 / 800.0;

    _tau_des[2][1] = (double)600 / 800.0;
    _tau_des[2][2] = (double)360 / 800.0;
    _tau_des[2][3] = (double)180 / 800.0;

    _tau_des[3][2] = (double)600 / 800.0;
    _tau_des[3][3] = (double)420 / 800.0;
  }

  // double _envelop_torque_scalar = 0.1;
  //_envelop_torque_scalar = 0.5;

  for (int i = 0; i < NOF; i++)
    for (int j = 0; j < NOJ; j++)
      _tau_des[i][j] *= _envelop_torque_scalar;
}

/*
 *
 */
GraspAlgorithm::Ptr CreateGraspAlgorithmV4LegacyReference(std::vector<std::unique_ptr<FingerIoContext>>& finger_io_contexts,
                                                          HandSide hand_side) {
  return std::make_unique<GraspAlgorithmV4LegacyReference>(finger_io_contexts, hand_side);
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

#if 0 
void GraspAlgorithmV4LegacyReference::LegacyCalculateJacobian() {

  // %% link length of three-fingers (m)
  // % l2 = 0.054;
  // % l3 = 0.0384;
  // % l4 = 0.0257;

  // %%
  // % link legnth of each finger (m)
  // % notation : la_bc (length of JTab to JTac)
  // l1_12=0;
  // l1_23=l2;
  // l1_34=l3;
  // l1_45=l4;

  // l2_12=0;

  //   _J[0][0][0] = 257.0f / 10000.0f * (-S1_C[0] * S2_C[0] * C3_C[0] - S1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] +
  //                 257.0f / 10000.0f * (S1_C[0] * S2_C[0] * S3_C[0] - S1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] -
  //                 24.0f / 625.0f * S1_C[0] * S2_C[0] * C3_C[0] - 24.0f / 625.0f * S1_C[0] * C2_C[0] * S3_C[0] -
  //                 27.0f / 500.0f * S1_C[0] * S2_C[0];

  //   _J[0][0][0] = 257.0f / 10000.0f * (-S1_C[0] * S2_C[0] * C3_C[0] - S1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] +
  //                 257.0f / 10000.0f * (S1_C[0] * S2_C[0] * S3_C[0] - S1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] -
  //                 384.0f / 10000.0f * S1_C[0] * S2_C[0] * C3_C[0] - 24.0f / 625.0f * S1_C[0] * C2_C[0] * S3_C[0] -
  //                 540.0f / 10000.0f * S1_C[0] * S2_C[0];


  _J[0][0][0] = 257.0f / 10000.0f * (-S1_C[0] * S2_C[0] * C3_C[0] - S1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] +
                257.0f / 10000.0f * (S1_C[0] * S2_C[0] * S3_C[0] - S1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] -
                24.0f / 625.0f * S1_C[0] * S2_C[0] * C3_C[0] - 24.0f / 625.0f * S1_C[0] * C2_C[0] * S3_C[0] -
                27.0f / 500.0f * S1_C[0] * S2_C[0];
  _J[0][0][1] = 257.0f / 10000.0f * (-C1_C[0] * S2_C[0] * S3_C[0] + C1_C[0] * C2_C[0] * C3_C[0]) * C4_C[0] +
                257.0f / 10000.0f * (-C1_C[0] * C2_C[0] * S3_C[0] - C1_C[0] * S2_C[0] * C3_C[0]) * S4_C[0] +
                24.0f / 625.0f * C1_C[0] * C2_C[0] * C3_C[0] - 24.0f / 625.0f * C1_C[0] * S2_C[0] * S3_C[0] +
                27.0f / 500.0f * C1_C[0] * C2_C[0];
  _J[0][0][2] = 257.0f / 10000.0f * (-C1_C[0] * S2_C[0] * S3_C[0] + C1_C[0] * C2_C[0] * C3_C[0]) * C4_C[0] +
                257.0f / 10000.0f * (-C1_C[0] * C2_C[0] * S3_C[0] - C1_C[0] * S2_C[0] * C3_C[0]) * S4_C[0] -
                24.0f / 625.0f * C1_C[0] * S2_C[0] * S3_C[0] + 24.0f / 625.0f * C1_C[0] * C2_C[0] * C3_C[0];
  _J[0][0][3] = -257.0f / 10000.0f * (C1_C[0] * S2_C[0] * C3_C[0] + C1_C[0] * C2_C[0] * S3_C[0]) * S4_C[0] +
                257.0f / 10000.0f * (-C1_C[0] * S2_C[0] * S3_C[0] + C1_C[0] * C2_C[0] * C3_C[0]) * C4_C[0];

  _J[0][1][0] =
      257.0f / 10000.0f * (4981.0f / 5000.0f * C1_C[0] * S2_C[0] * C3_C[0] + 4981.0f / 5000.0f * C1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] +
      257.0f / 10000.0f * (-4981.0f / 5000.0f * C1_C[0] * S2_C[0] * S3_C[0] + 4981.0f / 5000.0f * C1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] +
      14943.0f / 390625.0f * C1_C[0] * S2_C[0] * C3_C[0] + 14943.0f / 390625.0f * C1_C[0] * C2_C[0] * S3_C[0] +
      134487.0f / 2500000.0f * C1_C[0] * S2_C[0];
  _J[0][1][1] = 257.0f / 10000.0f *
                    ((4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0] +
                     (-4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * S3_C[0]) *
                    C4_C[0] +
                257.0f / 10000.0f *
                    (-(4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * S3_C[0] +
                     (-4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * C3_C[0]) *
                    S4_C[0] +
                24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0] +
                24.0f / 625.0f * (-4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * S3_C[0] +
                134487.0f / 2500000.0f * S1_C[0] * C2_C[0] - 23517.0f / 5000000.0f * S2_C[0];
  _J[0][1][2] = 257.0f / 10000.0f *
                    (-(4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * S3_C[0] +
                     (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0]) *
                    C4_C[0] +
                257.0f / 10000.0f *
                    (-(4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * C3_C[0] -
                     (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * S3_C[0]) *
                    S4_C[0] -
                24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * S3_C[0] +
                24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0];
  _J[0][1][3] = -257.0f / 10000.0f *
                    ((4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * C3_C[0] +
                     (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * S3_C[0]) *
                    S4_C[0] +
                257.0f / 10000.0f *
                    (-(4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * S3_C[0] +
                     (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0]) *
                    C4_C[0];

  _J[0][2][0] =
      257.0f / 10000.0f * (-871.0f / 10000.0f * C1_C[0] * S2_C[0] * C3_C[0] - 871.0f / 10000.0f * C1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] +
      257.0f / 10000.0f * (871.0f / 10000.0f * C1_C[0] * S2_C[0] * S3_C[0] - 871.0f / 10000.0f * C1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] -
      2613.0f / 781250.0f * C1_C[0] * S2_C[0] * C3_C[0] - 2613.0f / 781250.0f * C1_C[0] * C2_C[0] * S3_C[0] -
      23517.0f / 5000000.0f * C1_C[0] * S2_C[0];
  _J[0][2][1] = 257.0f / 10000.0f *
                    ((-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0] +
                     (871.0f / 10000.0f * S1_C[0] * S2_C[0] - 4981.0f / 5000.0f * C2_C[0]) * S3_C[0]) *
                    C4_C[0] +
                257.0f / 10000.0f *
                    (-(-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0] +
                     (871.0f / 10000.0f * S1_C[0] * S2_C[0] - 4981.0f / 5000.0f * C2_C[0]) * C3_C[0]) *
                    S4_C[0] +
                24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0] +
                24.0f / 625.0f * (871.0f / 10000.0f * S1_C[0] * S2_C[0] - 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] -
                23517.0f / 5000000.0f * S1_C[0] * C2_C[0] - 134487.0f / 2500000.0f * S2_C[0];
  _J[0][2][2] = 257.0f / 10000.0f *
                    (-(-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] +
                     (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0]) *
                    C4_C[0] +
                257.0f / 10000.0f *
                    (-(-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * C3_C[0] -
                     (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0]) *
                    S4_C[0] -
                24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] +
                24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0];
  _J[0][2][3] = -257.0f / 10000.0f *
                    ((-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * C3_C[0] +
                     (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0]) *
                    S4_C[0] +
                257.0f / 10000.0f *
                    (-(-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] +
                     (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0]) *
                    C4_C[0];

  _J[1][0][0] = 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * C3_C[1] - S1_C[1] * C2_C[1] * S3_C[1]) * C4_C[1] +
                257.0f / 10000.0f * (S1_C[1] * S2_C[1] * S3_C[1] - S1_C[1] * C2_C[1] * C3_C[1]) * S4_C[1] -
                24.0f / 625.0f * S1_C[1] * S2_C[1] * C3_C[1] - 24.0f / 625.0f * S1_C[1] * C2_C[1] * S3_C[1] -
                27.0f / 500.0f * S1_C[1] * S2_C[1];
  _J[1][0][1] = 257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1] +
                257.0f / 10000.0f * (-C1_C[1] * C2_C[1] * S3_C[1] - C1_C[1] * S2_C[1] * C3_C[1]) * S4_C[1] +
                24.0f / 625.0f * C1_C[1] * C2_C[1] * C3_C[1] - 24.0f / 625.0f * C1_C[1] * S2_C[1] * S3_C[1] +
                27.0f / 500.0f * C1_C[1] * C2_C[1];
  _J[1][0][2] = 257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1] +
                257.0f / 10000.0f * (-C1_C[1] * C2_C[1] * S3_C[1] - C1_C[1] * S2_C[1] * C3_C[1]) * S4_C[1] -
                24.0f / 625.0f * C1_C[1] * S2_C[1] * S3_C[1] + 24.0f / 625.0f * C1_C[1] * C2_C[1] * C3_C[1];
  _J[1][0][3] = -257.0f / 10000.0f * (C1_C[1] * S2_C[1] * C3_C[1] + C1_C[1] * C2_C[1] * S3_C[1]) * S4_C[1] +
                257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1];

  _J[1][1][0] = 257.0f / 10000.0f * (C1_C[1] * S2_C[1] * C3_C[1] + C1_C[1] * C2_C[1] * S3_C[1]) * C4_C[1] +
                257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * S4_C[1] +
                24.0f / 625.0f * C1_C[1] * S2_C[1] * C3_C[1] + 24.0f / 625.0f * C1_C[1] * C2_C[1] * S3_C[1] +
                27.0f / 500.0f * C1_C[1] * S2_C[1];
  _J[1][1][1] = 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * S3_C[1] + S1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1] +
                257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * C3_C[1] - S1_C[1] * C2_C[1] * S3_C[1]) * S4_C[1] +
                24.0f / 625.0f * S1_C[1] * C2_C[1] * C3_C[1] - 24.0f / 625.0f * S1_C[1] * S2_C[1] * S3_C[1] +
                27.0f / 500.0f * S1_C[1] * C2_C[1];
  _J[1][1][2] = 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * S3_C[1] + S1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1] +
                257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * C3_C[1] - S1_C[1] * C2_C[1] * S3_C[1]) * S4_C[1] -
                24.0f / 625.0f * S1_C[1] * S2_C[1] * S3_C[1] + 24.0f / 625.0f * S1_C[1] * C2_C[1] * C3_C[1];
  _J[1][1][3] = -257.0f / 10000.0f * (S1_C[1] * S2_C[1] * C3_C[1] + S1_C[1] * C2_C[1] * S3_C[1]) * S4_C[1] +
                257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * S3_C[1] + S1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1];

  _J[1][2][0] = 0;
  _J[1][2][1] = 257.0f / 10000.0f * (-C2_C[1] * S3_C[1] - S2_C[1] * C3_C[1]) * C4_C[1] +
                257.0f / 10000.0f * (S2_C[1] * S3_C[1] - C2_C[1] * C3_C[1]) * S4_C[1] - 24.0f / 625.0f * S2_C[1] * C3_C[1] -
                24.0f / 625.0f * C2_C[1] * S3_C[1] - 27.0f / 500.0f * S2_C[1];
  _J[1][2][2] = 257.0f / 10000.0f * (-C2_C[1] * S3_C[1] - S2_C[1] * C3_C[1]) * C4_C[1] +
                257.0f / 10000.0f * (S2_C[1] * S3_C[1] - C2_C[1] * C3_C[1]) * S4_C[1] - 24.0f / 625.0f * C2_C[1] * S3_C[1] -
                24.0f / 625.0f * S2_C[1] * C3_C[1];
  _J[1][2][3] = -257.0f / 10000.0f * (C2_C[1] * C3_C[1] - S2_C[1] * S3_C[1]) * S4_C[1] +
                257.0f / 10000.0f * (-C2_C[1] * S3_C[1] - S2_C[1] * C3_C[1]) * C4_C[1];

  _J[2][0][0] = 257.0f / 10000.0f * (-S1_C[2] * S2_C[2] * C3_C[2] - S1_C[2] * C2_C[2] * S3_C[2]) * C4_C[2] +
                257.0f / 10000.0f * (S1_C[2] * S2_C[2] * S3_C[2] - S1_C[2] * C2_C[2] * C3_C[2]) * S4_C[2] -
                24.0f / 625.0f * S1_C[2] * S2_C[2] * C3_C[2] - 24.0f / 625.0f * S1_C[2] * C2_C[2] * S3_C[2] -
                27.0f / 500.0f * S1_C[2] * S2_C[2];
  _J[2][0][1] = 257.0f / 10000.0f * (-C1_C[2] * S2_C[2] * S3_C[2] + C1_C[2] * C2_C[2] * C3_C[2]) * C4_C[2] +
                257.0f / 10000.0f * (-C1_C[2] * C2_C[2] * S3_C[2] - C1_C[2] * S2_C[2] * C3_C[2]) * S4_C[2] +
                24.0f / 625.0f * C1_C[2] * C2_C[2] * C3_C[2] - 24.0f / 625.0f * C1_C[2] * S2_C[2] * S3_C[2] +
                27.0f / 500.0f * C1_C[2] * C2_C[2];
  _J[2][0][2] = 257.0f / 10000.0f * (-C1_C[2] * S2_C[2] * S3_C[2] + C1_C[2] * C2_C[2] * C3_C[2]) * C4_C[2] +
                257.0f / 10000.0f * (-C1_C[2] * C2_C[2] * S3_C[2] - C1_C[2] * S2_C[2] * C3_C[2]) * S4_C[2] -
                24.0f / 625.0f * C1_C[2] * S2_C[2] * S3_C[2] + 24.0f / 625.0f * C1_C[2] * C2_C[2] * C3_C[2];
  _J[2][0][3] = -257.0f / 10000.0f * (C1_C[2] * S2_C[2] * C3_C[2] + C1_C[2] * C2_C[2] * S3_C[2]) * S4_C[2] +
                257.0f / 10000.0f * (-C1_C[2] * S2_C[2] * S3_C[2] + C1_C[2] * C2_C[2] * C3_C[2]) * C4_C[2];

  _J[2][1][0] =
      257.0f / 10000.0f * (4981.0f / 5000.0f * C1_C[2] * S2_C[2] * C3_C[2] + 4981.0f / 5000.0f * C1_C[2] * C2_C[2] * S3_C[2]) * C4_C[2] +
      257.0f / 10000.0f * (-4981.0f / 5000.0f * C1_C[2] * S2_C[2] * S3_C[2] + 4981.0f / 5000.0f * C1_C[2] * C2_C[2] * C3_C[2]) * S4_C[2] +
      14943.0f / 390625.0f * C1_C[2] * S2_C[2] * C3_C[2] + 14943.0f / 390625.0f * C1_C[2] * C2_C[2] * S3_C[2] +
      134487.0f / 2500000.0f * C1_C[2] * S2_C[2];
  _J[2][1][1] = 257.0f / 10000.0f *
                    ((4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2] +
                     (-4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * S3_C[2]) *
                    C4_C[2] +
                257.0f / 10000.0f *
                    (-(4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * S3_C[2] +
                     (-4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * C3_C[2]) *
                    S4_C[2] +
                24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2] +
                24.0f / 625.0f * (-4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * S3_C[2] +
                134487.0f / 2500000.0f * S1_C[2] * C2_C[2] + 23517.0f / 5000000.0f * S2_C[2];
  _J[2][1][2] = 257.0f / 10000.0f *
                    (-(4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * S3_C[2] +
                     (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2]) *
                    C4_C[2] +
                257.0f / 10000.0f *
                    (-(4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * C3_C[2] -
                     (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * S3_C[2]) *
                    S4_C[2] -
                24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * S3_C[2] +
                24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2];
  _J[2][1][3] = -257.0f / 10000.0f *
                    ((4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * C3_C[2] +
                     (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * S3_C[2]) *
                    S4_C[2] +
                257.0f / 10000.0f *
                    (-(4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * S3_C[2] +
                     (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2]) *
                    C4_C[2];

  _J[2][2][0] =
      257.0f / 10000.0f * (871.0f / 10000.0f * C1_C[2] * S2_C[2] * C3_C[2] + 871.0f / 10000.0f * C1_C[2] * C2_C[2] * S3_C[2]) * C4_C[2] +
      257.0f / 10000.0f * (-871.0f / 10000.0f * C1_C[2] * S2_C[2] * S3_C[2] + 871.0f / 10000.0f * C1_C[2] * C2_C[2] * C3_C[2]) * S4_C[2] +
      2613.0f / 781250.0f * C1_C[2] * S2_C[2] * C3_C[2] + 2613.0f / 781250.0f * C1_C[2] * C2_C[2] * S3_C[2] +
      23517.0f / 5000000.0f * C1_C[2] * S2_C[2];
  _J[2][2][1] = 257.0f / 10000.0f *
                    ((871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2] +
                     (-871.0f / 10000.0f * S1_C[2] * S2_C[2] - 4981.0f / 5000.0f * C2_C[2]) * S3_C[2]) *
                    C4_C[2] +
                257.0f / 10000.0f *
                    (-(871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2] +
                     (-871.0f / 10000.0f * S1_C[2] * S2_C[2] - 4981.0f / 5000.0f * C2_C[2]) * C3_C[2]) *
                    S4_C[2] +
                24.0f / 625.0f * (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2] +
                24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[2] * S2_C[2] - 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] +
                23517.0f / 5000000.0f * S1_C[2] * C2_C[2] - 134487.0f / 2500000.0f * S2_C[2];
  _J[2][2][2] = 257.0f / 10000.0f *
                    (-(871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] +
                     (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2]) *
                    C4_C[2] +
                257.0f / 10000.0f *
                    (-(871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * C3_C[2] -
                     (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2]) *
                    S4_C[2] -
                24.0f / 625.0f * (871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] +
                24 / 625.0f * (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2];
  _J[2][2][3] = -257.0f / 10000.0f *
                    ((871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * C3_C[2] +
                     (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2]) *
                    S4_C[2] +
                257.0f / 10000.0f *
                    (-(871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] +
                     (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2]) *
                    C4_C[2];

  _J[3][0][0] = (137.0f * C1_C[3]) / 2500.0f - S1_C[3] / 200.0f + (257.0f * C1_C[3] * C3_C[3]) / 5000.0f +
                (37.0f * C34_C[3] * C1_C[3]) / 1000.0f - (257.0f * S1_C[3] * S2_C[3] * S3_C[3]) / 5000.0f -
                (37.0f * S34_C[3] * S1_C[3] * S2_C[3]) / 1000.0f;
  _J[3][0][1] = (37.0f * S34_C[3] * C1_C[3] * C2_C[3]) / 1000.0f + (257.0f * C1_C[3] * C2_C[3] * S3_C[3]) / 5000.0f;
  _J[3][0][2] = (37.0f * C34_C[3] * C1_C[3] * S2_C[3]) / 1000.0f - (37.0f * S34_C[3] * S1_C[3]) / 1000.0f -
                (257.0f * S1_C[3] * S3_C[3]) / 5000.0f + (257.0f * C1_C[3] * C3_C[3] * S2_C[3]) / 5000.0f;
  _J[3][0][3] = (37.0f * C34_C[3] * C1_C[3] * S2_C[3]) / 1000.0f - (37.0f * S34_C[3] * S1_C[3]) / 1000.0f;

  if (hand_side_ == LEFT) {
    _J[3][1][0] = (4981.0f * C1_C[3]) / 1000000.0f + (682397.0f * S1_C[3]) / 12500000.0f + (1280117.0f * C3_C[3] * S1_C[3]) / 25000000.0f +
                  (184297.0f * C34_C[3] * S1_C[3]) / 5000000.0f + (184297.0f * S34_C[3] * C1_C[3] * S2_C[3]) / 5000000.0f +
                  (1280117.0f * C1_C[3] * S2_C[3] * S3_C[3]) / 25000000.0f;
    _J[3][1][1] = (223847.0f * S2_C[3] * S3_C[3]) / 50000000.0f + (32227.0f * S34_C[3] * S2_C[3]) / 10000000.0f +
                  (184297.0f * S34_C[3] * C2_C[3] * S1_C[3]) / 5000000.0f + (1280117.0f * C2_C[3] * S1_C[3] * S3_C[3]) / 25000000.0f;
    _J[3][1][2] = -(223847.0f * C2_C[3] * C3_C[3]) / 50000000.0f + (1280117.0f * C1_C[3] * S3_C[3]) / 25000000.0f -
                  (32227.0f * C34_C[3] * C2_C[3]) / 10000000.0f + (184297.0f * S34_C[3] * C1_C[3]) / 5000000.0f +
                  (184297.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 5000000.0f + (1280117.0f * C3_C[3] * S1_C[3] * S2_C[3]) / 25000000.0f;
    _J[3][1][3] = -(32227.0f * C34_C[3] * C2_C[3]) / 10000000.0f + (184297.0f * S34_C[3] * C1_C[3]) / 5000000.0f +
                  (184297.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 5000000.0f;
  } else {
    _J[3][1][0] = -(4981.0f * C1_C[3]) / 1000000.0f - (682397.0f * S1_C[3]) / 12500000.0f - (1280117.0f * C3_C[3] * S1_C[3]) / 25000000.0f -
                  (184297.0f * C34_C[3] * S1_C[3]) / 5000000.0f - (184297.0f * S34_C[3] * C1_C[3] * S2_C[3]) / 5000000.0f -
                  (1280117.0f * C1_C[3] * S2_C[3] * S3_C[3]) / 25000000.0f;
    _J[3][1][1] = -(223847.0f * S2_C[3] * S3_C[3]) / 50000000.0f - (32227.0f * S34_C[3] * S2_C[3]) / 10000000.0f -
                  (184297.0f * S34_C[3] * C2_C[3] * S1_C[3]) / 5000000.0f - (1280117.0f * C2_C[3] * S1_C[3] * S3_C[3]) / 25000000.0f;
    _J[3][1][2] = (223847.0f * C2_C[3] * C3_C[3]) / 50000000.0f - (1280117.0f * C1_C[3] * S3_C[3]) / 25000000.0f +
                  (32227.0f * C34_C[3] * C2_C[3]) / 10000000.0f - (184297.0f * S34_C[3] * C1_C[3]) / 5000000.0f -
                  (184297.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 5000000.0f - (1280117.0f * C3_C[3] * S1_C[3] * S2_C[3]) / 25000000.0f;
    _J[3][1][3] = (32227.0f * C34_C[3] * C2_C[3]) / 10000000.0f - (184297.0f * S34_C[3] * C1_C[3]) / 5000000.0f -
                  (184297.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 5000000.0f;
  }

  _J[3][2][0] = (871.0f * C1_C[3]) / 2000000.0f + (119327.0f * S1_C[3]) / 25000000.0f + (223847.0f * C3_C[3] * S1_C[3]) / 50000000.0f +
                (32227.0f * C34_C[3] * S1_C[3]) / 10000000.0f + (32227.0f * S34_C[3] * C1_C[3] * S2_C[3]) / 10000000.0f +
                (223847.0f * C1_C[3] * S2_C[3] * S3_C[3]) / 50000000.0f;
  _J[3][2][1] = (32227.0f * S34_C[3] * C2_C[3] * S1_C[3]) / 10000000.0f - (184297.0f * S34_C[3] * S2_C[3]) / 5000000.0f -
                (1280117.0f * S2_C[3] * S3_C[3]) / 25000000.0f + (223847.0f * C2_C[3] * S1_C[3] * S3_C[3]) / 50000000.0f;
  _J[3][2][2] = (1280117.0f * C2_C[3] * C3_C[3]) / 25000000.0f + (223847.0f * C1_C[3] * S3_C[3]) / 50000000.0f +
                (184297.0f * C34_C[3] * C2_C[3]) / 5000000.0f + (32227.0f * S34_C[3] * C1_C[3]) / 10000000.0f +
                (32227.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 10000000.0f + (223847.0f * C3_C[3] * S1_C[3] * S2_C[3]) / 50000000.0f;
  _J[3][2][3] = (184297.0f * C34_C[3] * C2_C[3]) / 5000000.0f + (32227.0f * S34_C[3] * C1_C[3]) / 10000000.0f +
                (32227.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 10000000.0f;


  if(false){
    /*
     * Octave로 구한 식으로 _J 를 검증함. 
     *  - Index Finger 값만 
     *  - J0 == J0r 확인 
     */
    constexpr double l2 = 0.054;
    constexpr double l3 = 0.0384;
    constexpr double l4 = 0.0257;
    const auto& _q = _q_filtered;
    const auto& theta11 = _q[0][0];
    const auto& theta12 = _q[0][1];
    const auto& theta13 = _q[0][2];
    const auto& theta14 = _q[0][3];
    const auto& pi = M_PI; 
    // Jacobian J0 elements: Finger 1 (Index)
    double J0_11 = -(l2 * sin(theta12) + l3 * sin(theta12 + theta13) + l4 * sin(theta12 + theta13 + theta14)) * sin(theta11);
    double J0_12 = (l2 * cos(theta12) + l3 * cos(theta12 + theta13) + l4 * cos(theta12 + theta13 + theta14)) * cos(theta11);
    double J0_13 = (l3 * cos(theta12 + theta13) + l4 * cos(theta12 + theta13 + theta14)) * cos(theta11);
    double J0_14 = l4 * cos(theta11) * cos(theta12 + theta13 + theta14);

    double J0_21 = (l2 * sin(theta12) + l3 * sin(theta12 + theta13) + l4 * sin(theta12 + theta13 + theta14)) * cos(pi / 36) * cos(theta11);
    double J0_22 = l2 * (sin(theta11) * cos(pi / 36) * cos(theta12) - sin(pi / 36) * sin(theta12)) +
                   l3 * (sin(theta11) * cos(pi / 36) * cos(theta12 + theta13) - sin(pi / 36) * sin(theta12 + theta13)) +
                   l4 * (sin(theta11) * cos(pi / 36) * cos(theta12 + theta13 + theta14) - sin(pi / 36) * sin(theta12 + theta13 + theta14));
    double J0_23 = l3 * (sin(theta11) * cos(pi / 36) * cos(theta12 + theta13) - sin(pi / 36) * sin(theta12 + theta13)) +
                   l4 * (sin(theta11) * cos(pi / 36) * cos(theta12 + theta13 + theta14) - sin(pi / 36) * sin(theta12 + theta13 + theta14));
    double J0_24 = l4 * (sin(theta11) * cos(pi / 36) * cos(theta12 + theta13 + theta14) - sin(pi / 36) * sin(theta12 + theta13 + theta14));

    double J0_31 = -(l2 * sin(theta12) + l3 * sin(theta12 + theta13) + l4 * sin(theta12 + theta13 + theta14)) * sin(pi / 36) * cos(theta11);
    double J0_32 = -l2 * sin(pi / 36) * sin(theta11) * cos(theta12) - l2 * sin(theta12) * cos(pi / 36) -
                   l3 * sin(pi / 36) * sin(theta11) * cos(theta12 + theta13) - l3 * sin(theta12 + theta13) * cos(pi / 36) -
                   l4 * sin(pi / 36) * sin(theta11) * cos(theta12 + theta13 + theta14) -
                   l4 * sin(theta12 + theta13 + theta14) * cos(pi / 36);
    double J0_33 = -l3 * sin(pi / 36) * sin(theta11) * cos(theta12 + theta13) - l3 * sin(theta12 + theta13) * cos(pi / 36) -
                   l4 * sin(pi / 36) * sin(theta11) * cos(theta12 + theta13 + theta14) -
                   l4 * sin(theta12 + theta13 + theta14) * cos(pi / 36);
    double J0_34 = -l4 * (sin(pi / 36) * sin(theta11) * cos(theta12 + theta13 + theta14) + sin(theta12 + theta13 + theta14) * cos(pi / 36));

    Eigen::MatrixXd J0(3, 4);
    for (int joint_idx = 0; joint_idx < NOJ; joint_idx++) {
      for (int dim_idx = 0; dim_idx < 3; dim_idx++) {
        J0(dim_idx, joint_idx) = _J[0][dim_idx][joint_idx];
      }
    }

    // clang-format off 
    Eigen::MatrixXd J0r(3, 4); 
    J0r << J0_11, J0_12, J0_13, J0_14, 
           J0_21, J0_22, J0_23, J0_24, 
           J0_31, J0_32, J0_33, J0_34;
    // clang-format on 
    
    SPDLOG_TRACE("J0\n{}", util::mat2str(J0));
    SPDLOG_TRACE("J0r\n{}", util::mat2str(J0r));
    SPDLOG_TRACE("Error {:.4e}", (J0 - J0r).norm()); 
  }

}
#endif
