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

/**
 * @file plexus_spec.hpp
 * @brief Defines constants, message IDs, and data structures for the Allegro Hand Plexus protocol.
 * @author ('c')void
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace plexus_sdk {

/** @brief Maximum number of fingers on the hand. */
constexpr int FINGER_MAX = 4;
/** @brief Number of joints per finger. */
constexpr int JONIT_PER_FINGER = 4;
/** @brief Total number of joints on the hand. */
constexpr int JONIT_MAX = FINGER_MAX * JONIT_PER_FINGER;
/** @brief Maximum number of hand IDs supported. */
constexpr int HAND_ID_MAX = 8; // Not currently used, but defined for protocol specification.

/** @brief Default sampling period for position data in seconds */
constexpr double POSITION_SAMPLING_PERIOD = 0.01; // sec
constexpr double VELOCITY_SAMPLING_PERIOD = 0.01; // sec
constexpr double TORQUE_SAMPLING_PERIOD = 0.01;   // sec
constexpr double IMU_SAMPLING_PERIOD = 0.01;      // sec
/** @brief Default cutoff frequency for the low-pass filter in Hz. */
constexpr double LOWPASS_FILTER_CUTOFF_FREQ = 30.0;

// https://emanual.robotis.com/docs/kr/dxl/x/xc330-t288/
// Stall Torque 1.150 [Nm/A]
constexpr double TORQUE_CONSTANT = 1.15 / 1000.0; // Nm/mA

/**
 * @brief Enumeration for finger indices.
 */
enum FingerIndex {
  FINGER_INDEX = 0,  ///< Index finger.
  FINGER_MIDDLE = 1, ///< Middle finger.
  FINGER_RING = 2,   ///< Ring finger.
  FINGER_THUMB = 3,  ///< Thumb.
};

/**
 * @brief Enumeration of CAN message identifiers for commands and data requests.
 */
enum MessageId {
  //  Define CAN Command
  ID_CMD_SYSTEM_ON_OFF = 0x03, ///< Command to turn the system on or off.
  ID_CMD_SET_PERIOD = 0x04,    ///< Command to set the communication period.
  ID_CMD_CONFIG = 0x05,        ///< Command for configuration settings.

  ID_CMD_SET_POSE = 0x10,                    ///< Base ID for setting pose for a finger.
  ID_CMD_SET_POSE_1 = (ID_CMD_SET_POSE + 0), ///< Set pose for finger 1.
  ID_CMD_SET_POSE_2 = (ID_CMD_SET_POSE + 1), ///< Set pose for finger 2.
  ID_CMD_SET_POSE_3 = (ID_CMD_SET_POSE + 2), ///< Set pose for finger 3.
  ID_CMD_SET_POSE_4 = (ID_CMD_SET_POSE + 3), ///< Set pose for finger 4.

  ID_CMD_SET_TORQUE = 0x14,                      ///< Base ID for setting torque for a finger.
  ID_CMD_SET_TORQUE_1 = (ID_CMD_SET_TORQUE + 0), ///< Set torque for finger 1.
  ID_CMD_SET_TORQUE_2 = (ID_CMD_SET_TORQUE + 1), ///< Set torque for finger 2.
  ID_CMD_SET_TORQUE_3 = (ID_CMD_SET_TORQUE + 2), ///< Set torque for finger 3.
  ID_CMD_SET_TORQUE_4 = (ID_CMD_SET_TORQUE + 3), ///< Set torque for finger 4.

  //  Define CAN Data Reqeust (RTR)
  ID_RTR_FINGER_POSE = 0x20,                       ///< Base ID for requesting finger pose.
  ID_RTR_FINGER_POSE_1 = (ID_RTR_FINGER_POSE + 0), ///< Request pose for finger 1.
  ID_RTR_FINGER_POSE_2 = (ID_RTR_FINGER_POSE + 1), ///< Request pose for finger 2.
  ID_RTR_FINGER_POSE_3 = (ID_RTR_FINGER_POSE + 2), ///< Request pose for finger 3.
  ID_RTR_FINGER_POSE_4 = (ID_RTR_FINGER_POSE + 3), ///< Request pose for finger 4.

  ID_RTR_FINGER_TORQUE = 0x24,                         ///< Base ID for requesting finger torque.
  ID_RTR_FINGER_TORQUE_1 = (ID_RTR_FINGER_TORQUE + 0), ///< Request torque for finger 1.
  ID_RTR_FINGER_TORQUE_2 = (ID_RTR_FINGER_TORQUE + 1), ///< Request torque for finger 2.
  ID_RTR_FINGER_TORQUE_3 = (ID_RTR_FINGER_TORQUE + 2), ///< Request torque for finger 3.
  ID_RTR_FINGER_TORQUE_4 = (ID_RTR_FINGER_TORQUE + 3), ///< Request torque for finger 4.

  ID_RTR_FINGER_VELOCITY = 0x28,                           ///< Base ID for requesting finger velocity.
  ID_RTR_FINGER_VELOCITY_1 = (ID_RTR_FINGER_VELOCITY + 0), ///< Request velocity for finger 1.
  ID_RTR_FINGER_VELOCITY_2 = (ID_RTR_FINGER_VELOCITY + 1), ///< Request velocity for finger 2.
  ID_RTR_FINGER_VELOCITY_3 = (ID_RTR_FINGER_VELOCITY + 2), ///< Request velocity for finger 3.
  ID_RTR_FINGER_VELOCITY_4 = (ID_RTR_FINGER_VELOCITY + 3), ///< Request velocity for finger 4.

  ID_RTR_IMU_DATA = 0x30, ///< Request for IMU data.

  ID_RTR_TEMPERATURE = 0x34,                       ///< Base ID for requesting temperature.
  ID_RTR_TEMPERATURE_1 = (ID_RTR_TEMPERATURE + 0), ///< Request temperature for finger 1.
  ID_RTR_TEMPERATURE_2 = (ID_RTR_TEMPERATURE + 1), ///< Request temperature for finger 2.
  ID_RTR_TEMPERATURE_3 = (ID_RTR_TEMPERATURE + 2), ///< Request temperature for finger 3.
  ID_RTR_TEMPERATURE_4 = (ID_RTR_TEMPERATURE + 3), ///< Request temperature for finger 4.

  ID_RTR_HAND_INFO = 0x40, ///< Request for hand information.
  ID_RTR_SERIAL = 0x41,    ///< Request for the hand's serial number.
};

/**
 * @brief Enumeration for the hand's operation modes.
 */
enum OperationMode {
  // CURRENT_BASED_POSITION_CONTROL = 0x0A, ///< Position control with current-based compliance.
  // POSITION_CONTROL = 0x0B,               ///< Standard position control.
  // TORQUE_CONTROL = 0x0C,                 ///< Torque (effort) control.

  CURRENT_BASED_POSITION_CONTROL = 0x0A, ///< Position control with current-based compliance.
  TORQUE_CONTROL = 0x0B,                 ///< Torque (effort) control.
};

/**
 * @brief Structure defining the layout of the CAN frame identifier.
 *
 * This bitfield defines the layout for a 32-bit integer that represents
 * the CAN ID for a frame, as used by the CAN driver's API.
 */
struct can_id_t {
  uint32_t hand_id : 2;     ///< Unique identifier for the hand (0-3).
  uint32_t message_id : 27; ///< The specific message identifier.
  uint32_t rtr : 2;         ///< Remote Transmission Request flag.
  uint32_t extended : 1;    ///< Extended frame format flag.
};

/**
 * @brief Data structure for sending configuration commands.
 *
 * Used to set the device ID or the operation mode of the hand.
 */
struct data_config_t {
  uint8_t sel; // device_id or operating mode
  uint8_t value;
  uint8_t rfu[4] = {0, 0, 0, 0}; ///< Reserved for future use.

  inline void encode(uint8_t device_id) {
    sel = 2;
    value = device_id;
  }
  inline void encode(OperationMode mode) {
    sel = 1;
    value = mode;
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for hand information received from the device.
 *
 * This structure holds general information about the hand, such as version,
 * type, and status.
 */
struct data_info_t {
  uint16_t hand_version;     ///< Hardware version of the hand.
  uint16_t firmware_version; ///< Firmware version of the main board.
  uint8_t hand_type;         ///< Hand type (e.g., 0x01 for right hand).
  uint8_t hub_temperature;   ///< Temperature of the main hub in Celsius.
  uint8_t status;            ///< General status flags (reserved for future use).
  struct {
    uint8_t overload_error : 1;         ///< Flag for overload error.
    uint8_t electrical_shock_error : 1; ///< Flag for electrical shock error.
    uint8_t temperature_error : 1;      ///< Flag for over-temperature error.
    uint8_t input_voltage_error : 1;    ///< Flag for input voltage error.
    uint8_t f1_connection_error : 1;    ///< Flag for finger 1 (Index) connection error.
    uint8_t f2_connection_error : 1;    ///< Flag for finger 2 (Middle) connection error.
    uint8_t f3_connection_error : 1;    ///< Flag for finger 3 (Ring) connection error.
    uint8_t f4_connection_error : 1;    ///< Flag for finger 4 (Thumb) connection error.
  } joint_status;
  std::string to_string();
} __attribute__((__packed__));

/**
 * @brief Data structure for the hand's serial number.
 */
struct data_serial_t {
  uint8_t data[8]; // ASCII
  /**
   * @brief Converts the raw serial data to a full serial number string.
   * @return The formatted serial number (e.g., "SAH040...").
   */
  std::string to_string() { return fmt::format("SAH040{}", std::string((const char*)data, 8)); }
} __attribute__((__packed__));

/**
 * @brief Data structure for encoding and decoding joint torque values.
 */
struct data_torque_t {
  int16_t joint_torque[JONIT_PER_FINGER]; ///< Raw torque values for 4 joints in mA.
  /**
   * @brief
   * @param joint_idx The index of the joint_torque within the finger (0-3).
   * @param value The normalized torque value to encode.
   */
  inline void encode(int joint_idx, double torque) {
    // torque := Nm
    assert(joint_idx < JONIT_PER_FINGER);
    joint_torque[joint_idx] = torque / TORQUE_CONSTANT;
  }
  inline float decode(int joint_idx) const {
    /// @brief
    assert(joint_idx < JONIT_PER_FINGER);
    return joint_torque[joint_idx] * TORQUE_CONSTANT;
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for encoding and decoding joint position values.
 */
struct data_position_t {
  int16_t joint[JONIT_PER_FINGER];                           ///< Raw encoder counts for 4 joints.
  static constexpr double scale_factor_ = 4096 / (2 * M_PI); ///< Scale factor to convert radians to encoder counts.
  /**
   * @brief Encodes a position value in radians into the raw format for sending.
   * @param joint_idx The index of the joint within the finger (0-3).
   * @param value The position value in radians to encode.
   */
  inline void encode(int joint_idx, double value) {
    assert(joint_idx < JONIT_PER_FINGER);
    joint[joint_idx] = value * scale_factor_;
  }
  inline float decode(int joint_idx) const {
    /// @brief Decodes a raw position value into radians.
    assert(joint_idx < JONIT_PER_FINGER);
    return joint[joint_idx] / scale_factor_;
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for encoding and decoding joint velocity values.
 */
struct data_velocity_t {
  int16_t joint[JONIT_PER_FINGER];                           ///< Raw encoder counts for 4 joints.
  static constexpr double scale_factor_ = 0.229 * M_PI / 30; ///< Scale factor to convert encoded rpm to radian/sec.
  inline float decode(int joint_idx) const {
    /// @brief Decodes a raw velocity value into radian/sec.
    assert(joint_idx < JONIT_PER_FINGER);
    return joint[joint_idx] * scale_factor_;
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for decoding joint temperature values.
 */
struct data_temperature_t {
  uint8_t joint_temperature[JONIT_PER_FINGER]; ///< Temperature in Celsius for each of the 4 joints in a finger.
  /** @brief Decodes the temperature for a specific joint. */
  inline uint8_t decode(int joint_idx) const {
    assert(joint_idx < JONIT_PER_FINGER);
    return joint_temperature[joint_idx];
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for decoding IMU roll, pitch, and yaw values. (TEST)
 */
#define SCALE_Q(n) (1.0f / (1 << n))
#define RAD_TO_DEG (180.0f / M_PI)

struct data_imu_rpy_t {
  uint16_t rpy[3]; ///< Raw roll, pitch, yaw data.
  float scale_factor_ = SCALE_Q(12) * RAD_TO_DEG;
  /// @brief Decodes a raw IMU value.
  inline float decode(int idx) {
    assert(idx < 3);
    return rpy[idx];
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for setting the sampling periods for different data types.
 */
struct data_period_t {
  uint8_t position;
  uint8_t velocity;
  uint8_t torque;
  uint8_t imu;
  uint8_t temperature;
} __attribute__((__packed__));

} // namespace plexus_sdk