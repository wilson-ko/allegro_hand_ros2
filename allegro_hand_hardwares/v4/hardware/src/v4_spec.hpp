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
 * @file v4_spec.hpp
 * @brief Defines constants, message IDs, and data structures for the Allegro Hand V4 protocol.
 * @author ('c')void
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace v4_sdk {

/** @brief Maximum number of fingers on the hand. */
constexpr int FINGER_MAX = 4;
/** @brief Number of joints per finger. */
constexpr int JONIT_PER_FINGER = 4;
/** @brief Total number of joints on the hand. */
constexpr int JONIT_MAX = FINGER_MAX * JONIT_PER_FINGER;
/** @brief Maximum number of hand IDs supported. */
constexpr int HAND_ID_MAX = 8; // Not currently used, but defined for protocol specification.

/** @brief Default sampling period for position data in seconds (3ms). */
constexpr double POSITION_SAMPLING_PERIOD = 3.0 / 1000;
/** @brief Default sampling period for temperature data in seconds (0.5s). */
constexpr double TEMPERATURE_SAMPLING_PERIOD = 0.5;
/** @brief Default cutoff frequency for the low-pass filter in Hz. */
constexpr double LOWPASS_FILTER_CUTOFF_FREQ = 30.0;

// PWM duty cycle limits for each joint type to prevent hardware damage.
constexpr double PWM_LIMIT_ROLL = 250.0 * 1.5;
constexpr double PWM_LIMIT_NEAR = 450.0 * 1.5;
constexpr double PWM_LIMIT_MIDDLE = 300.0 * 1.5;
constexpr double PWM_LIMIT_FAR = 190.0 * 1.5;

constexpr double PWM_LIMIT_THUMB_ROLL = 350.0 * 1.5;
constexpr double PWM_LIMIT_THUMB_NEAR = 270.0 * 1.5;
constexpr double PWM_LIMIT_THUMB_MIDDLE = 180.0 * 1.5;
constexpr double PWM_LIMIT_THUMB_FAR = 180.0 * 1.5;

constexpr double PWM_LIMIT_GLOBAL_8V = 800.0; // maximum: 1200
constexpr double PWM_LIMIT_GLOBAL_24V = 500.0;
constexpr double PWM_LIMIT_GLOBAL_12V = 1200.0;

constexpr double PWM_LIMIT_GLOBAL = PWM_LIMIT_GLOBAL_12V;
constexpr double TORQUE2PWM = PWM_LIMIT_GLOBAL;
constexpr double PWM2TORQUE = 1.0 / TORQUE2PWM;

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
  ID_CMD_SYSTEM_ON = 0x40,
  ID_CMD_SYSTEM_OFF = 0x41,
  ID_CMD_SET_TORQUE = 0x60,
  ID_CMD_SET_TORQUE_1 = ID_CMD_SET_TORQUE + 0,
  ID_CMD_SET_TORQUE_2 = ID_CMD_SET_TORQUE + 1,
  ID_CMD_SET_TORQUE_3 = ID_CMD_SET_TORQUE + 2,
  ID_CMD_SET_TORQUE_4 = ID_CMD_SET_TORQUE + 3,
  ID_CMD_SET_POSE = 0xE0,
  ID_CMD_SET_POSE_1 = ID_CMD_SET_POSE + 0,
  ID_CMD_SET_POSE_2 = ID_CMD_SET_POSE + 1,
  ID_CMD_SET_POSE_3 = ID_CMD_SET_POSE + 2,
  ID_CMD_SET_POSE_4 = ID_CMD_SET_POSE + 3,
  ID_CMD_SET_PERIOD = 0x81,
  ID_CMD_CONFIG = 0x68,

  //  Define CAN Data Reqeust (RTR)
  ID_RTR_HAND_INFO = 0x80,
  ID_RTR_SERIAL = 0x88,
  ID_RTR_FINGER_POSE = 0x20,
  ID_RTR_FINGER_POSE_1 = ID_RTR_FINGER_POSE + 0,
  ID_RTR_FINGER_POSE_2 = ID_RTR_FINGER_POSE + 1,
  ID_RTR_FINGER_POSE_3 = ID_RTR_FINGER_POSE + 2,
  ID_RTR_FINGER_POSE_4 = ID_RTR_FINGER_POSE + 3,
  ID_RTR_IMU_DATA = 0x30,
  ID_RTR_TEMPERATURE = 0x38,
  ID_RTR_TEMPERATURE_1 = ID_RTR_TEMPERATURE + 0,
  ID_RTR_TEMPERATURE_2 = ID_RTR_TEMPERATURE + 1,
  ID_RTR_TEMPERATURE_3 = ID_RTR_TEMPERATURE + 2,
  ID_RTR_TEMPERATURE_4 = ID_RTR_TEMPERATURE + 3
};

/**
 * @brief Structure for constructing the CAN frame identifier.
 *
 * This bitfield defines the layout for a 32-bit integer that is used to create
 * the CAN ID for a frame. It is specific to the CAN driver's API.
 */
struct can_id_t {
  uint32_t rfu : 2;         ///< Reserved for future use.
  uint32_t message_id : 28; ///< The core message identifier from the `MessageId` enum.
  uint32_t rtr : 1;         ///< Remote Transmission Request (RTR) flag. 1 for RTR, 0 for data frame.
  uint32_t extended : 1;    ///< Extended frame format flag (should be 1 for 29-bit IDs).
};

/**
 * @brief Data structure for hand information received from the device.
 *
 * This structure holds general information about the hand, such as version,
 * type, and status.
 */
struct data_info_t {
  uint8_t hardware_version_low;
  uint8_t hardware_version_high;
  uint8_t firmware_version_low;
  uint8_t firmware_version_high;
  uint8_t hand_type;
  uint8_t temperature;
  union {
    uint8_t status;
    struct {
      uint8_t servo_on : 1;
      uint8_t high_temperature_fault : 1;
      uint8_t internal_communication_fault : 1;
      uint8_t rfu : 5; // Reserved
    } bits;
  };
  std::string to_string() {
    return fmt::format("  - Hardware Version: 0x{:02x}{:02x}\n"
                       "  - Firmware Version: 0x{:02x}{:02x}\n"
                       "  - Hand Type: {} ({})\n"
                       "  - Temperature: {} 'C\n"
                       "  - Status: 0x{:02x} (Servo: {}, Temp Fault: {}, Comm Fault: {})", //
                       hardware_version_high, hardware_version_low, firmware_version_high, firmware_version_low, hand_type,
                       (hand_type == 0 ? "right" : "left"), temperature, status, (bits.servo_on ? "ON" : "OFF"),
                       (bits.high_temperature_fault ? "ON" : "OFF"), (bits.internal_communication_fault ? "ON" : "OFF"));
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for the hand's serial number.
 */
struct data_serial_t {
  uint8_t data[8]; // ASCII
  std::string to_string() { return fmt::format("SAH040{}", (const char*)data); }
} __attribute__((__packed__));

/**
 * @brief Data structure for sending joint torque commands.
 *
 * The values are raw PWM counts, not physical torque units.
 */
struct data_torque_t {
  int16_t joint_pwm[JONIT_PER_FINGER]; ///< PWM command for each of the 4 joints in a finger.
  inline void encode(int joint_idx, float torque) {
    assert(joint_idx < JONIT_PER_FINGER);
    joint_pwm[joint_idx] = TORQUE2PWM * torque;
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for joint position data (both sending and receiving).
 */
struct data_position_t {
  int16_t joint[JONIT_PER_FINGER]; ///< Raw encoder counts for each of the 4 joints in a finger.
  static constexpr double scale_factor_ = (333.3 / 65536.0) * (M_PI / 180.0);
  /** @brief Sets the raw encoder value from a physical angle in radians. */
  inline void encode(int joint_idx, double value) {
    // value := radian
    assert(joint_idx < JONIT_PER_FINGER);
    joint[joint_idx] = value / scale_factor_;
  }
  inline float decode(int joint_idx) const {
    assert(joint_idx < JONIT_PER_FINGER);
    return joint[joint_idx] * scale_factor_;
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for receiving joint temperature values.
 */
struct data_temperature_t {
  uint8_t joint_temperature[JONIT_PER_FINGER]; ///< Temperature in Celsius for each of the 4 joints in a finger.
  /** @brief Gets the temperature for a specific joint. */
  inline uint8_t decode(int joint_idx) const {
    assert(joint_idx < JONIT_PER_FINGER);
    return joint_temperature[joint_idx];
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for receiving IMU roll, pitch, and yaw values. (TEST PROTOCOL)
 */
struct data_imu_rpy_t {
  uint16_t rpy[3]; ///< Raw roll, pitch, yaw data.
  /** @brief Gets the raw value for a specific axis. */
  inline float decode(int idx) {
    assert(idx < 3);
    return rpy[idx];
  }
} __attribute__((__packed__));

/**
 * @brief Data structure for setting the sampling periods for different data types.
 */
struct data_period_t {
  uint16_t position;    // msec
  uint16_t imu;         // msec
  uint16_t temperature; // msec
} __attribute__((__packed__));

} // namespace v4_sdk