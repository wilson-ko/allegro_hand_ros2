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
 * @file plexus_spec.cpp
 * @brief Defines constants, message IDs, and data structures for the Allegro Hand Plexus protocol.
 * @author ('c')void
 */

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include "plexus_spec.hpp"

namespace plexus_sdk {

std::string data_info_t::to_string() {
  std::string s;

  s += fmt::format(" {:<20}: 0x{:X}\n", "Hand Version", (int)hand_version);
  s += fmt::format(" {:<20}: 0x{:X}\n", "Firmware Version", (int)firmware_version);
  s += fmt::format(" {:<20}: {}\n", "Hand Type", (int)hand_type == 0x01 ? "right" : "left");
  s += fmt::format(" {:<20}: {}\n", "Hub Temperature", (int)hub_temperature);
  s += fmt::format(" {:<20}: F1_Err={}, F2_Err={}, F3_Err={}, F4_Err={}, Volt_Err={}, Temp_Err={}, Shock_Err={}, Overload_Err={}",
                   "Joint Status", (int)joint_status.f4_connection_error, (int)joint_status.f3_connection_error,
                   (int)joint_status.f2_connection_error, (int)joint_status.f1_connection_error, (int)joint_status.input_voltage_error,
                   (int)joint_status.temperature_error, (int)joint_status.electrical_shock_error, (int)joint_status.overload_error);
  return s;
}

} // namespace plexus_sdk