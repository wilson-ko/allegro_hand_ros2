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
 * @file comm.cpp
 * @brief Implements the communication interface classes and the factory method for creating them.
 * @author ('c')void
 */

#include <chrono>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <spdlog/spdlog.h>

#include <allegro_hand_utility/utility.hpp>

namespace allegro_hand_utility {

/**
 * @see SpinLock::lock() in comm.hpp
 */
void SpinLock::lock() {
  // Loop until test_and_set() returns false (i.e., until the lock is acquired).
  while (flag_.test_and_set(std::memory_order_acquire)) {
#if __cplusplus >= 202002L
    // In C++20 and later, use wait() to reduce CPU load.
    flag_.wait(true, std::memory_order_relaxed);
#else
    // Before C++20, provide a hint to the CPU (useful on some architectures).
    // __builtin_ia32_pause(); // for x86/x64
#endif
  }
}

/**
 * @see SpinLock::unlock() in comm.hpp
 */
void SpinLock::unlock() { flag_.clear(std::memory_order_release); }

/**
 * @see LowpassFilter::LowpassFilter(double, double) in comm.hpp
 */
LowpassFilter::LowpassFilter(double sampling_frequency, double cutoff_frequency) {
  // Calculate coefficients using the Tustin's method (Bilinear Transform)
  double K = std::tan(M_PI * cutoff_frequency / sampling_frequency);

  a1_ = (1.0 - K) / (1.0 + K);
  b0_ = K / (1.0 + K);
  b1_ = K / (1.0 + K);

  // Initialize state variables
  v_prv_ = 0.0;
  v_filtered_ = 0.0;
}

/**
 * @see LowpassFilter::filtering(double) in comm.hpp
 */
double LowpassFilter::filtering(double v) {
  // Calculate the current filtered output, y[n]
  // y[n] = a1*y[n-1] + b0*x[n] + b1*x[n-1]
  double current_filtered_v = a1_ * v_filtered_ + b0_ * v + b1_ * v_prv_;

  // Update the state variables for the next iteration
  v_prv_ = v;
  v_filtered_ = current_filtered_v;

  return v_filtered_;
}

/**
 * @see DerivativeFilter::DerivativeFilter(double) in comm.hpp
 */
DerivativeFilter::DerivativeFilter(double sampling_frequency) : v_prv(0), v_dot(0), dt_(1.0 / sampling_frequency) { assert(dt_ > 0.0); }

/**
 * @see DerivativeFilter::filtering(double) in comm.hpp
 */
double DerivativeFilter::filtering(double v) {
  if (dt_ == 0) {
    return 0; // Avoid division by zero
  }
  v_dot = (v - v_prv) / dt_;
  v_prv = v;
  return v_dot;
}

SignalRateMonitor::SignalRateMonitor(size_t window_size) : window_size_(window_size) {}
SignalRateMonitor::~SignalRateMonitor() = default;

void SignalRateMonitor::tick(std::string signal_name) {
  const std::lock_guard<SpinLock> lock(lock_);

  const auto now = std::chrono::high_resolution_clock::now();
  auto& context = signals_[signal_name]; // Creates a new context if the signal name doesn't exist.

  if (context.count == 0) {
    // For the first tick, just record the start time.
    context.last_tick_time = now;
    context.count = 1;
    return;
  }

  const std::chrono::duration<double> period_duration = now - context.last_tick_time;
  const double period = period_duration.count();
  context.last_tick_time = now;
  context.count++;

  // Welford's online algorithm for moving window variance
  double old_mean = context.mean_period;
  size_t n = context.periods.size();

  if (n < window_size_) {
    // Window not full yet, standard Welford's algorithm
    n++;
    double delta = period - old_mean;
    context.mean_period += delta / n;
    double delta2 = period - context.mean_period;
    context.m2 += delta * delta2;
  } else {
    // Window is full, remove oldest and add newest
    double oldest_period = context.periods.front();
    context.periods.pop_front();

    double delta = period - oldest_period;
    context.mean_period += delta / n;
    double delta2 = period - context.mean_period;
    double delta3 = oldest_period - old_mean;
    context.m2 += delta * (delta2 + delta3);
  }
  context.periods.push_back(period);
}

std::map<std::string, SignalRateMonitor::RateInfo> SignalRateMonitor::get_info() const {
  const std::lock_guard<SpinLock> lock(lock_);

  std::map<std::string, RateInfo> all_info;
  for (const auto& pair : signals_) {
    const auto& context = pair.second;
    all_info[pair.first].count = context.count;
    if (context.count < 2) {
      all_info[pair.first].frequency = 0.0; // Not enough data
      all_info[pair.first].jitter = 0.0;    // Not enough data
      continue;
    }

    // Check if the signal has timed out.
    // If the time since the last tick is more than 10 times the mean period,
    // consider the signal to be stopped.
    const auto now = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> elapsed_since_last_tick = now - context.last_tick_time;
    if (context.mean_period > 0 && elapsed_since_last_tick.count() > context.mean_period * 10.0) {
      // Signal timed out, frequency is 0.
      all_info[pair.first].frequency = 0.0; // Not enough data
      all_info[pair.first].jitter = 0.0;    // Not enough data
      continue;
    }

    size_t n = context.periods.size();
    const double frequency = (context.mean_period > 0.0) ? 1.0 / context.mean_period : 0.0;
    const double variance = (n > 1) ? context.m2 / (n - 1) : 0.0; // Sample variance
    const double jitter = std::sqrt(variance);

    all_info[pair.first].frequency = frequency;
    all_info[pair.first].jitter = jitter;
  }
  return all_info;
}

std::string SignalRateMonitor::to_string() const {
  std::stringstream ss;
  ss << "Signal Rate Monitor Stats:\n";
  ss << "--------------------------------------------------\n";
  ss << fmt::format("{:<20} | {:>10} | {:>12}\n", "Signal Name", "Freq (Hz)", "Jitter (ms)");
  ss << "--------------------------------------------------\n";

  // get_info()를 호출하여 중복 코드를 피하고 일관성을 유지합니다.
  // get_info()는 이미 thread-safe하므로 to_string()에서 별도의 잠금이 필요 없습니다.
  auto all_info = get_info();

  if (all_info.empty()) {
    ss << "No signals monitored yet.\n";
  } else {
    // map은 키를 기준으로 자동 정렬되므로 일관된 순서로 출력됩니다.
    for (const auto& pair : all_info) {
      const auto& signal_name = pair.first;
      const auto& info = pair.second;
      ss << fmt::format("{:<20} | {:>10.2f} | {:>12.4f}\n", signal_name, info.frequency, info.jitter * 1000.0); // Jitter를 ms 단위로 표시
    }
  }
  ss << "--------------------------------------------------\n";
  return ss.str();
}

} //  namespace allegro_hand_utility