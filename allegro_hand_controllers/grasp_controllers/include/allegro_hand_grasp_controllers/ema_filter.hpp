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

#include <iostream>
#include <numeric> // For std::accumulate
#include <vector>

/**
 * @class ExponentialMovingAverageFilter
 * @brief Implements a buffer-less moving average filter using an exponential moving average (EMA) algorithm.
 *
 * This filter provides data smoothing similar to a simple moving average but with
 * O(1) memory complexity, as it does not require a buffer to store previous samples.
 */
class ExponentialMovingAverageFilter {
public:
  /**
   * @brief Constructor that takes the smoothing factor alpha.
   * @param alpha The smoothing factor (0 < alpha <= 1). A higher value gives more weight to recent data.
   */
  explicit ExponentialMovingAverageFilter(double alpha) : alpha_(alpha), initialized_(false), previousAverage_(0.0) {
    if (alpha_ <= 0.0 || alpha_ > 1.0) {
      throw std::invalid_argument("Alpha must be between 0 and 1.");
    }
  }

  /**
   * @brief Convenience constructor that takes the number of samples N to approximate a Simple Moving Average.
   * @param N The number of samples for an equivalent Simple Moving Average window.
   */
  explicit ExponentialMovingAverageFilter(int N) : initialized_(false), previousAverage_(0.0) {
    if (N <= 0) {
      throw std::invalid_argument("Number of samples (N) must be positive.");
    }
    // Common approximation to relate N from SMA to alpha for EMA
    alpha_ = 2.0 / (N + 1.0);
  }

  /**
   * @brief Processes a new sample and returns the filtered value.
   * @param newSample The new data point to filter.
   * @return The new calculated moving average.
   */
  double filter(double newSample) {
    if (!initialized_) {
      // For the first sample, the average is the sample itself.
      previousAverage_ = newSample;
      initialized_ = true;
    } else {
      // Apply the EMA formula
      previousAverage_ = (alpha_ * newSample) + (1.0 - alpha_) * previousAverage_;
    }
    return previousAverage_;
  }

  /**
   * @brief Resets the filter to its initial state.
   */
  void reset() {
    initialized_ = false;
    previousAverage_ = 0.0;
  }

private:
  double alpha_;           // Smoothing factor
  bool initialized_;       // Flag to check if the filter has been seeded
  double previousAverage_; // Stores the last calculated average
};
