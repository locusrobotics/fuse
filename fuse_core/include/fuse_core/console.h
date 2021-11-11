/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FUSE_CORE_CONSOLE_H
#define FUSE_CORE_CONSOLE_H

#include <chrono>

namespace fuse_core
{

/**
 * @brief a log filter that provides a condition to RCLCPP_*_STREAM_EXPRESSION and allows to reset the last time the
 * message was print, so the delayed and throttle conditions are computed from the initial state again.
 */
class DelayedThrottleFilter
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] period The throttle period in seconds
   */
  explicit DelayedThrottleFilter(const double period = 0) :
  reset_(true),
  period_(std::chrono::seconds<double>(period))
  {
  }

  /**
   * @brief Returns whether or not the log statement should be printed. Called before the log arguments are evaluated
   * and the message is formatted.
   *
   * This works as ROS_*_DELAYED_THROTTLE but the last time the filter condition was hit is handled by this filter, so
   * it can be reset.
   *
   * @return True if the filter condition is hit, false otherwise
   */
  bool isEnabled() override
  {
    const auto now = std::chrono::system_clock::now();

    if (reset_)
    {
      reset = false;
      last_hit_ = now;
      return true;
    }

    if (now > (last_hit_ + period_))
    {
      last_hit_ = now;
      return true;
    }

    return false;
  }

  /**
   * @brief Rests the last time the filter condition was hit
   */
  void reset()
  {
    reset_=true;
  }

private:
  bool reset_;
  std::chrono::duration period_;     //!< The throttle period in seconds
  std::chrono::time_point<std::chrono::system_clock> last_hit_;  //!< The last time in seconds the filter condition was hit, and the message was printed. A
                             //!< negative value means it has never been hit
};

}  // namespace fuse_core

#endif  // FUSE_CORE_CONSOLE_H
