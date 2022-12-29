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
#ifndef FUSE_CORE__CONSOLE_HPP_
#define FUSE_CORE__CONSOLE_HPP_

#include <chrono>

namespace fuse_core
{

// To replicate ROS 1 behavior, the throttle checking conditions were adapted from the logic here:
// https://github.com/ros/rosconsole/blob/c9503279e932a04b3d2667cca3d28a8133cacc22/include/ros/console.h
#if defined(_MSC_VER)
  #define FUSE_LIKELY(x)       (x)
  #define FUSE_UNLIKELY(x)     (x)
#else
  #define FUSE_LIKELY(x)       __builtin_expect((x), 1)
  #define FUSE_UNLIKELY(x)     __builtin_expect((x), 0)
#endif

/**
 * @brief a log filter that provides a condition to RCLCPP_*_STREAM_EXPRESSION and allows to reset
 *        the last time the message was print, so the delayed and throttle conditions are computed
 *        from the initial state again.
 */
class DelayedThrottleFilter
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] The throttle period in seconds
   */
  explicit DelayedThrottleFilter(const double period)
  : period_(std::chrono::duration<double, std::ratio<1>>(period))
  {
    reset();
  }

  /**
   * @brief Returns whether or not the log statement should be printed. Called before the log
   *        arguments are evaluated and the message is formatted.
   *
   * This borrows logic from ROS 1's delayed throttle logging, but the last time the filter
   * condition was hit is handled by this filter, so it can be reset.
   *
   * @param[in] now - The current ROS time at which to check if logging should fire
   *
   * @return True if the filter condition is hit (signalling that the message should print), false
   *         otherwise
   */
  bool isEnabled()
  {
    const auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now());

    if (last_hit_.time_since_epoch().count() < 0.0) {
      last_hit_ = now;
      return true;
    }

    if (FUSE_UNLIKELY(last_hit_ + period_ <= now) || FUSE_UNLIKELY(now < last_hit_)) {
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
    last_hit_ = std::chrono::time_point_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::from_time_t(-1));
  }

private:
  std::chrono::duration<double, std::ratio<1>> period_;  //!< The throttle period in seconds

  //!< The last time in milliseconds the filter condition was hit, and the message was printed. A
  //!< negative value means it has never been hit
  std::chrono::time_point<std::chrono::system_clock> last_hit_;
};

}  // namespace fuse_core

#endif  // FUSE_CORE__CONSOLE_HPP_
