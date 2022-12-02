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
#ifndef FUSE_CORE__THROTTLED_CALLBACK_HPP_
#define FUSE_CORE__THROTTLED_CALLBACK_HPP_

#include <functional>
#include <memory>
#include <utility>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace fuse_core
{

/**
 * @brief A throttled callback that encapsulates the logic to throttle a callback so it is only
 *        called after a given period in seconds (or more). The dropped calls can optionally be
 *        received in a dropped callback, that could be used to count the number of calls dropped.
 *
 * @tparam Callback The std::function callback
 */
template<class Callback>
class ThrottledCallback
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] keep_callback   The callback to call when kept, i.e. not dropped. Defaults to
   *                            nullptr
   * @param[in] drop_callback   The callback to call when dropped because of the throttling.
   *                            Defaults to nullptr
   * @param[in] throttle_period The throttling period duration in seconds. Defaults to 0.0, i.e. no
   *                            throttling
   * @param[in] clock           The clock to throttle against. Defaults to using RCL_SYSTEM_TIME
   */
  ThrottledCallback(
    Callback && keep_callback = nullptr,                 // NOLINT(whitespace/operators)
    Callback && drop_callback = nullptr,                 // NOLINT(whitespace/operators)
    const rclcpp::Duration & throttle_period = rclcpp::Duration(0, 0),
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>())
  : keep_callback_(keep_callback)
    , drop_callback_(drop_callback)
    , throttle_period_(throttle_period)
    , clock_(clock)
  {}

  /**
   * @brief Throttle period getter
   *
   * @return The current throttle period duration in seconds being used
   */
  const rclcpp::Duration & getThrottlePeriod() const
  {
    return throttle_period_;
  }

  /**
   * @brief Set the clock to throttle against
   *
   * @param[in] clock The clock to set
   */
  void setClock(rclcpp::Clock::SharedPtr clock)
  {
    clock_ = clock;

    // Reset since we changed the clock
    last_called_time_ = rclcpp::Time();
  }

  /**
   * @brief Throttle period setter
   *
   * @param[in] throttle_period The new throttle period duration in seconds to use
   */
  void setThrottlePeriod(const rclcpp::Duration & throttle_period)
  {
    throttle_period_ = throttle_period;
  }

  /**
   * @brief Keep callback setter
   *
   * @param[in] keep_callback The new keep callback to use
   */
  void setKeepCallback(const Callback & keep_callback)
  {
    keep_callback_ = keep_callback;
  }

  /**
   * @brief Drop callback setter
   *
   * @param[in] drop_callback The new drop callback to use
   */
  void setDropCallback(const Callback & drop_callback)
  {
    drop_callback_ = drop_callback;
  }

  /**
   * @brief Last called time
   *
   * @return The last time the keep callback was called
   */
  const rclcpp::Time & getLastCalledTime() const
  {
    return last_called_time_;
  }

  /**
   * @brief Callback that throttles the calls to the keep callback provided. When dropped because of
   *        throttling, the drop callback is called instead.
   *
   * @param[in] args The input arguments
   */
  template<class ... Args>
  void callback(Args &&... args)
  {
    // Keep the callback if:
    //
    // (a) This is the first call, i.e. the last called time is still invalid because it has not
    //     been set yet
    // (b) The throttle period is zero, so we should always keep the callbacks
    // (c) The elpased time between now and the last called time is greater than the throttle period
    rclcpp::Time now = clock_->now();

    if ((last_called_time_.nanoseconds() == 0) ||
      (throttle_period_.nanoseconds() == 0) ||
      now - last_called_time_ > throttle_period_)
    {
      if (keep_callback_) {
        keep_callback_(std::forward<Args>(args)...);
      }

      if (last_called_time_.nanoseconds() == 0) {
        last_called_time_ = now;
      } else {
        last_called_time_ += throttle_period_;
      }
    } else if (drop_callback_) {
      drop_callback_(std::forward<Args>(args)...);
    }
  }

  /**
   * @brief Operator() that simply calls the callback() method forwarding the input arguments
   *
   * @param[in] args The input arguments
   */
  template<class... Args>
  void operator()(Args &&... args)
  {
    callback(std::forward<Args>(args)...);
  }

private:
  Callback keep_callback_;         //!< The callback to call when kept, i.e. not dropped
  Callback drop_callback_;         //!< The callback to call when dropped because of throttling
  rclcpp::Duration throttle_period_;  //!< The throttling period duration in seconds
  rclcpp::Clock::SharedPtr clock_;  //!< The clock to throttle against
  rclcpp::Time last_called_time_;  //!< The last time the keep callback was called
};

/**
 * @brief Throttled callback for ROS messages
 *
 * @tparam M The ROS message type
 */
template<class M>
using ThrottledMessageCallback =
  ThrottledCallback<std::function<void (const M &)>>;

}  // namespace fuse_core

#endif  // FUSE_CORE__THROTTLED_CALLBACK_HPP_
