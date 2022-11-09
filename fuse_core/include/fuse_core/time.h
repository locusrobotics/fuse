/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Brett Downing
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


/*
 * TODO(CH3): Remove this file once the time changes are merged and released on rclcpp
 * https://github.com/ros2/rclcpp/pull/2040
 *
 * This file provisions the is_valid and wait_for_valid methods to check for non-zero time.
 */

#ifndef FUSE_CORE_TIME_H
#define FUSE_CORE_TIME_H

#include <chrono>
#include <iostream>
#include <stdexcept>

#include "rclcpp/contexts/default_context.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>


namespace fuse_core
{
// NOTE(CH3): These are snake case despite the rest of the codebase being camel-case because
//            they're anticipating rclcpp methods!!

/**
 * @brief Check if time is valid (non-zero)
 *
 * @param[in] time    An rclcpp::Time object to measure time against
 *
 * @return valid  true if time was or became valid
 */
// TOOD(CH3): Replace with rclcpp's implementation when https://github.com/ros2/rclcpp/pull/2040 is in
bool is_valid(rclcpp::Time time);


/**
 * @brief Check if clock's time is valid (non-zero)
 *
 * @param[in] clock    An rclcpp::Clock to measure time against
 *
 * @return valid  true if clock was or became valid
 */
// TOOD(CH3): Replace with rclcpp's implementation when https://github.com/ros2/rclcpp/pull/2040 is in
bool is_valid(rclcpp::Clock::SharedPtr clock);


/**
 * @brief Wait for clock's time to be valid (non-zero)
 *
 * @param[in] clock         An rclcpp::Clock to measure time against
 * @param[in] context       The context to wait in
 *
 * @return valid  true if clock was or became valid
 */
// TOOD(CH3): Replace with rclcpp's implementation when https://github.com/ros2/rclcpp/pull/2040 is in
bool wait_for_valid(
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context());


/**
 * @brief Wait for clock's time to be valid (non-zero), with timeout
 *
 * @param[in] clock         An rclcpp::Clock to measure time against
 * @param[in] timeout       The maximum time to wait for
 * @param[in] context       The context to wait in
 * @param[in] wait_tick_ns  The time to wait between each iteration of the wait loop (in ns)
 *
 * @return valid  true if clock was or became valid
 */
// TOOD(CH3): Replace with rclcpp's implementation when https://github.com/ros2/rclcpp/pull/2040 is in
bool wait_for_valid(
  rclcpp::Clock::SharedPtr clock,
  const rclcpp::Duration & timeout,
  rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context(),
  const rclcpp::Duration & wait_tick_ns = rclcpp::Duration(0, static_cast<uint32_t>(1e7)));

}

#endif  // FUSE_CORE_TIME_H
