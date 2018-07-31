/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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
#ifndef FUSE_VARIABLES_UTIL_H
#define FUSE_VARIABLES_UTIL_H

#include <ceres/jet.h>

#include <cmath>


namespace fuse_variables
{

/**
 * @brief Returns the Euler pitch angle from a quaternion
 *
 * @param[in] w The quaternion real-valued component
 * @param[in] x The quaternion x-axis component
 * @param[in] y The quaternion y-axis component
 * @param[in] z The quaternion z-axis component
 * @return      The quaternion's Euler pitch angle component
 */
template <typename T>
static inline T getPitch(const T w, const T x, const T y, const T z)
{
  // Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const T sin_pitch = T(2.0) * (w * y - z * x);

  if (ceres::abs(sin_pitch) >= T(1.0))
  {
    return (sin_pitch >= T(0.0) ? T(1.0) : T(-1.0)) * T(M_PI / 2.0);
  }
  else
  {
    return ceres::asin(sin_pitch);
  }
}

/**
 * @brief Returns the Euler roll angle from a quaternion
 *
 * @param[in] w The quaternion real-valued component
 * @param[in] x The quaternion x-axis component
 * @param[in] y The quaternion y-axis component
 * @param[in] z The quaternion z-axis component
 * @return      The quaternion's Euler pitch angle component
 */
template <typename T>
static inline T getRoll(const T w, const T x, const T y, const T z)
{
  // Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const T sin_roll = T(2.0) * (w * x + y * z);
  const T cos_roll = T(1.0) - (T(2.0) * (x * x + y * y));
  return ceres::atan2(sin_roll, cos_roll);
}

/**
 * @brief Returns the Euler yaw angle from a quaternion
 *
 * @param[in] w The quaternion real-valued component
 * @param[in] x The quaternion x-axis component
 * @param[in] y The quaternion y-axis component
 * @param[in] z The quaternion z-axis component
 * @return      The quaternion's Euler pitch angle component
 */
template <typename T>
static inline T getYaw(const T w, const T x, const T y, const T z)
{
  // Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const T sin_yaw = T(2.0) * (w * z + x * y);
  const T cos_yaw = T(1.0) - (T(2.0) * (y * y + z * z));
  return ceres::atan2(sin_yaw, cos_yaw);
}

}  // namespace fuse_variables

#endif  // FUSE_VARIABLES_UTIL_H
