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
#ifndef FUSE_CONSTRAINTS_UTIL_H
#define FUSE_CONSTRAINTS_UTIL_H

#include <ceres/jet.h>
#include <Eigen/Geometry>

#include <cmath>


namespace fuse_constraints
{

/**
 * @brief Wrap a 2D angle to the standard (-Pi, +Pi] range.
 *
 * @param[IN/OUT]  angle  Input angle to be wrapped to the (-Pi, +Pi] range. Angle is updated by this function.
 */
template<typename T>
void wrapAngle2D(T& angle)
{
  // Define some necessary variations of PI with the correct type (double or Jet)
  static const T PI = T(M_PI);
  static const T TAU = T(2 * M_PI);
  // Handle the 1*Tau roll-over (https://tauday.com/tau-manifesto)
  // Use ceres::floor because it is specialized for double and Jet types.
  angle -= TAU * ceres::floor((angle + PI) / TAU);
}

/**
 * @brief Create an 2x2 rotation matrix from an angle
 *
 * @param[IN]  angle  The rotation angle, in radians
 * @return            The equivalent 2x2 rotation matrix
 */
template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(const T angle)
{
  const T cos_angle = ceres::cos(angle);
  const T sin_angle = ceres::sin(angle);
  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_angle, -sin_angle, sin_angle, cos_angle;
  return rotation;
}

}  // namespace fuse_constraints

#endif  // FUSE_CONSTRAINTS_UTIL_H
