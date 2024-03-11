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
#ifndef FUSE_CORE__UTIL_HPP_
#define FUSE_CORE__UTIL_HPP_

#include <ceres/jet.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

#include <cmath>
#include <string>

#include <rclcpp/logging.hpp>

namespace fuse_core
{

/**
 * @brief Returns the Euler pitch angle from a quaternion
 *
 * @param[in] w The quaternion real-valued component
 * @param[in] x The quaternion x-axis component
 * @param[in] y The quaternion x-axis component
 * @param[in] z The quaternion x-axis component
 * @return      The quaternion's Euler pitch angle component
 */
template<typename T>
static inline T getPitch(const T w, const T x, const T y, const T z)
{
  // Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const T sin_pitch = T(2.0) * (w * y - z * x);

  if (ceres::abs(sin_pitch) >= T(1.0)) {
    return (sin_pitch >= T(0.0) ? T(1.0) : T(-1.0)) * T(M_PI / 2.0);
  } else {
    return ceres::asin(sin_pitch);
  }
}

/**
 * @brief Returns the Euler roll angle from a quaternion
 *
 * @param[in] w The quaternion real-valued component
 * @param[in] x The quaternion x-axis component
 * @param[in] y The quaternion x-axis component
 * @param[in] z The quaternion x-axis component
 * @return      The quaternion's Euler roll angle component
 */
template<typename T>
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
 * @param[in] y The quaternion x-axis component
 * @param[in] z The quaternion x-axis component
 * @return      The quaternion's Euler yaw angle component
 */
template<typename T>
static inline T getYaw(const T w, const T x, const T y, const T z)
{
  // Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const T sin_yaw = T(2.0) * (w * z + x * y);
  const T cos_yaw = T(1.0) - (T(2.0) * (y * y + z * z));
  return ceres::atan2(sin_yaw, cos_yaw);
}

/**
 * @brief Wrap a 2D angle to the standard [-Pi, +Pi) range.
 *
 * @param[in/out] angle Input angle to be wrapped to the [-Pi, +Pi) range. Angle is updated by this
 *                      function.
 */
template<typename T>
void wrapAngle2D(T & angle)
{
  // Define some necessary variations of PI with the correct type (double or Jet)
  static const T PI = T(M_PI);
  static const T TAU = T(2 * M_PI);
  // Handle the 1*Tau roll-over (https://tauday.com/tau-manifesto)
  // Use ceres::floor because it is specialized for double and Jet types.
  angle -= TAU * ceres::floor((angle + PI) / TAU);
}

/**
 * @brief Wrap a 2D angle to the standard (-Pi, +Pi] range.
 *
 * @param[in] angle Input angle to be wrapped to the (-Pi, +Pi] range.
 * @return The equivalent wrapped angle
 */
template<typename T>
T wrapAngle2D(const T & angle)
{
  T wrapped = angle;
  wrapAngle2D(wrapped);
  return wrapped;
}

/**
 * @brief Create an 2x2 rotation matrix from an angle
 *
 * @param[in] angle The rotation angle, in radians
 * @return          The equivalent 2x2 rotation matrix
 */
template<typename T>
Eigen::Matrix<T, 2, 2, Eigen::RowMajor> rotationMatrix2D(const T angle)
{
  const T cos_angle = ceres::cos(angle);
  const T sin_angle = ceres::sin(angle);
  Eigen::Matrix<T, 2, 2, Eigen::RowMajor> rotation;
  rotation << cos_angle, -sin_angle, sin_angle, cos_angle;
  return rotation;
}

/**
 * @brief Compute roll, pitch, and yaw from a quaternion
 *
 * @param[in] q Pointer to the quaternion array (4x1)
 * @param[in] rpy Pointer to the roll, pitch, yaw array (3x1)
 * @param[in] jacobian Pointer to the jacobian matrix (3x4, optional)
 */
static inline void quaternion2rpy(const double * q, double * rpy, double * jacobian = nullptr) 
{
  rpy[0] = fuse_core::getRoll(q[0], q[1], q[2], q[3]);
  rpy[1] = fuse_core::getPitch(q[0], q[1], q[2], q[3]);
  rpy[2] = fuse_core::getYaw(q[0], q[1], q[2], q[3]);

  if (jacobian) {
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_map(jacobian);
    const double qw = q[0];
    const double qx = q[1];
    const double qy = q[2];
    const double qz = q[3];
    const double discr = qw * qy - qx * qz; 
    const double gl_limit = 0.5 - 2.0 * std::numeric_limits<double>::epsilon(); 

    if (discr > gl_limit) {
      // pitch = 90 deg
      jacobian_map.setZero();
      jacobian_map(2, 0) = (2.0 * qx) / (qw * qw * ((qx * qx / qw * qw) + 1.0));
      jacobian_map(2, 1) = -2.0 / (qw * ((qx * qx / qw * qw) + 1.0));
      return;
    } else if (discr < -gl_limit) {
      // pitch = -90 deg
      jacobian_map.setZero();
      jacobian_map(2, 0) = (-2.0 * qx) / (qw * qw * ((qx * qx / qw * qw) + 1.0));
      jacobian_map(2, 1) = 2.0 / (qw * ((qx * qx / qw * qw) + 1.0));
      return;
    } else {
      // Non-degenerate case:
      jacobian_map(0, 0) =
        -(2.0 * qx) /
        ((std::pow((2.0 * qw * qx + 2.0 * qy * qz), 2.0) / std::pow((2.0 * qx * qx + 2.0 * qy * qy - 1.0), 2.0) +
        1.0) *
        (2.0 * qx * qx + 2.0 * qy * qy - 1.0));
      jacobian_map(0, 1) =
        -((2.0 * qw) / (2.0 * qx * qx + 2.0 * qy * qy - 1.0) -
        (4.0 * qx * (2.0 * qw * qx + 2.0 * qy * qz)) / std::pow((2.0 * qx * qx + 2.0 * qy * qy - 1.0), 2.0)) /
        (std::pow((2.0 * qw * qx + 2.0 * qy * qz), 2.0) / std::pow((2.0 * qx * qx + 2.0 * qy * qy - 1.0), 2.0) + 1.0);
      jacobian_map(0, 2) =
        -((2.0 * qz) / (2.0 * qx * qx + 2.0 * qy * qy - 1.0) -
        (4.0 * qy * (2.0 * qw * qx + 2.0 * qy * qz)) / std::pow((2.0 * qx * qx + 2.0 * qy * qy - 1.0), 2.0)) /
        (std::pow((2.0 * qw * qx + 2.0 * qy * qz), 2.0) / std::pow((2.0 * qx * qx + 2.0 * qy * qy - 1.0), 2.0) + 1.0);
      jacobian_map(0, 3) =
        -(2.0 * qy) /
        ((std::pow((2.0 * qw * qx + 2.0 * qy * qz), 2.0) / std::pow((2.0 * qx * qx + 2.0 * qy * qy - 1.0), 2.0) +
        1.0) *
        (2.0 * qx * qx + 2.0 * qy * qy - 1.0));

      jacobian_map(1, 0) = (2.0 * qy) / std::sqrt(1.0 - std::pow((2.0 * qw * qy - 2.0 * qx * qz), 2.0));
      jacobian_map(1, 1) = -(2.0 * qz) / std::sqrt(1.0 - std::pow((2.0 * qw * qy - 2.0 * qx * qz), 2.0));
      jacobian_map(1, 2) = (2.0 * qw) / std::sqrt(1.0 - std::pow((2.0 * qw * qy - 2.0 * qx * qz), 2.0));
      jacobian_map(1, 3) = -(2.0 * qx) / std::sqrt(1.0 - std::pow((2.0 * qw * qy - 2.0 * qx * qz), 2.0));

      jacobian_map(2, 0) =
        -(2.0 * qz) /
        ((std::pow((2.0 * qw * qz + 2.0 * qx * qy), 2.0) / std::pow((2.0 * qy * qy + 2.0 * qz * qz - 1.0), 2.0) +
        1.0) *
        (2.0 * qy * qy + 2.0 * qz * qz - 1.0));
      jacobian_map(2, 1) =
        -(2.0 * qy) /
        ((std::pow((2.0 * qw * qz + 2.0 * qx * qy), 2.0) / std::pow((2.0 * qy * qy + 2.0 * qz * qz - 1.0), 2.0) +
        1.0) *
        (2.0 * qy * qy + 2.0 * qz * qz - 1.0));
      jacobian_map(2, 2) =
        -((2.0 * qx) / (2.0 * qy * qy + 2.0 * qz * qz - 1.0) -
        (4.0 * qy * (2.0 * qw * qz + 2.0 * qx * qy)) / std::pow((2.0 * qy * qy + 2.0 * qz * qz - 1.0), 2.0)) /
        (std::pow((2.0 * qw * qz + 2.0 * qx * qy), 2.0) / std::pow((2.0 * qy * qy + 2.0 * qz * qz - 1.0), 2.0) + 1.0);
      jacobian_map(2, 3) =
        -((2.0 * qw) / (2.0 * qy * qy + 2.0 * qz * qz - 1.0) -
        (4.0 * qz * (2.0 * qw * qz + 2.0 * qx * qy)) / std::pow((2.0 * qy * qy + 2.0 * qz * qz - 1.0), 2.0)) /
        (std::pow((2.0 * qw * qz + 2.0 * qx * qy), 2.0) / std::pow((2.0 * qy * qy + 2.0 * qz * qz - 1.0), 2.0) + 1.0);
    }
  }
}

/**
 * @brief Compute product of two quaternions and the function jacobian
 * TODO(giafranchini): parametric jacobian computation? Atm this function is only used in 
 * normal_prior_pose_3d cost function. There we only need the derivatives wrt quaternion W,
 * so at the time we are only computing the jacobian wrt W
 * 
 * @param[in] z Pointer to the first quaternion array (4x1)
 * @param[in] w Pointer to the second quaternion array (4x1)
 * @param[in] zw Pointer to the output quaternion array (4x1) that will be populated with the result of z * w
 * @param[in] jacobian Pointer to the jacobian of zw with respect to w (4x4, optional)
 */
static inline void quaternionProduct(const double * z, const double * w, double * zw, double * jacobian = nullptr) 
{
  ceres::QuaternionProduct(z, w, zw);
  if (jacobian) {
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> jacobian_map(jacobian);
    jacobian_map << 
      z[0], -z[1], -z[2], -z[3],
      z[1],  z[0], -z[3],  z[2],
      z[2],  z[3],  z[0], -z[1],
      z[3], -z[2],  z[1],  z[0];
  }
}

/**
 * @brief Compute quaternion to AngleAxis conversion and the function jacobian
 * 
 * @param[in] q Pointer to the quaternion array (4x1)
 * @param[in] angle_axis Pointer to the angle_axis array (3x1) 
 * @param[in] jacobian Pointer to the jacobian matrix (3x4, optional)
 */
static inline void quaternionToAngleAxis(const double * q, double * angle_axis, double * jacobian = nullptr) 
{
  ceres::QuaternionToAngleAxis(q, angle_axis);
  if (jacobian) {
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_map(jacobian);
    const double & q0 = q[0];
    const double & q1 = q[1];
    const double & q2 = q[2];
    const double & q3 = q[3];
    const double q_sum2 = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    const double sin_theta2 = q1 * q1 + q2 * q2 + q3 * q3;
    const double sin_theta = std::sqrt(sin_theta2);
    const double cos_theta = q0;

    if (std::fpclassify(sin_theta) != FP_ZERO) {
      const double two_theta = 2.0 * 
        (cos_theta < 0.0 ? std::atan2(-sin_theta, -cos_theta) : std::atan2(sin_theta, cos_theta));  
      jacobian_map(0, 0) = -2.0 * q1 / q_sum2;
      jacobian_map(0, 1) = 
        two_theta / sin_theta + 
        (2.0 * q0 * q1 * q1) / (sin_theta2 * q_sum2) - 
        (q1 * q1 * two_theta) / std::pow(sin_theta2, 1.5);
      jacobian_map(0, 2) = 
        (2.0 * q0 * q1 * q2) / (sin_theta2 * q_sum2) - 
        (q1 * q2 * two_theta) / std::pow(sin_theta2, 1.5);
      jacobian_map(0, 3) =
        (2.0 * q0 * q1 * q3) / (sin_theta2 * q_sum2) - 
        (q1 * q3 * two_theta) / std::pow(sin_theta2, 1.5);
      
      jacobian_map(1, 0) = -2.0 * q2 / q_sum2;
      jacobian_map(1, 1) = 
        (2.0 * q0 * q1 * q2) / (sin_theta2 * q_sum2) - 
        (q1 * q2 * two_theta) / std::pow(sin_theta2, 1.5);
      jacobian_map(1, 2) = 
        two_theta / sin_theta + 
        (2.0 * q0 * q2 * q2) / (sin_theta2 * q_sum2) - 
        (q2 * q2 * two_theta) / std::pow(sin_theta2, 1.5);
      jacobian_map(1, 3) = 
        (2.0 * q0 * q2 * q3) / (sin_theta2 * q_sum2) - 
        (q2 * q3 * two_theta) / std::pow(sin_theta2, 1.5);
      
      jacobian_map(2, 0) = -2.0 * q3 / q_sum2;
      jacobian_map(2, 1) = 
        (2.0 * q0 * q1 * q3) / (sin_theta2 * q_sum2) - 
        (q1 * q3 * two_theta) / std::pow(sin_theta2, 1.5);
      jacobian_map(2, 2) = 
        (2.0 * q0 * q2 * q3) / (sin_theta2 * q_sum2) - 
        (q2 * q3 * two_theta) / std::pow(sin_theta2, 1.5);
      jacobian_map(2, 3) =
        two_theta / sin_theta + 
        (2.0 * q0 * q3 * q3) / (sin_theta2 * q_sum2) - 
        (q3 * q3 * two_theta) / std::pow(sin_theta2, 1.5);
    } else {
      jacobian_map.setZero();
      jacobian_map(1, 1) = 2.0;
      jacobian_map(2, 2) = 2.0;
      jacobian_map(3, 3) = 2.0;
    }
  }
}

/**
 * @brief Create a compound ROS topic name from two components
 *
 * @param[in] a The first element
 * @param[in] b The second element
 * @return      The joined topic name
 */
inline
std::string joinTopicName(std::string a, std::string b)
{
  if (a.empty()) {
    return b;
  }
  if (b.empty()) {
    return a;
  }
  if (b.front() == '/' || b.front() == '~') {
    RCLCPP_WARN(
      rclcpp::get_logger("fuse"), "Second argument to joinTopicName is absolute! Returning it.");
    return b;
  }

  if (a.back() == '/') {
    a.pop_back();
  }

  return a + "/" + b;
}

}  // namespace fuse_core

#endif  // FUSE_CORE__UTIL_HPP_
