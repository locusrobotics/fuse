/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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
#ifndef FUSE_CORE_PARAMETER_H
#define FUSE_CORE_PARAMETER_H

#include <fuse_core/eigen.h>
#include <fuse_core/loss_loader.h>

#include <ros/node_handle.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace fuse_core
{

/**
 * @brief Utility method for handling required ROS params
 *
 * @param[in] nh - The ROS node handle with which to load parameters
 * @param[in] key - The ROS parameter key for the required parameter
 * @param[out] value - The ROS parameter value for the \p key
 * @throws std::runtime_error if the parameter does not exist
 */
template <typename T>
void getParamRequired(const ros::NodeHandle& nh, const std::string& key, T& value)  // TODO(CH3): Replace nh with a node
{
  if (!nh.getParam(key, value))
  {
    const std::string error = "Could not find required parameter " + key + " in namespace " + nh.getNamespace();
    RCLCPP_FATAL_STREAM(node->get_logger(), error);
    throw std::runtime_error(error);
  }
}

/**
 * @brief Helper function that loads positive integral or floating point values from the parameter server
 *
 * @param[in] node_handle - The node handle used to load the parameter
 * @param[in] parameter_name - The parameter name to load
 * @param[in, out] default_value - A default value to use if the provided parameter name does not exist. As output it
 *                                 has the loaded (or default) value
 * @param[in] strict - Whether to check the loaded value is strictly positive or not, i.e. whether 0 is accepted or not
 */
template <typename T,
          typename = std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value>>
void getPositiveParam(const ros::NodeHandle& node_handle, const std::string& parameter_name, T& default_value,
                      const bool strict = true)  // TODO(CH3): Replace nh with a node
{
  T value;
  node_handle.param(parameter_name, value, default_value);
  if (value < 0 || (strict && value == 0))
  {
    RCLCPP_WARN_STREAM(node->get_logger(),
                       "The requested " << parameter_name << " is <" << (strict ? "=" : "")
                       << " 0. Using the default value (" << default_value << ") instead.");
  }
  else
  {
    default_value = value;
  }
}

/**
 * @brief Helper function that loads positive duration values from the parameter server
 *
 * @param[in] node_handle - The node handle used to load the parameter
 * @param[in] parameter_name - The parameter name to load
 * @param[in, out] default_value - A default value to use if the provided parameter name does not exist. As output it
 *                                 has the loaded (or default) value
 * @param[in] strict - Whether to check the loaded value is strictly positive or not, i.e. whether 0 is accepted or not
 */
inline void getPositiveParam(const ros::NodeHandle& node_handle, const std::string& parameter_name,
                             rclcpp::Duration& default_value, const bool strict = true)
{
  double default_value_sec = default_value.seconds();
  getPositiveParam(node_handle, parameter_name, default_value_sec, strict);
  default_value.fromSec(default_value_sec);
}

/**
 * @brief Helper function that loads a covariance matrix diagonal vector from the parameter server and checks the size
 * and the values are invalid, i.e. they are positive.
 *
 * @tparam Scalar - A scalar type, defaults to double
 * @tparam Size - An int size that specifies the expected size of the covariance matrix (rows and columns)
 *
 * @param[in] node_handle - The node handle used to load the parameter
 * @param[in] parameter_name - The parameter name to load
 * @param[in] default_value - A default value to use for all the diagonal elements if the provided parameter name does
 *                            not exist
 * @return The loaded (or default) covariance matrix, generated from the diagonal vector
 */
template <int Size, typename Scalar = double>
fuse_core::Matrix<Scalar, Size, Size> getCovarianceDiagonalParam(const ros::NodeHandle& node_handle,
                                                                 const std::string& parameter_name,
                                                                 Scalar default_value)
{
  using Vector = typename Eigen::Matrix<Scalar, Size, 1>;

  std::vector<Scalar> diagonal(Size, default_value);
  node_handle.param(parameter_name, diagonal, diagonal);

  const auto diagonal_size = diagonal.size();
  if (diagonal_size != Size)
  {
    throw std::invalid_argument("Invalid size of " + std::to_string(diagonal_size) + ", expected " +
                                std::to_string(Size));
  }

  if (std::any_of(diagonal.begin(), diagonal.end(),
                  [](const auto& value) { return value < Scalar(0); }))  // NOLINT(whitespace/braces)
  {
    throw std::invalid_argument("Invalid negative diagonal values in " + fuse_core::to_string(Vector(diagonal.data())));
  }

  return Vector(diagonal.data()).asDiagonal();
}

/**
 * @brief Utility method to load a loss configuration
 *
 * @param[in] nh - The ROS node handle with which to load parameters
 * @param[in] name - The ROS parameter name for the loss configuration parameter
 * @return Loss function or nullptr if the parameter does not exist
 */
inline fuse_core::Loss::SharedPtr loadLossConfig(const ros::NodeHandle& nh, const std::string& name)
{
  if (!nh.hasParam(name))
  {
    return {};
  }

  std::string loss_type;
  getParamRequired(nh, name + "/type", loss_type);

  auto loss = fuse_core::createUniqueLoss(loss_type);
  loss->initialize(nh.resolveName(name));

  return loss;
}

}  // namespace fuse_core

#endif  // FUSE_CORE_PARAMETER_H
