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
#ifndef FUSE_CORE__PARAMETER_HPP_
#define FUSE_CORE__PARAMETER_HPP_

#include <map>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <fuse_core/eigen.hpp>
#include <fuse_core/loss_loader.hpp>
#include <fuse_core/node_interfaces/node_interfaces.hpp>
#include <rclcpp/logging.hpp>
#include "rclcpp/parameter.hpp"

namespace fuse_core
{

// NOTE(CH3): Some of these basically mimic the behavior from rclcpp's node.hpp, but for interfaces

/**
 * @brief Compatibility wrapper for ros2 params in ros1 syntax
 *
 * Declare a parameter if not declared, otherwise, get its value.
 *
 * This is needed because the node parameters interface does not do the type conversions to and
 * from ParameterValue for us.
 *
 * @param[in] interfaces - The node interfaces used to load the parameter
 * @param[in] parameter_name - The ROS parameter name
 * @param[out] default_value - The default value for this parameter
 * @param[in] parameter_descriptor - An optional, custom description for the parameter.
 * @param[in] ignore_override When `true`, the parameter override is ignored. Default to `false`.
 * @return The value of the parameter.
 * @throws rclcpp::exceptions::InvalidParameterTypeException if the parameter type does not match
 */
template<class T>
T getParam(
  node_interfaces::NodeInterfaces<node_interfaces::Parameters> interfaces,
  const std::string & parameter_name,
  const T & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor(),
  bool ignore_override = false)
{
  auto params_interface = interfaces.get_node_parameters_interface();
  if (params_interface->has_parameter(parameter_name)) {
    return params_interface->get_parameter(parameter_name).get_parameter_value().get<T>();
  } else {
    try {
      return params_interface->declare_parameter(
        parameter_name, rclcpp::ParameterValue(default_value), parameter_descriptor, ignore_override
      ).get<T>();
    } catch (const rclcpp::ParameterTypeException & ex) {
      throw rclcpp::exceptions::InvalidParameterTypeException(parameter_name, ex.what());
    }
  }
}


/**
 * @brief Compatibility wrapper for ros2 params in ros1 syntax
 *
 * Declare a parameter if not declared, otherwise, get its value.
 *
 * This is needed because the node parameters interface does not do the type conversions to and
 * from ParameterValue for us.
 *
 * @param[in] interfaces - The node interfaces used to load the parameter
 * @param[in] parameter_name - The ROS parameter name
 * @param[in] parameter_descriptor - An optional, custom description for the parameter.
 * @param[in] ignore_override When `true`, the parameter override is ignored. Default to `false`.
 * @return The value of the parameter.
 * @throws rclcpp::exceptions::InvalidParameterTypeException if the parameter type does not match
 */
template<class T>
T getParam(
  node_interfaces::NodeInterfaces<node_interfaces::Parameters> interfaces,
  const std::string & parameter_name,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor(),
  bool ignore_override = false)
{
  // get advantage of parameter value template magic to get the correct rclcpp::ParameterType from T
  // NOTE(CH3): For the same reason we can't defer to the overload of getParam
  rclcpp::ParameterValue value{T{}};
  try {
    return interfaces.get_node_parameters_interface()->declare_parameter(
      parameter_name, value.get_type(), parameter_descriptor, ignore_override
    ).get<T>();
  } catch (const rclcpp::ParameterTypeException & ex) {
    throw rclcpp::exceptions::InvalidParameterTypeException(parameter_name, ex.what());
  }
}

namespace detail
{
/** @brief Internal function for unit testing.
 * @internal
*/
std::unordered_set<std::string>
list_parameter_override_prefixes(
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  std::string prefix);
}  // namespace detail

/**
 * @brief Get parameter overrides that have a given prefix
 *
 * Example:
 *  Say the given parameter overrides are foo, foo.baz, foo.bar.baz, and foobar.baz
 *  Given prefix "", this will return foo, and foobar
 *  Given prefix "foo", this will return foo.baz and foo.bar
 *  Given prefix "foo.bar", this will return foo.bar.baz
 *  Given prefix "foo.baz", this will return an empty list
 *
 *  All overrides are searched and returned, so that this can be used to
 *  conditionally declare parameters.
 *  The returned names may or may not be valid parameters, but instead are
 *  prefixes of valid parameters.
 *  The prefix itself will never be in the returned output.
 *
 *
 * @param[in] interfaces - The node interfaces used to get the parameter overrides
 * @param[in] prefix - the parameter prefix
 * @param[in] max_depth - how deep to return parameter override names, or 0 for
 *    unlimited depth.
*/
std::unordered_set<std::string>
list_parameter_override_prefixes(
  node_interfaces::NodeInterfaces<node_interfaces::Parameters> interfaces,
  std::string prefix);

/**
 * @brief Utility method for handling required ROS params
 *
 * @param[in] interfaces - The node interfaces used to load the parameter
 * @param[in] key - The ROS parameter key for the required parameter
 * @param[out] value - The ROS parameter value for the \p key
 * @throws std::runtime_error if the parameter does not exist
 */
inline
void getParamRequired(
  node_interfaces::NodeInterfaces<
    node_interfaces::Base,
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  const std::string & key,
  std::string & value
)
{
  std::string default_value = "";
  value = getParam(interfaces, key, default_value);

  if (value == default_value) {
    const std::string error =
      "Could not find required parameter " + key + " in namespace " +
      interfaces.get_node_base_interface()->get_namespace();

    RCLCPP_FATAL_STREAM(interfaces.get_node_logging_interface()->get_logger(), error);
    throw std::runtime_error(error);
  }
}

/**
 * @brief Helper function that loads positive integral or floating point values from the parameter
 *        server
 *
 * @param[in] interfaces - The node interfaces used to load the parameter
 * @param[in] parameter_name - The parameter name to load
 * @param[in, out] default_value - A default value to use if the provided parameter name does not
 *                 exist. As output it has the loaded (or default) value
 * @param[in] strict - Whether to check the loaded value is strictly positive or not, i.e. whether 0
 *                     is accepted or not
 */
template<typename T,
  typename = std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value>>
void getPositiveParam(
  node_interfaces::NodeInterfaces<
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  const std::string & parameter_name,
  T & default_value,
  const bool strict = true
)
{
  T value = getParam(interfaces, parameter_name, default_value);
  if (value < 0 || (strict && value == 0)) {
    RCLCPP_WARN_STREAM(
      interfaces.get_node_logging_interface()->get_logger(),
      "The requested " << parameter_name.c_str() << " is <" << (strict ? "=" : "")
                       << " 0. Using the default value (" << default_value << ") instead.");
  } else {
    default_value = value;
  }
}

/**
 * @brief Helper function that loads positive duration values from the parameter server
 *
 * @param[in] interfaces - The node interfaces used to load the parameter
 * @param[in] parameter_name - The parameter name to load
 * @param[in, out] default_value - A default value to use if the provided parameter name does not
 *                 exist. As output it has the loaded (or default) value
 * @param[in] strict - Whether to check the loaded value is strictly positive or not, i.e. whether 0
 *                     is accepted or not
 */
inline void getPositiveParam(
  node_interfaces::NodeInterfaces<
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  const std::string & parameter_name,
  rclcpp::Duration & default_value, const bool strict = true)
{
  double default_value_sec = default_value.seconds();
  getPositiveParam(interfaces, parameter_name, default_value_sec, strict);
  default_value = rclcpp::Duration::from_seconds(default_value_sec);
}

/**
 * @brief Helper function that loads a covariance matrix diagonal vector from the parameter server
 *        and checks the size and the values are invalid, i.e. they are positive.
 *
 * @tparam Scalar - A scalar type, defaults to double
 * @tparam Size - An int size that specifies the expected size of the covariance matrix (rows and
 *                columns)
 *
 * @param[in] interfaces - The node interfaces used to load the parameter
 * @param[in] parameter_name - The parameter name to load
 * @param[in] default_value - A default value to use for all the diagonal elements if the provided
 *                            parameter name does not exist
 * @return The loaded (or default) covariance matrix, generated from the diagonal vector
 */
template<int Size, typename Scalar = double>
fuse_core::Matrix<Scalar, Size, Size> getCovarianceDiagonalParam(
  node_interfaces::NodeInterfaces<
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  const std::string & parameter_name,
  Scalar default_value
)
{
  using Vector = typename Eigen::Matrix<Scalar, Size, 1>;

  std::vector<Scalar> diagonal(Size, default_value);
  diagonal = getParam(interfaces, parameter_name, diagonal);

  const auto diagonal_size = diagonal.size();
  if (diagonal_size != Size) {
    throw std::invalid_argument(
            "Invalid size of " + std::to_string(diagonal_size) + ", expected " +
            std::to_string(Size));
  }

  if (std::any_of(
      diagonal.begin(), diagonal.end(),
      [](const auto & value) {return value < Scalar(0);}))  // NOLINT(whitespace/braces)
  {
    throw std::invalid_argument(
            "Invalid negative diagonal values in " +
            fuse_core::to_string(Vector(diagonal.data())));
  }

  return Vector(diagonal.data()).asDiagonal();
}

/**
 * @brief Utility method to load a loss configuration
 *
 * @param[in] interfaces - The node interfaces used to load the parameter
 * @param[in] name - The ROS parameter name for the loss configuration parameter
 * @return Loss function or nullptr if the parameter does not exist
 */
inline fuse_core::Loss::SharedPtr loadLossConfig(
  node_interfaces::NodeInterfaces<
    node_interfaces::Base,
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  const std::string & name
)
{
  if (!interfaces.get_node_parameters_interface()->has_parameter(
      name + ".type"))
  {
    return {};
  }

  std::string loss_type;
  getParamRequired(interfaces, name + ".type", loss_type);

  auto loss = fuse_core::createUniqueLoss(loss_type);
  loss->initialize(interfaces, interfaces.get_node_base_interface()->get_fully_qualified_name());

  return loss;
}

}  // namespace fuse_core

#endif  // FUSE_CORE__PARAMETER_HPP_
