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
#ifndef FUSE_MODELS__COMMON__SENSOR_CONFIG_HPP_
#define FUSE_MODELS__COMMON__SENSOR_CONFIG_HPP_

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <fuse_models/common/variable_traits.hpp>

#include <boost/algorithm/string/case_conv.hpp>
#include <fuse_variables/acceleration_linear_2d_stamped.hpp>
#include <fuse_variables/acceleration_linear_3d_stamped.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <fuse_variables/velocity_angular_2d_stamped.hpp>
#include <fuse_variables/velocity_angular_3d_stamped.hpp>
#include <fuse_variables/velocity_linear_2d_stamped.hpp>
#include <fuse_variables/velocity_linear_3d_stamped.hpp>
#include <rclcpp/logging.hpp>


namespace fuse_models
{

namespace common
{

/**
 * @brief Utility method for printing and throwing an error for invalid dimension specification
 * @param[in] dimension - The erroneous dimension name
 * @throws runtime_error
 */
inline void throwDimensionError(const std::string & dimension)
{
  std::string error = "Dimension " + dimension + " is not valid for this type.";
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("fuse"), error);
  throw std::runtime_error(error);
}

/**
 * @brief Method that converts from 2D linear axis dimension names to index values
 *
 * This method is enabled only for variables that contain _only_ 2D linear quantities
 *
 * @param[in] dimension - The dimension name to convert
 * @return the index of the enumerated dimension for that type
 * @throws runtime_error if the dimension name is invalid
 */
template<typename T>
std::enable_if_t<is_linear_2d<T>::value, size_t> toIndex(const std::string & dimension)
{
  auto lower_dim = boost::algorithm::to_lower_copy(dimension);
  if (lower_dim == "x") {return static_cast<size_t>(T::X);}
  if (lower_dim == "y") {return static_cast<size_t>(T::Y);}

  throwDimensionError(dimension);

  return 0u;
}

/**
 * @brief Method that converts from 3D linear axis dimension names to index values
 *
 * This method is enabled only for variables that contain _only_ 3D linear quantities
 *
 * @param[in] dimension - The dimension name to convert
 * @return the index of the enumerated dimension for that type
 * @throws runtime_error if the dimension name is invalid
 */
template<typename T>
std::enable_if_t<is_linear_3d<T>::value, size_t> toIndex(const std::string & dimension)
{
  auto lower_dim = boost::algorithm::to_lower_copy(dimension);
  if (lower_dim == "x") {return static_cast<size_t>(T::X);}
  if (lower_dim == "y") {return static_cast<size_t>(T::Y);}
  if (lower_dim == "z") {return static_cast<size_t>(T::Z);}

  throwDimensionError(dimension);

  return 0u;
}

/**
 * @brief Method that converts from 2D angular axis dimension names to index values
 *
 * This method is enabled only for variables that contain _only_ 2D angular quantities
 *
 * @param[in] dimension - The dimension name to convert
 * @return the index of the enumerated dimension for that type
 * @throws runtime_error if the dimension name is invalid
 */
template<typename T>
std::enable_if_t<is_angular_2d<T>::value, size_t> toIndex(const std::string & dimension)
{
  auto lower_dim = boost::algorithm::to_lower_copy(dimension);
  if (lower_dim == "yaw" || lower_dim == "z") {
    return static_cast<size_t>(fuse_variables::Orientation2DStamped::YAW);
  }

  throwDimensionError(dimension);

  return 0u;
}

/**
 * @brief Method that converts from 3D angular axis dimension names to index values
 *
 * This method is enabled only for variables that contain _only_ 3D angular quantities
 *
 * @param[in] dimension - The dimension name to convert
 * @return the index of the enumerated dimension for that type
 * @throws runtime_error if the dimension name is invalid
 */
template<typename T>
std::enable_if_t<is_angular_3d<T>::value && !is_orientation<T>::value, size_t> toIndex(const std::string & dimension)
{
  auto lower_dim = boost::algorithm::to_lower_copy(dimension);
  if (lower_dim == "roll" || lower_dim == "x") {
    return static_cast<size_t>(T::ROLL);
  }
  if (lower_dim == "pitch" || lower_dim == "y") {
    return static_cast<size_t>(T::PITCH);
  }
  if (lower_dim == "yaw" || lower_dim == "z") {
    return static_cast<size_t>(T::YAW);
  }

  throwDimensionError(dimension);

  return 0u;
}

template<typename T>
std::enable_if_t<is_angular_3d<T>::value && is_orientation<T>::value, size_t> toIndex(const std::string & dimension)
{
  // Trick to get roll, pitch, yaw indexes as 0, 1, 2
  auto lower_dim = boost::algorithm::to_lower_copy(dimension);
  if (lower_dim == "roll" || lower_dim == "x") {
    return static_cast<size_t>(fuse_variables::Orientation3DStamped::Euler::ROLL) - 4UL;
  }
  if (lower_dim == "pitch" || lower_dim == "y") {
    return static_cast<size_t>(fuse_variables::Orientation3DStamped::Euler::PITCH) - 4UL;
  }
  if (lower_dim == "yaw" || lower_dim == "z") {
    return static_cast<size_t>(fuse_variables::Orientation3DStamped::Euler::YAW) - 4UL;
  }

  throwDimensionError(dimension);

  return 0u;
}

/**
 * @brief Utility method to convert a vector of dimension names to a vector of dimension indices
 *
 * Note that the dimensions are sorted, and so the order in which the user specifies them will have
 * no bearing when the measurement vectors and covariances are actually built elsewhere.
 *
 * @param[in] dimension_names - The dimension names to convert
 * @return a vector of indices that are consistent with the enumerations for that variable type
 * @throws runtime_error if any dimension name is invalid
 */
template<typename T>
std::vector<size_t> getDimensionIndices(const std::vector<std::string> & dimension_names)
{
  std::vector<size_t> indices;
  indices.reserve(dimension_names.size());

  std::transform(
    dimension_names.begin(),
    dimension_names.end(),
    std::back_inserter(indices),
    toIndex<T>);

  // Remove duplicates
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

  return indices;
}

}  // namespace common

}  // namespace fuse_models

#endif  // FUSE_MODELS__COMMON__SENSOR_CONFIG_HPP_
