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
#ifndef FUSE_MODELS_PARAMETERS_PARAMETER_BASE_H
#define FUSE_MODELS_PARAMETERS_PARAMETER_BASE_H

#include <fuse_models/common/sensor_config.h>

#include <ros/node_handle.h>

#include <stdexcept>
#include <string>
#include <vector>


namespace fuse_models
{

namespace parameters
{

struct ParameterBase
{
  ParameterBase() = default;
  virtual ~ParameterBase() = default;

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  virtual void loadFromROS(const ros::NodeHandle& nh) = 0;
};

/**
 * @brief Utility method to load a sensor configuration, i.e. the dimension indices
 *
 * @tparam T - The variable type the dimension indices belong to
 *
 * @param[in] nh - The ROS node handle with which to load parameters
 * @param[in] name - The ROS parameter name for the sensor configuration parameter
 * @return A vector with the dimension indices, that would be empty if the parameter does not exist
 */
template <typename T>
inline std::vector<size_t> loadSensorConfig(const ros::NodeHandle& nh, const std::string& name)
{
  std::vector<std::string> dimensions;
  if (nh.getParam(name, dimensions))
  {
    return common::getDimensionIndices<T>(dimensions);
  }

  return {};
}

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS_PARAMETERS_PARAMETER_BASE_H
