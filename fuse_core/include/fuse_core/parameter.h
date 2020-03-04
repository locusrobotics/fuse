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

#include <fuse_core/loss_loader.h>

#include <ros/node_handle.h>

#include <stdexcept>
#include <string>


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
void getParamRequired(const ros::NodeHandle& nh, const std::string& key, T& value)
{
  if (!nh.getParam(key, value))
  {
    const std::string error = "Could not find required parameter " + key + " in namespace " + nh.getNamespace();
    ROS_FATAL_STREAM(error);
    throw std::runtime_error(error);
  }
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
