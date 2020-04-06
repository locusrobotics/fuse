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
#ifndef FUSE_MODELS_PARAMETERS_ACCELERATION_2D_PARAMS_H
#define FUSE_MODELS_PARAMETERS_ACCELERATION_2D_PARAMS_H

#include <fuse_core/loss.h>
#include <fuse_core/parameter.h>
#include <fuse_variables/acceleration_linear_2d_stamped.h>
#include <fuse_models/parameters/parameter_base.h>

#include <ros/node_handle.h>

#include <string>
#include <vector>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Acceleration2D class
 */
struct Acceleration2DParams : public ParameterBase
{
  public:
    /**
     * @brief Method for loading parameter values from ROS.
     *
     * @param[in] nh - The ROS node handle with which to load parameters
     */
    void loadFromROS(const ros::NodeHandle& nh) final
    {
      indices = loadSensorConfig<fuse_variables::AccelerationLinear2DStamped>(nh, "dimensions");

      nh.getParam("disable_checks", disable_checks);
      nh.getParam("queue_size", queue_size);

      double throttle_period_double = throttle_period.toSec();
      fuse_core::getPositiveParam(nh, "throttle_period", throttle_period_double);
      throttle_period.fromSec(throttle_period_double);

      fuse_core::getParamRequired(nh, "topic", topic);
      fuse_core::getParamRequired(nh, "target_frame", target_frame);

      loss = fuse_core::loadLossConfig(nh, "loss");
    }

    bool disable_checks { false };
    int queue_size { 10 };
    ros::Duration throttle_period { 0.0 };  //!< The throttle period duration in seconds
    std::string topic {};
    std::string target_frame {};
    std::vector<size_t> indices;
    fuse_core::Loss::SharedPtr loss;
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS_PARAMETERS_ACCELERATION_2D_PARAMS_H
