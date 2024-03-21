/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, ARTI - Autonomous Robot Technology GmbH
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
#ifndef FUSE_OPTIMIZERS_STEP_WISE_FIXED_LAG_SMOOTHER_PARAMS_H
#define FUSE_OPTIMIZERS_STEP_WISE_FIXED_LAG_SMOOTHER_PARAMS_H

#include <fuse_core/ceres_options.h>
#include <fuse_core/parameter.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <fuse_optimizers/fixed_lag_smoother_params.h>

#include <ceres/solver.h>

#include <algorithm>
#include <string>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief Defines the set of parameters required by the fuse_optimizers::StepWiseFixedLagSmoother class
 */
struct StepWiseFixedLagSmootherParams : FixedLagSmootherParams
{
public:
  /**
   * @brief The ordered list of steps for the optimization with each source inside
   *
   * The optimization is performed sequentially where each step adds
   * more constraints from different sources to the graph.
   */
  std::vector<std::vector<std::string>> optimization_steps;

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh)
  {
    FixedLagSmootherParams::loadFromROS(nh);

    // load optimization steps
    XmlRpc::XmlRpcValue optimization_steps_raw;
    fuse_core::getParamRequired<XmlRpc::XmlRpcValue>(nh, "optimization_steps", optimization_steps_raw);

    ROS_ASSERT(optimization_steps_raw.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int step_i = 0; step_i < optimization_steps_raw.size(); ++step_i)
    {
      ROS_ASSERT(optimization_steps_raw[step_i].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      ROS_ASSERT(optimization_steps_raw[step_i]["sources"].getType() == XmlRpc::XmlRpcValue::TypeArray);

      std::vector<std::string> optimization_step_sources;

      for (int source_i = 0; source_i < optimization_steps_raw[step_i]["sources"].size(); ++source_i)
      {
        ROS_ASSERT(optimization_steps_raw[step_i]["sources"][source_i].getType() == XmlRpc::XmlRpcValue::TypeString);

        optimization_step_sources.push_back(
            static_cast<std::string>(optimization_steps_raw[step_i]["sources"][source_i]));
      }

      optimization_steps.push_back(optimization_step_sources);
    }
  }
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_STEP_WISE_FIXED_LAG_SMOOTHER_PARAMS_H
