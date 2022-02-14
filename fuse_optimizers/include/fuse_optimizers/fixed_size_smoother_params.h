/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Locus Robotics
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
#ifndef FUSE_OPTIMIZERS_FIXED_SIZE_SMOOTHER_PARAMS_H
#define FUSE_OPTIMIZERS_FIXED_SIZE_SMOOTHER_PARAMS_H

#include <fuse_core/parameter.h>
#include <fuse_optimizers/windowed_optimizer_params.h>
#include <ros/node_handle.h>

namespace fuse_optimizers
{
/**
 * @brief Defines the set of parameters required by the fuse_optimizers::FixedSizeSmoother class
 */
struct FixedSizeSmootherParams : public WindowedOptimizerParams
{
public:
  SMART_PTR_DEFINITIONS(FixedSizeSmootherParams);

  /**
   * @brief Thenumber of unique stamps in the window
   */
  int num_states{ 10 };

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh)
  {
    WindowedOptimizerParams::loadFromROS(nh);

    // Read settings from the parameter server
    fuse_core::getPositiveParam(nh, "num_states", num_states);
  }
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_FIXED_SIZE_SMOOTHER_PARAMS_H
