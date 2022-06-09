/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#ifndef FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H
#define FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H

#include <fuse_core/ceres_options.h>
#include <fuse_core/parameter.h>
#include <fuse_core/time.h>

//#include <ros/duration.h>
//#include <ros/node_handle.h>

#include <ceres/solver.h>

#include <algorithm>
#include <string>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief Defines the set of parameters required by the fuse_optimizers::FixedLagSmoother class
 */
struct FixedLagSmootherParams
{
public:
  /**
   * @brief The duration of the smoothing window in seconds
   */
  double lag_duration { 5.0 };

  /**
   * @brief The target duration for optimization cycles
   *
   * If an optimization takes longer than expected, an optimization cycle may be skipped. The optimization period
   * may be specified in either the "optimization_period" parameter in seconds, or in the "optimization_frequency"
   * parameter in Hz.
   */
  double optimization_period { 0.1 };

  /**
   * @brief The topic name of the advertised reset service
   */
  std::string reset_service { "~reset" };

  /**
   * @brief The maximum time to wait for motion models to be generated for a received transaction.
   *
   * Transactions are processed sequentially, so no new transactions will be added to the graph while waiting for
   * motion models to be generated. Once the timeout expires, that transaction will be deleted from the queue.
   */
  double transaction_timeout { 0.1 };

  /**
   * @brief Ceres Solver::Options object that controls various aspects of the optimizer.
   */
  ceres::Solver::Options solver_options;

  /**
   * @brief Method for loading parameter values from ROS.
   *

   * @param[in] node_params - The node parameter interface handle used to load the parameter
   * @param[in] logger - The ros logger used to report errors
   */
  void loadFromROS(  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
  rclcpp::Logger logger)
  {
    // Read settings from the parameter server
    fuse_core::getPositiveParam(node_params, logger, "lag_duration", lag_duration);

    if (nh.hasParam("optimization_frequency"))
    {
      double optimization_frequency{ 1.0 / optimization_period };
      fuse_core::getPositiveParam(node_params, logger, "optimization_frequency", optimization_frequency);
      optimization_period = 1.0 / optimization_frequency;
    }
    else
    {
      fuse_core::getPositiveParam(node_params, logger, "optimization_period", optimization_period);
    }

    nh.getParam("reset_service", reset_service);

    fuse_core::getPositiveParam(node_params, logger, "transaction_timeout", transaction_timeout);

    fuse_core::loadSolverOptionsFromROS(node_params, logger, solver_options);
  }
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H
