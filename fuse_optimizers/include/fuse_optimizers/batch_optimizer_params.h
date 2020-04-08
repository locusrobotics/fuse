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
#ifndef FUSE_OPTIMIZERS_BATCH_OPTIMIZER_PARAMS_H
#define FUSE_OPTIMIZERS_BATCH_OPTIMIZER_PARAMS_H

#include <fuse_core/ceres_options.h>
#include <fuse_core/parameter.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <ceres/solver.h>

#include <algorithm>
#include <string>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief Defines the set of parameters required by the fuse_optimizers::FixedLagSmoother class
 */
struct BatchOptimizerParams
{
public:
  /**
   * @brief The target duration for optimization cycles
   *
   * If an optimization takes longer than expected, an optimization cycle may be skipped. The optimization period
   * may be specified in either the "optimization_period" parameter in seconds, or in the "optimization_frequency"
   * parameter in Hz.
   */
  ros::Duration optimization_period { 0.1 };

  /**
   * @brief The maximum time to wait for motion models to be generated for a received transaction.
   *
   * Transactions are processed sequentially, so no new transactions will be added to the graph while waiting for
   * motion models to be generated. Once the timeout expires, that transaction will be deleted from the queue.
   */
  ros::Duration transaction_timeout { 0.1 };

  /**
   * @brief Ceres Solver::Options object that controls various aspects of the optimizer.
   */
  ceres::Solver::Options solver_options;

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh)
  {
    // Read settings from the parameter server
    if (nh.hasParam("optimization_frequency"))
    {
      double optimization_frequency{ 1.0 / optimization_period.toSec() };
      fuse_core::getPositiveParam(nh, "optimization_frequency", optimization_frequency);
      optimization_period.fromSec(1.0 / optimization_frequency);
    }
    else
    {
      double optimization_period_sec{ optimization_period.toSec() };
      fuse_core::getPositiveParam(nh, "optimization_period", optimization_period_sec);
      optimization_period.fromSec(optimization_period_sec);
    }

    double transaction_timeout_sec{ transaction_timeout.toSec() };
    fuse_core::getPositiveParam(nh, "transaction_timeout", transaction_timeout_sec);
    transaction_timeout.fromSec(transaction_timeout_sec);

    fuse_core::loadSolverOptionsFromROS(ros::NodeHandle(nh, "solver_options"), solver_options);
  }
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_BATCH_OPTIMIZER_PARAMS_H
