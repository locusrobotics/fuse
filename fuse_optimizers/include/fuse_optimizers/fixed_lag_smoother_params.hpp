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

#ifndef FUSE_OPTIMIZERS__FIXED_LAG_SMOOTHER_PARAMS_HPP_
#define FUSE_OPTIMIZERS__FIXED_LAG_SMOOTHER_PARAMS_HPP_

#include <ceres/solver.h>

#include <algorithm>
#include <string>
#include <vector>

#include <fuse_core/ceres_options.hpp>
#include <fuse_core/parameter.hpp>
#include <rclcpp/rclcpp.hpp>


namespace fuse_optimizers
{

/**
 * @brief Defines the set of parameters required by the fuse_optimizers::FixedLagSmoother class
 */
struct FixedLagSmootherParams
{
public:
  /**
   * @brief Map the ceres::optimizer log levels to diagnostic levels
   */
  std::vector<std::string> diagnostic_warning_status {"NO_CONVERGENCE"};

  /**
   * @brief Map the ceres::optimizer log levels to diagnostic levels
   */
  std::vector<std::string> diagnostic_error_status {"FAILURE", "USER_FAILURE"};

  /**
   * @brief The duration of the smoothing window in seconds
   */
  rclcpp::Duration lag_duration {5, 0};

  /**
   * @brief The target duration for optimization cycles
   *
   * If an optimization takes longer than expected, an optimization cycle may be skipped. The
   * optimization period may be specified in either the "optimization_period" parameter in seconds,
   * or in the "optimization_frequency" parameter in Hz. "optimization_frequency" will be
   * prioritized.
   */
  rclcpp::Duration optimization_period {0, static_cast<uint32_t>(RCUTILS_S_TO_NS(0.1))};

  /**
   * @brief The topic name of the advertised reset service
   */
  std::string reset_service {"~/reset"};

  /**
   * @brief The maximum time to wait for motion models to be generated for a received transaction.
   *
   * Transactions are processed sequentially, so no new transactions will be added to the graph
   * while waiting for motion models to be generated. Once the timeout expires, that transaction
   * will be deleted from the queue.
   */
  rclcpp::Duration transaction_timeout {0, static_cast<uint32_t>(RCUTILS_S_TO_NS(0.1))};

  /**
   * @brief Ceres Solver::Options object that controls various aspects of the optimizer.
   */
  ceres::Solver::Options solver_options;

  /**
   * @brief Method for loading parameter values from ROS.
   *

   * @param[in] node - The node used to load the parameter
   */
  void loadFromROS(
    fuse_core::node_interfaces::NodeInterfaces<
      fuse_core::node_interfaces::Base,
      fuse_core::node_interfaces::Logging,
      fuse_core::node_interfaces::Parameters
    > interfaces)
  {
    // Read settings from the parameter server
    fuse_core::getParam(interfaces, "diagnostic_error_status", diagnostic_error_status);
    fuse_core::getParam(interfaces, "diagnostic_warning_status", diagnostic_warning_status);
    fuse_core::getPositiveParam(interfaces, "lag_duration", lag_duration);

    double optimization_frequency{-1.0};
    optimization_frequency = fuse_core::getParam(
      interfaces, "optimization_frequency",
      optimization_frequency);
    fuse_core::getPositiveParam(interfaces, "optimization_period", optimization_period);

    if (optimization_frequency != -1.0) {
      if (optimization_frequency < 0) {
        RCLCPP_WARN_STREAM(
          interfaces.get_node_logging_interface()->get_logger(),
          "The requested optimization_frequency parameter is < 0. Using the optimization_period"
          "parameter instead!");
      }
      optimization_period =
        rclcpp::Duration::from_seconds(1.0 / optimization_frequency);
    }

    fuse_core::getParam(interfaces, "reset_service", reset_service);

    fuse_core::getPositiveParam(interfaces, "transaction_timeout", transaction_timeout);

    fuse_core::loadSolverOptionsFromROS(interfaces, solver_options, "solver_options");
  }
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS__FIXED_LAG_SMOOTHER_PARAMS_HPP_
