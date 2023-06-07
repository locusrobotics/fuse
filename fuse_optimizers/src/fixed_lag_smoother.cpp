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
#include <fuse_optimizers/fixed_lag_smoother.h>

#include <fuse_core/graph.h>
#include <fuse_core/parameter.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_optimizers/windowed_optimizer.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <iterator>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace fuse_optimizers
{
  FixedLagSmoother::FixedLagSmoother(fuse_core::Graph::UniquePtr graph, const ParameterType::SharedPtr &params,
                                     const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle)
      : fuse_optimizers::WindowedOptimizer(std::move(graph), params, node_handle, private_node_handle), params_(params)
  {
  }

  FixedLagSmoother::FixedLagSmoother(fuse_core::Graph::UniquePtr graph, const ros::NodeHandle &node_handle,
                                     const ros::NodeHandle &private_node_handle)
      : FixedLagSmoother::FixedLagSmoother(
            std::move(graph), ParameterType::make_shared(fuse_core::loadFromROS<ParameterType>(private_node_handle)),
            node_handle, private_node_handle)
  {
  }

  void FixedLagSmoother::preprocessMarginalization(const fuse_core::Transaction &new_transaction)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timestamp_tracking_.addNewTransaction(new_transaction);
  }

  std::vector<fuse_core::UUID> FixedLagSmoother::computeVariablesToMarginalize()
  {
    // Find the most recent variable timestamp, then carefully subtract the lag duration.
    // ROS Time objects do not handle negative values.
    auto start_time = getStartTime();

    std::lock_guard<std::mutex> lock(mutex_);
    auto now = timestamp_tracking_[timestamp_tracking_.numStates() - 1];
    lag_expiration_ = (start_time + params_->lag_duration < now) ? now - params_->lag_duration : start_time;
    auto marginalize_variable_uuids = std::vector<fuse_core::UUID>();
    timestamp_tracking_.query(lag_expiration_, std::back_inserter(marginalize_variable_uuids));
    return marginalize_variable_uuids;
  }

  void FixedLagSmoother::postprocessMarginalization(const fuse_core::Transaction &marginal_transaction)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timestamp_tracking_.addMarginalTransaction(marginal_transaction);
  }

  bool FixedLagSmoother::validateTransaction(const std::string &sensor_name, const fuse_core::Transaction &transaction)
  {
    auto min_stamp = transaction.minStamp();
    std::lock_guard<std::mutex> lock(mutex_);
    if (min_stamp < lag_expiration_)
    {
      ROS_DEBUG_STREAM(
          "The current lag expiration time is "
          << lag_expiration_ << ". The queued transaction with timestamp " << transaction.stamp() << " from sensor "
          << sensor_name << " has a minimum involved timestamp of " << min_stamp << ", which is "
          << (lag_expiration_ - min_stamp) << " seconds too old. Ignoring this transaction.");
      return false;
    }
    return true;
  }

  void FixedLagSmoother::onReset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timestamp_tracking_.clear();
    lag_expiration_ = ros::Time(0, 0);
  }

} // namespace fuse_optimizers
