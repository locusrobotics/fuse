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
#include <fuse_optimizers/fixed_size_smoother.h>

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
FixedSizeSmoother::FixedSizeSmoother(fuse_core::Graph::UniquePtr graph, const ParameterType::SharedPtr& params,
                                     const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle)
  : fuse_optimizers::WindowedOptimizer(std::move(graph), params, node_handle, private_node_handle), params_(params)
{
}

FixedSizeSmoother::FixedSizeSmoother(fuse_core::Graph::UniquePtr graph, const ros::NodeHandle& node_handle,
                                     const ros::NodeHandle& private_node_handle)
  : FixedSizeSmoother::FixedSizeSmoother(
        std::move(graph), ParameterType::make_shared(fuse_core::loadFromROS<ParameterType>(private_node_handle)),
        node_handle, private_node_handle)
{
}

void FixedSizeSmoother::preprocessMarginalization(const fuse_core::Transaction& new_transaction)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timestamp_tracking_.addNewTransaction(new_transaction);
}

std::vector<fuse_core::UUID> FixedSizeSmoother::computeVariablesToMarginalize()
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto marginalize_variable_uuids = std::vector<fuse_core::UUID>();

  // if the total number of states is greater than our optimization window, then find the new state
  // given we remove the first n states to bring the number of states back to our desired window size
  if ((int)timestamp_tracking_.numStates() > params_->num_states)
  {
    size_t num_states_to_marginalize = timestamp_tracking_.numStates() - params_->num_states;
    ros::Time new_start_time = timestamp_tracking_[num_states_to_marginalize - 1];
    timestamp_tracking_.query(new_start_time, std::back_inserter(marginalize_variable_uuids));
  }

  return marginalize_variable_uuids;
}

void FixedSizeSmoother::postprocessMarginalization(const fuse_core::Transaction& marginal_transaction)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timestamp_tracking_.addMarginalTransaction(marginal_transaction);
}

bool FixedSizeSmoother::validateTransaction(const std::string& sensor_name, const fuse_core::Transaction& transaction)
{
  auto min_stamp = transaction.minStamp();
  std::lock_guard<std::mutex> lock(mutex_);
  ros::Time window_start = timestamp_tracking_[0];
  if (min_stamp < window_start)
  {
    ROS_DEBUG_STREAM("The current optimization window starts at "
                     << window_start << ". The queued transaction with timestamp " << transaction.stamp()
                     << " from sensor " << sensor_name << " has a minimum involved timestamp of " << min_stamp
                     << ", which is prior to the beginning "
                     << "of this window, ignoring this transaction.");
    return false;
  }
  return true;
}

void FixedSizeSmoother::onReset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  timestamp_tracking_.clear();
}

}  // namespace fuse_optimizers
