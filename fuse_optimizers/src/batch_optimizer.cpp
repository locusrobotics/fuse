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
#include <fuse_core/transaction.h>
#include <fuse_optimizers/batch_optimizer.h>
#include <fuse_optimizers/optimizer.h>
#include <ros/ros.h>

#include <algorithm>
#include <mutex>
#include <string>
#include <utility>
#include <thread>


namespace fuse_optimizers
{

BatchOptimizer::BatchOptimizer(
  fuse_core::Graph::UniquePtr graph,
  const ros::NodeHandle& node_handle,
  const ros::NodeHandle& private_node_handle) :
    fuse_optimizers::Optimizer(std::move(graph), node_handle, private_node_handle),
    combined_transaction_(fuse_core::Transaction::make_shared()),
    optimization_request_(false),
    start_time_(ros::TIME_MAX),
    started_(false)
{
  params_.loadFromROS(private_node_handle);

  // Configure a timer to trigger optimizations
  optimize_timer_ = node_handle_.createTimer(
    ros::Duration(params_.optimization_period),
    &BatchOptimizer::optimizerTimerCallback,
    this);

  // Start the optimization thread
  optimization_thread_ = std::thread(&BatchOptimizer::optimizationLoop, this);
}

BatchOptimizer::~BatchOptimizer()
{
  // Wake up any sleeping threads
  optimization_requested_.notify_all();
  // Wait for the threads to shutdown
  if (optimization_thread_.joinable())
  {
    optimization_thread_.join();
  }
}

void BatchOptimizer::applyMotionModelsToQueue()
{
  // We need get the pending transactions from the queue
  std::lock_guard<std::mutex> pending_transactions_lock(pending_transactions_mutex_);
  // Use the most recent transaction time as the current time
  ros::Time current_time(0, 0);
  if (!pending_transactions_.empty())
  {
    current_time = pending_transactions_.rbegin()->first;
  }
  // Attempt to process each pending transaction
  while (!pending_transactions_.empty())
  {
    auto& element = pending_transactions_.begin()->second;
    // Apply the motion models to the transaction
    if (!applyMotionModels(element.sensor_name, *element.transaction))
    {
      if (element.transaction->stamp() + params_.transaction_timeout < current_time)
      {
        // Warn that this transaction has expired, then skip it.
        ROS_ERROR_STREAM("The queued transaction with timestamp " << element.transaction->stamp()
                          << " could not be processed after " << (current_time - element.transaction->stamp())
                          << " seconds, which is greater than the 'transaction_timeout' value of "
                          << params_.transaction_timeout << ". Ignoring this transaction.");
        pending_transactions_.erase(pending_transactions_.begin());
        continue;
      }
      else
      {
        // Stop processing future transactions. Try again next time.
        break;
      }
    }
    // Merge the sensor+motion model transactions into a combined transaction that will be applied directly to the graph
    {
      std::lock_guard<std::mutex> combined_transaction_lock(combined_transaction_mutex_);
      combined_transaction_->merge(*element.transaction, true);
    }
    // We are done with this transaction. Delete it from the queue.
    pending_transactions_.erase(pending_transactions_.begin());
  }
}

void BatchOptimizer::optimizationLoop()
{
  // Optimize constraints until told to exit
  while (ros::ok())
  {
    // Wait for the next signal to start the next optimization cycle
    {
      std::unique_lock<std::mutex> lock(optimization_requested_mutex_);
      optimization_requested_.wait(lock, [this]{ return optimization_request_ || !ros::ok(); });  // NOLINT
    }
    // If a shutdown is requested, exit now.
    if (!ros::ok())
    {
      break;
    }
    // Copy the combined transaction so it can be shared with all the plugins
    fuse_core::Transaction::ConstSharedPtr const_transaction;
    {
      std::lock_guard<std::mutex> lock(combined_transaction_mutex_);
      const_transaction = std::move(combined_transaction_);
      combined_transaction_ = fuse_core::Transaction::make_shared();
    }
    // Update the graph
    graph_->update(*const_transaction);
    // Optimize the entire graph
    graph_->optimize(params_.solver_options);
    // Make a copy of the graph to share
    fuse_core::Graph::ConstSharedPtr const_graph = graph_->clone();
    // Optimization is complete. Notify all the things about the graph changes.
    notify(const_transaction, const_graph);
    // Clear the request flag now that this optimization cycle is complete
    optimization_request_ = false;
  }
}

void BatchOptimizer::optimizerTimerCallback(const ros::TimerEvent& /*event*/)
{
  // If an "ignition" transaction hasn't been received, then we can't do anything yet.
  if (!started_)
  {
    return;
  }
  // Attempt to generate motion models for any queued transactions
  applyMotionModelsToQueue();
  // Check if there is any pending information to be applied to the graph.
  {
    std::lock_guard<std::mutex> lock(combined_transaction_mutex_);
    optimization_request_ = !combined_transaction_->empty();
  }
  // If there is some pending work, trigger the next optimization cycle.
  // If the optimizer has not completed the previous optimization cycle, then it
  // will not be waiting on the condition variable signal, so nothing will happen.
  if (optimization_request_)
  {
    optimization_requested_.notify_one();
  }
}

void BatchOptimizer::transactionCallback(
  const std::string& sensor_name,
  fuse_core::Transaction::SharedPtr transaction)
{
  // Add the new transaction to the pending set
  // Either we haven't "started" yet and we want to keep a short history of transactions around
  // Or we have "started" already, and the new transaction is after the starting time.
  ros::Time transaction_time = transaction->stamp();
  ros::Time last_pending_time(0, 0);
  if (!started_ || transaction_time >= start_time_)
  {
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    pending_transactions_.emplace(transaction_time, TransactionQueueElement(sensor_name, std::move(transaction)));
    last_pending_time = pending_transactions_.rbegin()->first;
  }
  // If we haven't "started" yet...
  if (!started_)
  {
    // Check if this transaction "starts" the system
    if (sensor_models_.at(sensor_name).ignition)
    {
      started_ = true;
      start_time_ = transaction_time;
    }
    // Purge old transactions from the pending queue
    ros::Time purge_time(0, 0);
    if (started_)
    {
      purge_time = start_time_;
    }
    else if (ros::Time(0, 0) + params_.transaction_timeout < last_pending_time)  // prevent a bad subtraction
    {
      purge_time = last_pending_time - params_.transaction_timeout;
    }
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    auto purge_iter = pending_transactions_.lower_bound(purge_time);
    pending_transactions_.erase(pending_transactions_.begin(), purge_iter);
  }
  // If we have "started", attempt to process any pending transactions
  if (started_)
  {
    applyMotionModelsToQueue();
  }
}

void BatchOptimizer::setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
{
  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "BatchOptimizer");

  Optimizer::setDiagnostics(status);

  status.add("Started", started_);
  {
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    status.add("Pending Transactions", pending_transactions_.size());
  }
}

}  // namespace fuse_optimizers
