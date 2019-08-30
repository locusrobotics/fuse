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

#include <fuse_constraints/marginalize_variables.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_optimizers/optimizer.h>
#include <ros/ros.h>

#include <algorithm>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


namespace
{
/**
 * @brief Delete an element from the vector using a reverse iterator
 *
 * @param[in] container The contain to delete from
 * @param[in] position  A reverse iterator that access the element to be erased
 * @return A reverse iterator pointing to the element after the erased element
 */
template <typename T>
typename std::vector<T>::reverse_iterator erase(
  std::vector<T>& container,
  typename std::vector<T>::reverse_iterator position)
{
  // Reverse iterators are weird
  // https://stackoverflow.com/questions/1830158/how-to-call-erase-with-a-reverse-iterator
  // Basically a reverse iterator access the element one place before the element it points at.
  // E.g. The reverse iterator rbegin points at end, but accesses end-1.
  // When you delete something, you need to increment the reverse iterator first, then convert it to a standard
  // iterator for the erase operation.
  std::advance(position, 1);
  container.erase(position.base());
  return position;
}
}  // namespace

namespace fuse_optimizers
{

FixedLagSmoother::FixedLagSmoother(
  fuse_core::Graph::UniquePtr graph,
  const ros::NodeHandle& node_handle,
  const ros::NodeHandle& private_node_handle) :
    fuse_optimizers::Optimizer(std::move(graph), node_handle, private_node_handle),
    optimization_request_(false),
    optimization_running_(true),
    start_time_(ros::TIME_MAX),
    started_(false)
{
  params_.loadFromROS(private_node_handle);

  // Warn about possible configuration errors
  // TODO(swilliams) Move this warning to the Parameter loadFromROS() method once all parameters are loaded there.
  for (const auto& sensor_model_name : params_.ignition_sensors)
  {
    if (sensor_models_.find(sensor_model_name) == sensor_models_.end())
    {
      ROS_WARN_STREAM("Sensor '" << sensor_model_name << "' is configured as an ignition sensor, but no sensor "
                      "model with that name currently exists. This is likely a configuration error.");
    }
  }

  // Test for auto-start
  autostart();

  // Start the optimization thread
  optimization_thread_ = std::thread(&FixedLagSmoother::optimizationLoop, this);

  // Configure a timer to trigger optimizations
  optimize_timer_ = node_handle_.createTimer(
    params_.optimization_period,
    &FixedLagSmoother::optimizerTimerCallback,
    this);

  // Advertise a service that resets the optimizer to its initial state
  reset_service_server_ = node_handle_.advertiseService(
    ros::names::resolve(params_.reset_service),
    &FixedLagSmoother::resetServiceCallback,
    this);
}

FixedLagSmoother::~FixedLagSmoother()
{
  // Wake up any sleeping threads
  optimization_running_ = false;
  optimization_requested_.notify_all();
  // Wait for the threads to shutdown
  if (optimization_thread_.joinable())
  {
    optimization_thread_.join();
  }
}

void FixedLagSmoother::autostart()
{
  if (params_.ignition_sensors.empty())
  {
    // No ignition sensors were provided. Auto-start.
    started_ = true;
    start_time_ = ros::Time(0, 0);
    ROS_INFO_STREAM("No ignition sensors were specified. Optimization will begin immediately.");
  }
}

void FixedLagSmoother::preprocessMarginalization(const fuse_core::Transaction& new_transaction)
{
  timestamp_tracking_.addNewTransaction(new_transaction);
}

std::vector<fuse_core::UUID> FixedLagSmoother::computeVariablesToMarginalize()
{
  auto current_stamp = timestamp_tracking_.currentStamp();
  auto lag_stamp = ros::Time(0, 0);
  if (current_stamp > ros::Time(0, 0) + params_.lag_duration)
  {
    lag_stamp = current_stamp - params_.lag_duration;
  }
  auto old_variables = std::vector<fuse_core::UUID>();
  timestamp_tracking_.query(lag_stamp, std::back_inserter(old_variables));
  return old_variables;
}

void FixedLagSmoother::postprocessMarginalization(const fuse_core::Transaction& marginal_transaction)
{
  timestamp_tracking_.addMarginalTransaction(marginal_transaction);
}

void FixedLagSmoother::optimizationLoop()
{
  auto exit_wait_condition = [this]()
  {
    return this->optimization_request_ || !this->optimization_running_ || !ros::ok();
  };
  // Optimize constraints until told to exit
  while (ros::ok() && optimization_running_)
  {
    // Wait for the next signal to start the next optimization cycle
    auto optimization_deadline = ros::Time(0, 0);
    {
      std::unique_lock<std::mutex> lock(optimization_requested_mutex_);
      optimization_requested_.wait(lock, exit_wait_condition);
      optimization_deadline = optimization_deadline_;
    }
    // If a shutdown is requested, exit now.
    if (!optimization_running_ || !ros::ok())
    {
      break;
    }
    // Optimize
    {
      std::lock_guard<std::mutex> lock(optimization_mutex_);
      // Apply motion models
      auto new_transaction = fuse_core::Transaction::make_shared();
      // DANGER: processQueue obtains a lock from the pending_transactions_mutex_
      //         We do this to ensure state of the graph does not change between unlocking the pending_transactions
      //         queue and obtaining the lock for the graph. But we have now obtained two different locks. If we are
      //         not extremely careful, we could get a deadlock.
      processQueue(*new_transaction);
      // Prepare for selecting the marginal variables
      preprocessMarginalization(*new_transaction);
      // Combine the new transactions with any marginal transaction from the end of the last cycle
      new_transaction->merge(marginal_transaction_);
      // Update the graph
      graph_->update(*new_transaction);
      // Optimize the entire graph
      graph_->optimize(params_.solver_options);
      // Optimization is complete. Notify all the things about the graph changes.
      notify(std::move(new_transaction), graph_->clone());
      // Compute a transaction that marginalizes out those variables.
      marginal_transaction_ = fuse_constraints::marginalizeVariables(
        ros::this_node::getName(),
        computeVariablesToMarginalize(),
        *graph_);
      // Perform any post-marginal cleanup
      postprocessMarginalization(marginal_transaction_);
      // Note: The marginal transaction will not be applied until the next optimization iteration
      // Log a warning if the optimization took too long
      auto optimization_complete = ros::Time::now();
      if (optimization_complete > optimization_deadline)
      {
        ROS_WARN_STREAM("Optimization exceeded the configured duration by " <<
                        (optimization_complete - optimization_deadline) << "s");
      }
    }
    // Clear the request flag now that this optimization cycle is complete
    optimization_request_ = false;
  }
}

void FixedLagSmoother::optimizerTimerCallback(const ros::TimerEvent& event)
{
  // If an "ignition" transaction hasn't been received, then we can't do anything yet.
  if (!started_)
  {
    return;
  }
  // If there is some pending work, trigger the next optimization cycle.
  // If the optimizer has not completed the previous optimization cycle, then it
  // will not be waiting on the condition variable signal, so nothing will happen.
  {
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    optimization_request_ = !pending_transactions_.empty();
  }
  if (optimization_request_)
  {
    {
      std::lock_guard<std::mutex> lock(optimization_requested_mutex_);
      optimization_deadline_ = event.current_expected + params_.optimization_period;
    }
    optimization_requested_.notify_one();
  }
}

void FixedLagSmoother::processQueue(fuse_core::Transaction& transaction)
{
  // We need to get the pending transactions from the queue
  std::lock_guard<std::mutex> pending_transactions_lock(pending_transactions_mutex_);
  // Use the most recent transaction time as the current time
  auto current_time = ros::Time(0, 0);
  if (!pending_transactions_.empty())
  {
    current_time = pending_transactions_.front().stamp();
  }
  // Attempt to process each pending transaction
  auto sensor_blacklist = std::vector<std::string>();
  auto transaction_riter = pending_transactions_.rbegin();
  while (transaction_riter != pending_transactions_.rend())
  {
    auto& element = *transaction_riter;
    if (std::find(sensor_blacklist.begin(), sensor_blacklist.end(), element.sensor_name) != sensor_blacklist.end())
    {
      // We should not process transactions from this sensor
      ++transaction_riter;
    }
    else if (applyMotionModels(element.sensor_name, *element.transaction))
    {
      // Processing was successful. Add the results to the final transaction, delete this one, and move to the next.
      transaction.merge(*element.transaction, true);
      transaction_riter = erase(pending_transactions_, transaction_riter);
    }
    else
    {
      // The motion model processing failed.
      // Check the transaction timeout to determine if it should be removed or skipped.
      if (element.transaction->stamp() + params_.transaction_timeout < current_time)
      {
        // Warn that this transaction has expired, then skip it.
        ROS_ERROR_STREAM("The queued transaction with timestamp " << element.transaction->stamp() <<
                          " from sensor " << element.sensor_name << " could not be processed after " <<
                          (current_time - element.transaction->stamp()) << " seconds, which is greater " <<
                          "than the 'transaction_timeout' value of " << params_.transaction_timeout <<
                          ". Ignoring this transaction.");
        transaction_riter = erase(pending_transactions_, transaction_riter);
      }
      else
      {
        // The motion model failed. Stop further processing of this sensor and try again next time.
        sensor_blacklist.push_back(element.sensor_name);
        ++transaction_riter;
      }
    }
  }
}

bool FixedLagSmoother::resetServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Tell all the plugins to stop
  stopPlugins();
  // Reset the optimizer state
  started_ = false;
  start_time_ = ros::TIME_MAX;
  optimization_request_ = false;
  // DANGER: The optimizationLoop() function obtains the lock optimization_mutex_ lock and the
  //         pending_transactions_mutex_ lock at the same time. We perform a parallel locking scheme here to
  //         prevent the possibility of deadlocks.
  {
    std::lock_guard<std::mutex> lock(optimization_mutex_);
    // Clear all pending transactions
    {
      std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
      pending_transactions_.clear();
    }
    // Clear the graph and marginal tracking states
    graph_->clear();
    marginal_transaction_ = fuse_core::Transaction();
    timestamp_tracking_.clear();
  }
  // Tell all the plugins to start
  startPlugins();
  // Test for auto-start
  autostart();

  return true;
}

void FixedLagSmoother::transactionCallback(
  const std::string& sensor_name,
  fuse_core::Transaction::SharedPtr transaction)
{
  // If this transaction occurs before the start time, just ignore it
  auto transaction_time = transaction->stamp();
  if (started_ && transaction_time < start_time_)
  {
    ROS_DEBUG_STREAM("Received a transaction before the start time from sensor '" << sensor_name << "'.\n" <<
                     "  start_time: " << start_time_ << ", transaction time: " << transaction_time <<
                     ", difference: " << (start_time_ - transaction_time) << "s");
    return;
  }
  {
    // We need to add the new transaction to the pending_transactions_ queue
    std::lock_guard<std::mutex> pending_transactions_lock(pending_transactions_mutex_);

    // Add the new transaction to the pending set
    // The pending set is arranged "smallest stamp last" to making popping off the back more efficient
    auto comparator = [](const ros::Time& value, const TransactionQueueElement& element)
    {
      return value >= element.stamp();
    };
    auto position = std::upper_bound(
      pending_transactions_.begin(),
      pending_transactions_.end(),
      transaction->stamp(),
      comparator);
    pending_transactions_.insert(position, {sensor_name, std::move(transaction)});  // NOLINT

    // If we haven't "started" yet..
    if (!started_)
    {
      // ...check if we should
      if (std::binary_search(params_.ignition_sensors.begin(), params_.ignition_sensors.end(), sensor_name))
      {
        started_ = true;
        start_time_ = transaction_time;
      }
      // And purge out old transactions
      //  - Either we just started and we want to purge out anything before the start time
      //  - Or we want to limit the pending size while waiting for an ignition sensor
      auto purge_time = ros::Time(0, 0);
      auto last_pending_time = pending_transactions_.front().stamp();
      if (started_)
      {
        purge_time = start_time_;
      }
      else if (ros::Time(0, 0) + params_.transaction_timeout < last_pending_time)  // ros::Time doesn't allow negatives
      {
        purge_time = last_pending_time - params_.transaction_timeout;
      }
      while (!pending_transactions_.empty() && pending_transactions_.back().stamp() < purge_time)
      {
        pending_transactions_.pop_back();
      }
    }
  }
}

}  // namespace fuse_optimizers
