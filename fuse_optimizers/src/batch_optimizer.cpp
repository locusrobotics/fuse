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

#include <algorithm>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <fuse_core/transaction.hpp>
#include <fuse_core/util.hpp>
#include <fuse_optimizers/batch_optimizer.hpp>
#include <fuse_optimizers/optimizer.hpp>
#include <rclcpp/rclcpp.hpp>


namespace fuse_optimizers
{

BatchOptimizer::BatchOptimizer(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  fuse_core::Graph::UniquePtr graph
)
: fuse_optimizers::Optimizer(interfaces, std::move(graph)),
  start_time_(rclcpp::Time::max()),
  started_(false),
  optimization_request_(false),
  combined_transaction_(fuse_core::Transaction::make_shared())
{
  params_.loadFromROS(interfaces_);

  // Configure a timer to trigger optimizations
  optimize_timer_ = rclcpp::create_timer(
    interfaces_,
    clock_,
    params_.optimization_period,
    std::bind(&BatchOptimizer::optimizerTimerCallback, this),
    interfaces_.get_node_base_interface()->get_default_callback_group()
  );

  // Advertise a service that resets the optimizer to its initial state
  reset_service_server_ = rclcpp::create_service<std_srvs::srv::Empty>(
    interfaces_.get_node_base_interface(),
    interfaces_.get_node_services_interface(),
    fuse_core::joinTopicName(
      interfaces_.get_node_base_interface()->get_name(),
      params_.reset_service),
    std::bind(
      &BatchOptimizer::resetServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    ),
    rclcpp::ServicesQoS(),
    interfaces_.get_node_base_interface()->get_default_callback_group()
  );

  // Start the optimization thread
  optimization_thread_ = std::thread(&BatchOptimizer::optimizationLoop, this);
}

BatchOptimizer::~BatchOptimizer()
{
  // Wake up any sleeping threads
  optimization_requested_.notify_all();
  // Wait for the threads to shutdown
  if (optimization_thread_.joinable()) {
    optimization_thread_.join();
  }
}

void BatchOptimizer::applyMotionModelsToQueue()
{
  // We need get the pending transactions from the queue
  std::lock_guard<std::mutex> pending_transactions_lock(pending_transactions_mutex_);
  rclcpp::Time current_time;
  // Use the most recent transaction time as the current time
  if (!pending_transactions_.empty()) {
    // Use the most recent transaction time as the current time
    current_time = pending_transactions_.rbegin()->first;
  }

  // TODO(CH3): We might have to check for time validity here? Attempt to process each pending
  //            transaction
  while (!pending_transactions_.empty()) {
    auto & element = pending_transactions_.begin()->second;
    // Apply the motion models to the transaction
    if (!applyMotionModels(element.sensor_name, *element.transaction)) {
      if (element.transaction->stamp() + params_.transaction_timeout < current_time) {
        // Warn that this transaction has expired, then skip it.
        RCLCPP_ERROR_STREAM(
          logger_,
          "The queued transaction with timestamp "
            << element.transaction->stamp().nanoseconds() << " could not be processed after "
            << (current_time - element.transaction->stamp()).nanoseconds()
            << " seconds, which is greater than the 'transaction_timeout' value of "
            << params_.transaction_timeout.nanoseconds() << ". Ignoring this transaction.");
        pending_transactions_.erase(pending_transactions_.begin());
        continue;
      } else {
        // Stop processing future transactions. Try again next time.
        break;
      }
    }
    // Merge the sensor+motion model transactions into a combined transaction that will be applied
    // directly to the graph
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
  while (interfaces_.get_node_base_interface()->get_context()->is_valid()) {
    // Wait for the next signal to start the next optimization cycle
    {
      std::unique_lock<std::mutex> lock(optimization_requested_mutex_);
      optimization_requested_.wait(
        lock,
        [this] {
          /* *INDENT-OFF* */
          return (
            optimization_request_ ||
            !interfaces_.get_node_base_interface()->get_context()->is_valid()
          );
          /* *INDENT-ON* */
        });
    }
    // If a shutdown is requested, exit now.
    if (!interfaces_.get_node_base_interface()->get_context()->is_valid()) {
      break;
    }

    {
      std::lock_guard<std::mutex> lock(optimization_mutex_);
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
}

void BatchOptimizer::optimizerTimerCallback()
{
  // If an "ignition" transaction hasn't been received, then we can't do anything yet.
  if (!started_) {
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
  if (optimization_request_) {
    optimization_requested_.notify_one();
  }
}

bool BatchOptimizer::resetServiceCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>
)
{
  // Tell all the plugins to stop
  stopPlugins();
  // Reset the optimizer state
  {
    std::lock_guard<std::mutex> lock(optimization_requested_mutex_);
    optimization_request_ = false;
  }
  started_ = false;
  // DANGER: The optimizationLoop() function obtains the lock optimization_mutex_ lock and the
  //         combined_transaction_mutex_ lock at the same time. We perform a parallel locking scheme
  //         here to prevent the possibility of deadlocks.
  {
    std::lock_guard<std::mutex> lock(optimization_mutex_);
    // Clear the combined transation
    {
      std::lock_guard<std::mutex> lock(combined_transaction_mutex_);
      combined_transaction_ = fuse_core::Transaction::make_shared();
    }
    // Clear the graph and marginal tracking states
    graph_->clear();
  }
  // Clear all pending transactions
  // The transaction callback and the optimization timer callback are the only other locations
  // where the pending_transactions_ variable is modified. As long as the BatchOptimizer node
  // handle is single-threaded, then pending_transactions_ variable cannot be modified while the
  // reset callback is running. Therefore, there are no timing or sequence issues with exactly
  // where inside the reset service callback the pending_transactions_ are cleared.
  {
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    pending_transactions_.clear();
  }
  // Tell all the plugins to start
  startPlugins();

  return true;
}

void BatchOptimizer::transactionCallback(
  const std::string & sensor_name,
  fuse_core::Transaction::SharedPtr transaction)
{
  // Add the new transaction to the pending set
  // Either we haven't "started" yet and we want to keep a short history of transactions around
  // Or we have "started" already, and the new transaction is after the starting time.
  auto transaction_clock_type = transaction->stamp().get_clock_type();

  rclcpp::Time transaction_time = transaction->stamp();
  rclcpp::Time last_pending_time(0, 0, transaction_clock_type);  // NOTE(CH3): Uninitialized
  if (!started_ || transaction_time >= start_time_) {
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    pending_transactions_.emplace(
      transaction_time,
      TransactionQueueElement(sensor_name, std::move(transaction)));
    last_pending_time = pending_transactions_.rbegin()->first;
  }
  // If we haven't "started" yet...
  if (!started_) {
    // Check if this transaction "starts" the system
    if (sensor_models_.at(sensor_name).ignition) {
      started_ = true;
      start_time_ = transaction_time;
    }
    // Purge old transactions from the pending queue
    rclcpp::Time purge_time(0, 0, transaction_clock_type);  // NOTE(CH3): Uninitialized
    if (started_) {
      purge_time = start_time_;
    } else if (  // prevent a bad subtraction  // NOLINT
      rclcpp::Time(
        params_.transaction_timeout.nanoseconds(), last_pending_time.get_clock_type()
      ) < last_pending_time)
    {
      purge_time = last_pending_time - params_.transaction_timeout;
    }
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    auto purge_iter = pending_transactions_.lower_bound(purge_time);
    pending_transactions_.erase(pending_transactions_.begin(), purge_iter);
  }
  // If we have "started", attempt to process any pending transactions
  if (started_) {
    applyMotionModelsToQueue();
  }
}

void BatchOptimizer::setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "BatchOptimizer");

  Optimizer::setDiagnostics(status);

  status.add("Started", started_);
  {
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    status.add("Pending Transactions", pending_transactions_.size());
  }
}

}  // namespace fuse_optimizers
