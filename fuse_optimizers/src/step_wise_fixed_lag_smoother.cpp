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
#include <fuse_optimizers/step_wise_fixed_lag_smoother.h>

#include <fuse_constraints/marginalize_variables.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_optimizers/optimizer.h>
#include <ros/ros.h>

#include <algorithm>
#include <iterator>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>


namespace fuse_optimizers
{

StepWiseFixedLagSmoother::StepWiseFixedLagSmoother(
  fuse_core::Graph::UniquePtr graph,
  const ros::NodeHandle& node_handle,
  const ros::NodeHandle& private_node_handle) :
    fuse_optimizers::FixedLagSmoother(std::move(graph), node_handle, private_node_handle)
{
  params_.loadFromROS(private_node_handle);

  // add its own marginal constraints source to the optimization steps
  if (params_.optimization_steps.empty())
  {
    params_.optimization_steps.push_back({ros::this_node::getName()});
  }
  else
  {
    params_.optimization_steps[0].push_back(ros::this_node::getName());
  }

  // Start the optimization thread
  // optimization_thread_ = std::thread(&StepWiseFixedLagSmoother::optimizationLoop, this);
}

void StepWiseFixedLagSmoother::startOptimization()
{
  // Start the optimization thread
  optimization_thread_ = std::thread(&StepWiseFixedLagSmoother::optimizationLoop, this);

  // Configure a timer to trigger optimizations
  optimize_timer_ = node_handle_.createTimer<StepWiseFixedLagSmoother>(
    params_.optimization_period,
    &StepWiseFixedLagSmoother::optimizerTimerCallback,
    this);
}

StepWiseFixedLagSmoother::~StepWiseFixedLagSmoother()
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

void StepWiseFixedLagSmoother::optimizationLoop()
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
      optimization_request_ = false;
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
      processQueue(*new_transaction, lag_expiration_);
      // Skip this optimization cycle if the transaction is empty because something failed while processing the pending
      // transactions queue.
      if (new_transaction->empty())
      {
        continue;
      }
      // Prepare for selecting the marginal variables
      preprocessMarginalization(*new_transaction);
      // Combine the new transactions with any marginal transaction from the end of the last cycle
      new_transaction->merge(marginal_transaction_);

      ROS_DEBUG_STREAM("New transaction for all optimization steps: " << *new_transaction);

      for (auto &&types : params_.optimization_steps)
      {
        ROS_DEBUG_STREAM("optimization_step");
        for (auto &&type : types)
        {
          ROS_DEBUG_STREAM("type: " << type);
        }
        
        fuse_core::Transaction::SharedPtr new_transaction_for_loop = new_transaction->clone();

        // Keep only the sensor models that will be optimized in this loop
        std::unordered_set<fuse_core::UUID> constraints_for_removal;
        for (const auto& constraint : new_transaction_for_loop->addedConstraints())
        {
          if (std::find(types.begin(), types.end(), constraint.source()) == types.end())
          {
            constraints_for_removal.insert(constraint.uuid());
          }
        }

        // remove all constraints marked for removal
        for (auto &&i : constraints_for_removal)
        {
          new_transaction_for_loop->removeConstraint(i);
        }
        
        // skip if there are no elements for optimization in this optimization step
        if (boost::empty(new_transaction_for_loop->addedConstraints()) || new_transaction_for_loop->empty())
        {
          ROS_DEBUG_STREAM("Skipping because no elements in transaction");
          continue;
        }

        ROS_DEBUG_STREAM("New transaction within optimization step: " << *new_transaction_for_loop);
      
        // Update the graph
        try
        {
          graph_->update(*new_transaction_for_loop);
        }
        catch (const std::exception& ex)
        {
          std::ostringstream oss;
          oss << "Graph:\n";
          graph_->print(oss);
          oss << "\nTransaction:\n";
          new_transaction_for_loop->print(oss);

          ROS_FATAL_STREAM("Failed to update graph with transaction: " << ex.what()
                                                                      << "\nLeaving optimization loop and requesting "
                                                                          "node shutdown...\n" << oss.str());
          ros::requestShutdown();
          break;
        }
        // Optimize the entire graph
        summary_ = graph_->optimize(params_.solver_options);

        // Optimization is complete. Notify all the things about the graph changes.
        const auto new_transaction_stamp = new_transaction_for_loop->stamp();
        notify(std::move(new_transaction_for_loop), graph_->clone());

        // Abort if optimization failed. Not converging is not a failure because the solution found is usable.
        if (!summary_.IsSolutionUsable())
        {
          ROS_FATAL_STREAM("Optimization failed after updating the graph with the transaction with timestamp "
                          << new_transaction_stamp << ". Leaving optimization loop and requesting node shutdown...");
          ROS_INFO_STREAM(summary_.FullReport());
          ros::requestShutdown();
          break;
        }
      }

      // Compute a transaction that marginalizes out those variables.
      lag_expiration_ = computeLagExpirationTime();
      marginal_transaction_ = fuse_constraints::marginalizeVariables(
        ros::this_node::getName(),
        computeVariablesToMarginalize(lag_expiration_),
        *graph_);
      // Perform any post-marginal cleanup
      postprocessMarginalization(marginal_transaction_);
      // Note: The marginal transaction will not be applied until the next optimization iteration
      // Log a warning if the optimization took too long
      auto optimization_complete = ros::Time::now();
      if (optimization_complete > optimization_deadline)
      {
        ROS_WARN_STREAM_THROTTLE(10.0, "Optimization exceeded the configured duration by "
                                           << (optimization_complete - optimization_deadline) << "s");
      }
    }
  }
}

}  // namespace fuse_optimizers
