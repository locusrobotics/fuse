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
#ifndef FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_H
#define FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_H

#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_optimizers/fixed_lag_smoother_params.h>
#include <fuse_optimizers/optimizer.h>
#include <fuse_optimizers/variable_stamp_index.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/Empty.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief A fixed-lag smoother implementation that marginalizes out variables that are older than a defined lag time
 *
 * This implementation assumes that all added variable types are either derived from the fuse_variables::Stamped class,
 * or are directly connected to at least one fuse_variables::Stamped variable via a constraint. The current time of
 * the fixed-lag smoother is determined by the newest stamp of all added fuse_variables::Stamped variables.
 *
 * During optimization:
 *  (1) new variables and constraints are added to the graph
 *  (2) the augmented graph is optimized and the variable values are updated
 *  (3) all motion models, sensors, and publishers are notified of the updated graph
 *  (4) all variables older than "current time - lag duration" are marginalized out.
 *
 * Optimization is performed at a fixed frequency, controlled by the \p optimization_frequency parameter. Received
 * sensor transactions are queued while the optimization is processing, then applied to the graph at the start of the
 * next optimization cycle. If the previous optimization is not yet complete when the optimization period elapses,
 * then a warning will be logged but a new optimization will *not* be started. The previous optimization will run to
 * completion, and the next optimization will not begin until the next scheduled optimization period.
 *
 * Parameters:
 *  - lag_duration (float, default: 5.0) The duration of the smoothing window in seconds
 *  - motion_models (struct array) The set of motion model plugins to load
 *    @code{.yaml}
 *    - name: string  (A unique name for this motion model)
 *      type: string  (The plugin loader class string for the desired motion model type)
 *    - ...
 *    @endcode
 *  - optimization_frequency (float, default: 10.0) The target frequency for optimization cycles. If an optimization
 *                                                  takes longer than expected, an optimization cycle may be skipped.
 *  - publishers (struct array) The set of publisher plugins to load
 *    @code{.yaml}
 *    - name: string  (A unique name for this publisher)
 *      type: string  (The plugin loader class string for the desired publisher type)
 *    - ...
 *    @endcode
 *  - sensor_models (struct array) The set of sensor model plugins to load
 *    @code{.yaml}
 *    - name: string  (A unique name for this sensor model)
 *      type: string  (The plugin loader class string for the desired sensor model type)
 *      motion_models: [name1, name2, ...]  (An optional list of motion model names that should be applied)
 *    - ...
 *    @endcode
 *  - transaction_timeout (float, default: 0.10) The maximum time to wait for motion models to be generated for a
 *                                               received transactions. Transactions are processes sequentially, so
 *                                               no new transactions will be added to the graph while waiting for
 *                                               motion models to be generated. Once the timeout expires, that
 *                                               transaction will be deleted from the queue.
 */
class FixedLagSmoother : public Optimizer
{
public:
  FUSE_SMART_PTR_DEFINITIONS(FixedLagSmoother)
  using ParameterType = FixedLagSmootherParams;

  /**
   * @brief Constructor
   *
   * @param[in] graph               The derived graph object. This allows different graph implementations to be used
   *                                with the same optimizer code.
   * @param[in] node_handle         A node handle in the global namespace
   * @param[in] private_node_handle A node handle in the node's private namespace
   */
  FixedLagSmoother(
    fuse_core::Graph::UniquePtr graph,
    const ros::NodeHandle& node_handle = ros::NodeHandle(),
    const ros::NodeHandle& private_node_handle = ros::NodeHandle("~"));

  /**
   * @brief Destructor
   */
  virtual ~FixedLagSmoother();

protected:
  /**
   * Structure containing the information required to process a transaction after it was received.
   */
  struct TransactionQueueElement
  {
    std::string sensor_name;
    fuse_core::Transaction::SharedPtr transaction;

    const rclcpp::Time& stamp() const { return transaction->stamp(); }
    const rclcpp::Time& minStamp() const { return transaction->minStamp(); }
    const rclcpp::Time& maxStamp() const { return transaction->maxStamp(); }
  };

  /**
   * @brief Queue of Transaction objects, sorted by timestamp.
   *
   * Note: Because the queue size of the fixed-lag smoother is expected to be small, the sorted queue is implemented
   * using a std::vector. The queue size must exceed several hundred entries before a std::set will outperform a
   * sorted vector.
   *
   * Also, we sort the queue with the smallest stamp last. This allows us to clear the queue using the more
   * efficient pop_back() operation.
   */
  using TransactionQueue = std::vector<TransactionQueueElement>;

  // Read-only after construction
  std::thread optimization_thread_;  //!< Thread used to run the optimizer as a background process
  ParameterType params_;  //!< Configuration settings for this fixed-lag smoother

  // Inherently thread-safe
  std::atomic<bool> ignited_;  //!< Flag indicating the optimizer has received a transaction from an ignition sensor
                               //!< and it is queued but not processed yet
  std::atomic<bool> optimization_running_;  //!< Flag indicating the optimization thread should be running
  std::atomic<bool> started_;  //!< Flag indicating the optimizer has received a transaction from an ignition sensor

  // Guarded by pending_transactions_mutex_
  std::mutex pending_transactions_mutex_;  //!< Synchronize modification of the pending_transactions_ container
  TransactionQueue pending_transactions_;  //!< The set of received transactions that have not been added to the
                                           //!< optimizer yet. Transactions are added by the main thread, and removed
                                           //!< and processed by the optimization thread.

  // Guarded by optimization_mutex_
  std::mutex optimization_mutex_;  //!< Mutex held while the graph is begin optimized
  // fuse_core::Graph* graph_ member from the base class
  rclcpp::Time lag_expiration_;  //!< The oldest stamp that is inside the fixed-lag smoother window
  fuse_core::Transaction marginal_transaction_;  //!< The marginals to add during the next optimization cycle
  VariableStampIndex timestamp_tracking_;  //!< Object that tracks the timestamp associated with each variable
  ceres::Solver::Summary summary_;  //!< Optimization summary, written by optimizationLoop and read by setDiagnostics

  // Guarded by optimization_requested_mutex_
  std::mutex optimization_requested_mutex_;  //!< Required condition variable mutex
  rclcpp::Time optimization_deadline_;  //!< The deadline for the optimization to complete. Triggers a warning if exceeded.
  bool optimization_request_;  //!< Flag to trigger a new optimization
  std::condition_variable optimization_requested_;  //!< Condition variable used by the optimization thread to wait
                                                    //!< until a new optimization is requested by the main thread

  // Guarded by start_time_mutex_
  mutable std::mutex start_time_mutex_;  //!< Synchronize modification to the start_time_ variable
  rclcpp::Time start_time_;  //!< The timestamp of the first ignition sensor transaction

  // Ordering ROS objects with callbacks last
  rclcpp::TimerBase::SharedPtr optimize_timer_;  //!< Trigger an optimization operation at a fixed frequency
  ros::ServiceServer reset_service_server_;  //!< Service that resets the optimizer to its initial state

  /**
   * @brief Automatically start the smoother if no ignition sensors are specified
   */
  void autostart();

  /**
   * @brief Perform any required preprocessing steps before \p computeVariablesToMarginalize() is called
   *
   * All new transactions that will be applied to the graph are provided. This does not include the marginal
   * transaction that is computed later.
   *
   * This method will be called before the graph has been updated.
   *
   * @param[in] new_transaction All new, non-marginal-related transactions that *will be* applied to the graph
   */
  void preprocessMarginalization(const fuse_core::Transaction& new_transaction);

  /**
   * @brief Compute the oldest timestamp that is part of the configured lag window
   */
  rclcpp::Time computeLagExpirationTime() const;

  /**
   * @brief Compute the set of variables that should be marginalized from the graph
   *
   * This will be called after \p preprocessMarginalization() and after the graph has been updated with the any
   * previous marginal transactions and new transactions.
   *
   * @param[in] lag_expiration The oldest timestamp that should remain in the graph
   * @return A container with the set of variables to marginalize out. Order of the variables is not specified.
   */
  std::vector<fuse_core::UUID> computeVariablesToMarginalize(const rclcpp::Time& lag_expiration);

  /**
   * @brief Perform any required post-marginalization bookkeeping
   *
   * The transaction containing the actual changed to the graph is supplied. This will be called before the
   * transaction is actually applied to the graph.
   *
   * @param[in] marginal_transaction The actual changes to the graph caused my marginalizing out the requested
   *                                 variables.
   */
  void postprocessMarginalization(const fuse_core::Transaction& marginal_transaction);

  /**
   * @brief Function that optimizes all constraints, designed to be run in a separate thread.
   *
   * This function waits for an optimization or shutdown signal, then either calls optimize() or exits appropriately.
   */
  void optimizationLoop();

  /**
   * @brief Callback fired at a fixed frequency to trigger a new optimization cycle.
   *
   * This callback checks if a current optimization cycle is still running. If not, a new optimization cycle is started.
   * If so, we simply wait for the next timer event to start another optimization cycle.
   */
  void optimizerTimerCallback();

  /**
   * @brief Generate motion model constraints for pending transactions and combine them into a single transaction
   *
   * Transactions are processed sequentially based on timestamp. If motion models are successfully generated for a
   * pending transactions, that transaction is merged into the combined transaction and removed from the pending
   * queue. If motion models fail to generate after the configured transaction_timeout_, the transaction will be
   * deleted from the pending queue and a warning will be displayed.
   *
   * @param[out] transaction The transaction object to be augmented with pending motion model and sensor transactions
   * @param[in]  lag_expiration The oldest timestamp that should remain in the graph
   */
  void processQueue(fuse_core::Transaction& transaction, const rclcpp::Time& lag_expiration);

  /**
   * @brief Service callback that resets the optimizer to its original state
   */
  bool resetServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * @brief Thread-safe read-only access to the timestamp of the first transaction
   */
  rclcpp::Time getStartTime() const
  {
    std::lock_guard<std::mutex> lock(start_time_mutex_);
    return start_time_;
  }

  /**
   * @brief Thread-safe write access to the optimizer start time
   */
  void setStartTime(const rclcpp::Time& start_time)
  {
    std::lock_guard<std::mutex> lock(start_time_mutex_);
    start_time_ = start_time;
  }

  /**
   * @brief Callback fired every time the SensorModel plugin creates a new transaction
   *
   * This callback is responsible for ensuring all associated motion models are applied before any other processing
   * takes place. See Optimizer::applyMotionModels() for a helper function that does just that.
   *
   * This implementation shares ownership of the transaction object.
   *
   * @param[in] name        The name of the sensor that produced the Transaction
   * @param[in] transaction The populated Transaction object created by the loaded SensorModel plugin
   */
  void transactionCallback(
    const std::string& sensor_name,
    fuse_core::Transaction::SharedPtr transaction) override;

  /**
   * @brief Update and publish diagnotics
   * @param[in] status The diagnostic status
   */
  void setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) override;
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_H
