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

#ifndef FUSE_OPTIMIZERS__BATCH_OPTIMIZER_HPP_
#define FUSE_OPTIMIZERS__BATCH_OPTIMIZER_HPP_

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fuse_core/graph.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_optimizers/batch_optimizer_params.hpp>
#include <fuse_optimizers/optimizer.hpp>
#include <fuse_graphs/hash_graph.hpp>


namespace fuse_optimizers
{

/**
 * @brief A simple optimizer implementation that uses batch optimization
 *
 * The batch optimization takes place in a separate thread. Received sensor transactions are queued
 * while the optimization is processing, then applied to the graph at the start of the next
 * optimization cycle. Optimization cycles are started at a fixed frequency. If the previous
 * optimization is not yet complete when the optimization period elapses, then a new optimization
 * will not be started. The previous optimization will run to completion, and the next optimization
 * will not begin until the next scheduled optimization period. For batch optimization problems that
 * continuously grow in size, this means that the optimization period is not overly important. The
 * time spent waiting versus the time spent optimizing will approach zero as the problem size
 * increases.
 *
 * Parameters:
 *  - motion_models (struct array) The set of motion model plugins to load
 *    @code{.yaml}
 *    - name: string  (A unique name for this motion model)
 *      type: string  (The plugin loader class string for the desired motion model type)
 *    - ...
 *    @endcode
 *  - optimization_period (float, default: 10.0) The minimum time delay, in seconds, between
 *                                               optimization cycles.
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
 *      motion_models: [name1, name2, ...]  (An optional list of motion model names that should be
 *                                           applied)
 *    - ...
 *    @endcode
 *
 *  - transaction_timeout (float, default: 10.0) The maximum time to wait for motion models to be
 *                                               generated for a received transactions. Transactions
 *                                               are processes sequentially, so no new transactions
 *                                               will be added to the graph while waiting for motion
 *                                               models to be generated. Once the timeout expires,
 *                                               that transaction will be deleted from the queue.
 */
class BatchOptimizer : public Optimizer
{
public:
  FUSE_SMART_PTR_DEFINITIONS(BatchOptimizer)
  using ParameterType = BatchOptimizerParams;

  /**
   * @brief Constructor
   *
   * @param[in] interfaces          The node interfaces for the node driving the optimizer
   * @param[in] graph               The graph used with the optimizer
   */
  BatchOptimizer(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    fuse_core::Graph::UniquePtr graph = nullptr
  );

  /**
   * @brief Destructor
   */
  virtual ~BatchOptimizer();

protected:
  /**
   * Structure containing the information required to process a transaction after it was received.
   */
  struct TransactionQueueElement
  {
    std::string sensor_name;
    fuse_core::Transaction::SharedPtr transaction;

    TransactionQueueElement(
      const std::string & sensor_name,
      fuse_core::Transaction::SharedPtr transaction)
    : sensor_name(sensor_name),
      transaction(std::move(transaction)) {}
  };

  /**
   * @brief Queue of Transaction objects, sorted by timestamp.
   *
   * Note: Because the queue is sorted on insert, the insert operation is O(logN). However, it is
   * far more likely that the computer will run out of memory before the insertion time grows large
   * enough to surpass the sensor update period.
   */
  using TransactionQueue = std::multimap<rclcpp::Time, TransactionQueueElement>;

  fuse_core::Transaction::SharedPtr combined_transaction_;  //!< Transaction used aggregate
                                                            //!< constraints and variables from
                                                            //!< multiple sensors and motions models
                                                            //!< before being applied to the graph.
  std::mutex combined_transaction_mutex_;  //!< Synchronize access to the combined transaction
                                           //!< across different threads
  ParameterType params_;  //!< Configuration settings for this optimizer
  std::atomic<bool> optimization_request_;  //!< Flag to trigger a new optimization
  std::condition_variable optimization_requested_;  //!< Condition variable used by the optimization
                                                    //!< thread to wait until a new optimization is
                                                    //!< requested by the main thread
  std::mutex optimization_requested_mutex_;  //!< Required condition variable mutex
  std::thread optimization_thread_;  //!< Thread used to run the optimizer as a background process
  rclcpp::TimerBase::SharedPtr optimize_timer_;  //!< Trigger an optimization operation at a fixed
                                                 //!< frequency
  TransactionQueue pending_transactions_;  //!< The set of received transactions that have not been
                                           //!< added to the optimizer yet. Transactions are added
                                           //!< by the main thread, and removed and processed by the
                                           //!< optimization thread.
  std::mutex pending_transactions_mutex_;  //!< Synchronize modification of the
                                           //!< pending_transactions_ container
  rclcpp::Time start_time_;  //!< The timestamp of the first ignition sensor transaction
  bool started_;  //!< Flag indicating the optimizer is ready/has received a transaction from an
                  //!< ignition sensor

  /**
   * @brief Generate motion model constraints for pending transactions
   *
   * Transactions are processed sequentially based on timestamp. If motion models are successfully
   * generated for a pending transactions, that transaction is merged into the combined_transaction_
   * variable and removed from the pending queue. If motion models fail to generate after the
   * configured transaction_timeout_, the transaction will be deleted from the pending queue and a
   * warning will be displayed.
   */
  void applyMotionModelsToQueue();

  /**
   * @brief Function that optimizes all constraints, designed to be run in a separate thread.
   *
   * This function waits for an optimization or shutdown signal, then either calls optimize() or
   * exits appropriately.
   */
  void optimizationLoop();

  /**
   * @brief Callback fired at a fixed frequency to trigger a new optimization cycle.
   *
   * This callback checks if a current optimization cycle is still running. If not, a new
   * optimization cycle is started. If so, we simply wait for the next timer event to start another
   * optimization cycle.
   */
  void optimizerTimerCallback();

  /**
   * @brief Callback fired every time the SensorModel plugin creates a new transaction
   *
   * This callback is responsible for ensuring all associated motion models are applied before any
   * other processing takes place. See Optimizer::applyMotionModels() for a helper function that
   * does just that.
   *
   * This implementation shares ownership of the transaction object.
   *
   * @param[in] name        The name of the sensor that produced the Transaction
   * @param[in] stamps      Any timestamps associated with the added variables. These are sent to
   *                        the motion models to generate connected constraints.
   * @param[in] transaction The populated Transaction object created by the loaded SensorModel
   *                        plugin
   */
  void transactionCallback(
    const std::string & sensor_name,
    fuse_core::Transaction::SharedPtr transaction) override;

  /**
   * @brief Update and publish diagnotics
   * @param[in] status The diagnostic status
   */
  void setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status) override;
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS__BATCH_OPTIMIZER_HPP_
