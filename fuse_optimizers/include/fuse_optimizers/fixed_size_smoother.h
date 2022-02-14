/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Locus Robotics
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
#ifndef FUSE_OPTIMIZERS_FIXED_SIZE_SMOOTHER_H
#define FUSE_OPTIMIZERS_FIXED_SIZE_SMOOTHER_H

#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_optimizers/fixed_size_smoother_params.h>
#include <fuse_optimizers/variable_stamp_index.h>
#include <fuse_optimizers/windowed_optimizer.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <mutex>
#include <string>
#include <vector>

namespace fuse_optimizers
{
/**
 * @brief A fixed-2ize smoother implementation that marginalizes out variables that are older than a defined buffer size
 *
 * This implementation assumes that all added variable types are either derived from the fuse_variables::Stamped class,
 * or are directly connected to at least one fuse_variables::Stamped variable via a constraint. The current time of
 * the fixed-size smoother is determined by the newest stamp of all added fuse_variables::Stamped variables.
 *
 * During optimization:
 *  (1) new variables and constraints are added to the graph
 *  (2) the augmented graph is optimized and the variable values are updated
 *  (3) all motion models, sensors, and publishers are notified of the updated graph
 *  (4) all variables outside of the fixed buffer are marginalized out
 *
 * Optimization is performed at a fixed frequency, controlled by the \p optimization_frequency parameter. Received
 * sensor transactions are queued while the optimization is processing, then applied to the graph at the start of the
 * next optimization cycle. If the previous optimization is not yet complete when the optimization period elapses,
 * then a warning will be logged but a new optimization will *not* be started. The previous optimization will run to
 * completion, and the next optimization will not begin until the next scheduled optimization period.
 *
 * Parameters:
 *  - num_states (float, default: 10) The number of unique timestamped states in the window
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
class FixedSizeSmoother : public WindowedOptimizer
{
public:
  SMART_PTR_DEFINITIONS(FixedSizeSmoother);
  using ParameterType = FixedSizeSmootherParams;

  /**
   * @brief Constructor
   *
   * @param[in] graph               The derived graph object. This allows different graph implementations to be used
   *                                with the same optimizer code.
   * @param[in] params              A structure containing all of the configuration parameters required by the
   *                                fixed-size smoother
   * @param[in] node_handle         A node handle in the global namespace
   * @param[in] private_node_handle A node handle in the node's private namespace
   */
  FixedSizeSmoother(fuse_core::Graph::UniquePtr graph, const ParameterType::SharedPtr& params,
                    const ros::NodeHandle& node_handle = ros::NodeHandle(),
                    const ros::NodeHandle& private_node_handle = ros::NodeHandle("~"));

  /**
   * @brief Constructor
   *
   * The parameters will be loaded from the ROS parameter server using the private node handle
   *
   * @param[in] graph               The derived graph object. This allows different graph implementations to be used
   *                                with the same optimizer code.
   * @param[in] node_handle         A node handle in the global namespace
   * @param[in] private_node_handle A node handle in the node's private namespace
   */
  explicit FixedSizeSmoother(fuse_core::Graph::UniquePtr graph, const ros::NodeHandle& node_handle = ros::NodeHandle(),
                             const ros::NodeHandle& private_node_handle = ros::NodeHandle("~"));

protected:
  // Read-only after construction
  ParameterType::SharedPtr params_;  //!< Configuration settings for this fixed-Size smoother

  // Guarded by mutex_
  std::mutex mutex_;                       //!< Mutex held while the fixed-size smoother variables are modified
  VariableStampIndex timestamp_tracking_;  //!< Object that tracks the timestamp associated with each variable

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
  void preprocessMarginalization(const fuse_core::Transaction& new_transaction) override;

  /**
   * @brief Compute the set of variables that should be marginalized from the graph
   *
   * This will be called after \p preprocessMarginalization() and after the graph has been updated with the any
   * previous marginal transactions and new transactions.
   *
   * @param[in] Size_expiration The oldest timestamp that should remain in the graph
   * @return A container with the set of variables to marginalize out. Order of the variables is not specified.
   */
  std::vector<fuse_core::UUID> computeVariablesToMarginalize() override;

  /**
   * @brief Perform any required post-marginalization bookkeeping
   *
   * The transaction containing the actual changed to the graph is supplied. This will be called before the
   * transaction is actually applied to the graph.
   *
   * @param[in] marginal_transaction The actual changes to the graph caused my marginalizing out the requested
   *                                 variables.
   */
  void postprocessMarginalization(const fuse_core::Transaction& marginal_transaction) override;

  /**
   * @brief Determine if a new transaction should be applied to the graph
   *
   * Test if the transaction is within the defined Size window of the smoother.
   *
   * @param[in] sensor_name - The name of the sensor that produced the provided transaction
   * @param[in] transaction - The transaction to be validated
   */
  bool validateTransaction(const std::string& sensor_name, const fuse_core::Transaction& transaction) override;

  /**
   * @brief Perform any required operations whenever the optimizer is reset
   */
  void onReset() override;
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_FIXED_SIZE_SMOOTHER_H
