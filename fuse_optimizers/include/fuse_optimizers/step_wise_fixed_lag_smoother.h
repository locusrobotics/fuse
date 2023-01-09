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
#ifndef FUSE_OPTIMIZERS_STEP_WISE_FIXED_LAG_SMOOTHER_H
#define FUSE_OPTIMIZERS_STEP_WISE_FIXED_LAG_SMOOTHER_H

#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_optimizers/step_wise_fixed_lag_smoother_params.h>
#include <fuse_optimizers/fixed_lag_smoother.h>
#include <fuse_optimizers/variable_stamp_index.h>
#include <ros/ros.h>
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
 * @brief A step-wise fixed-lag smoother implementation that marginalizes out variables that are older than a defined lag time
 * and optimizes step-wise adding more and more constraints from different sources
 *
 * This implementation assumes that all added variable types are either derived from the fuse_variables::Stamped class,
 * or are directly connected to at least one fuse_variables::Stamped variable via a constraint. The current time of
 * the fixed-lag smoother is determined by the newest stamp of all added fuse_variables::Stamped variables.
 *
 * During optimization:
 *  (1) a set of new variables and constraints are computed 
 *  (2) each optimization step adds the constraints with the defined source(s) to the graph
 *  (2) the augmented graph is optimized and the variable values are updated
 *  (3) steps 2 and 3 are continued until all optimization steps are completed
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
 *  - optimization_steps (struct array) The set of sources that should be optimized for each optimization step
 *                                      It is recommended to always add the ignition source(s) and motion model source(s) 
 *                                      to the first optimization step. The constraints and variables of the marginalization 
 *                                      are always added to the first optimization step.
 *    @code{.yaml}
 *    - sources: string array  (A list of strings of the names of the sources of the constraints. 
 *                              The names refer to the names that are used in the other parameters sensor_models and motion_models)
 *    - ...
 *    @endcode
 *    full example:
 *    @code{.yaml}
 *    optimization_steps:
 *     -
 *       sources:
 *         - "initial_localization_sensor"
 *         - "unicycle_motion_model"
 *     -
 *       sources:
 *         - "odometry_sensor"
 *         - "imu_sensor"
 *     -
 *       sources:
 *         - "scan_to_scan"
 *     -
 *       sources:
 *         - "scan_to_map"
 *    @endcode
 */
class StepWiseFixedLagSmoother : public FixedLagSmoother
{
public:
  FUSE_SMART_PTR_DEFINITIONS(StepWiseFixedLagSmoother);
  using ParameterType = StepWiseFixedLagSmootherParams;

  /**
   * @brief Constructor
   *
   * @param[in] graph               The derived graph object. This allows different graph implementations to be used
   *                                with the same optimizer code.
   * @param[in] node_handle         A node handle in the global namespace
   * @param[in] private_node_handle A node handle in the node's private namespace
   */
  StepWiseFixedLagSmoother(
    fuse_core::Graph::UniquePtr graph,
    const ros::NodeHandle& node_handle = ros::NodeHandle(),
    const ros::NodeHandle& private_node_handle = ros::NodeHandle("~"));

  /**
   * @brief Destructor
   */
  virtual ~StepWiseFixedLagSmoother();

  /**
   * @brief Function that optimizes all constraints, designed to be run in a separate thread.
   *
   * This function waits for an optimization or shutdown signal, then either calls optimize() or exits appropriately.
   */
  void optimizationLoop();

  void startOptimization() override;

protected:

  // Read-only after construction
  ParameterType params_;  //!< Configuration settings for this fixed-lag smoother

};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_STEP_WISE_FIXED_LAG_SMOOTHER_H
