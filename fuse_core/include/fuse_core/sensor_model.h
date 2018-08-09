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
#ifndef FUSE_CORE_SENSOR_MODEL_H
#define FUSE_CORE_SENSOR_MODEL_H

#include <fuse_core/graph.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <ros/callback_queue.h>

#include <functional>
#include <set>
#include <string>


namespace fuse_core
{

/**
 * @brief The signature of the callback function that will be executed for every generated transaction object.
 */
using TransactionCallback = std::function<void(const std::set<ros::Time>& stamps,
                                               const Transaction::SharedPtr& transaction)>;

/**
 * @brief The interface definiton for sensor model plugins in the fuse ecosystem.
 *
 * A sensor model plugin is responsible for generating new constraints and passing them along to the optimizer, where
 * the actual sensor fusion takes place. This class defines the basic interface between the sensor model and the
 * optimizer. If you are developing your own sensor model, look at the AsyncSensorModel class, as this provides
 * additional features that make the sensor model act similar to a ROS node or nodelet.
 *
 * The interface SensorModel class provides the injectCallback() function. This function serves the same basic purpose
 * as publishing a Transaction using a standard ROS publisher, but this method is specific to sensor models and the
 * fuse ecosystem. Under the hood, this inserts a call into the fuse optimizer's callback queue, just as publishing a
 * message would, but the memory copy and serialization are avoided. Derived classes should call this method whenever
 * they are ready to send a fuse_core::Transaction object to the optimizer.
 */
class SensorModel
{
public:
  SMART_PTR_ALIASES_ONLY(SensorModel);

  /**
   * @brief Destructor
   */
  virtual ~SensorModel() = default;

  /**
   * @brief Function to be executed whenever the optimizer has completed a Graph update
   *
   * This method will be called by the optimizer, in the optimizer's thread, after each Graph update is complete. This
   * generally means that new variables have been inserted into the Graph, and new optimized values are available.
   * To simplify synchronization between the sensor models and other consumers of Graph data, the provided Graph
   * object will never be updated by anyone. Thus, only read access to the Graph is provided. Information may be
   * accessed or computed, but it cannot be changed. The optimizer provides the sensors with Graph updates by
   * sending a new Graph object, not by modifying this Graph object.
   *
   * @param[in] graph A read-only pointer to the graph object, allowing queries to be performed whenever needed.
   */
  virtual void graphCallback(Graph::ConstSharedPtr graph) {}

   /**
   * @brief Perform any required post-construction initialization, such as subscribing to topics or reading from the
   * parameter server.
   *
   * This will be called on each plugin after construction, and after the ROS node has been initialized. Plugins are
   * encouraged to subnamespace any of their parameters to prevent conflicts and allow the same plugin to be used
   * multiple times with different settings and topics.
   *
   * @param[in] name                       A unique name to give this plugin instance
   * @param[in] transaction_callback       The function to call every time a transaction is published
   * @param[in] transaction_callback_queue The callback queue the callback function should be inserted into. This
   *                                       will likely belong to the parent of the plugin, as no attempt to spin this
   *                                       queue is performed.
   */
  virtual void initialize(
    const std::string& name,
    TransactionCallback transaction_callback,
    ros::CallbackQueue* transaction_callback_queue) = 0;

 /**
   * @brief Inject a transaction callback function into a callback queue
   *
   * It is expected that sensor model plugins will generate new constraints (packaged inside a Transaction) as a result
   * of received sensor data. Instead of the Optimizer periodically checking for new transactions, we provide a
   * "push" mechanism for the sensor model to send the transaction to the Optimizer immediately by injecting the
   * transaction into the Optimizer's callback queue. The Optimizer's transaction callback function will fire within
   * the Optimizer's callback thread(s). The optimize will supply a pointer to its transaction callback function and
   * its callback queue within the initialize() call.
   *
   * @param[in] stamps                     All timestamps associated with the added variables. These are sent to the
   *                                       motion models.
   * @param[in] transaction                Object describing the set of variables and constraints that have been added
   *                                       and/or removed.
   * @param[in] transaction_callback       The callback function to inject into the callback queue
   * @param[in] transaction_callback_queue The callback queue the callback function should be inserted into.
   */
  static void injectCallback(
    const std::set<ros::Time>& stamps,
    const Transaction::SharedPtr& transaction,
    const TransactionCallback& transaction_callback,
    ros::CallbackQueue* transaction_callback_queue);

  /**
   * @brief Get the unique name of this sensor
   */
  virtual const std::string& name() const = 0;

protected:
  /**
   * @brief Default Constructor
   */
  SensorModel() = default;
};

}  // namespace fuse_core

#endif  // FUSE_CORE_SENSOR_MODEL_H
