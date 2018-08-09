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
#ifndef FUSE_CORE_ASYNC_SENSOR_MODEL_H
#define FUSE_CORE_ASYNC_SENSOR_MODEL_H

#include <fuse_core/graph.h>
#include <fuse_core/macros.h>
#include <fuse_core/sensor_model.h>
#include <fuse_core/transaction.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>

#include <functional>
#include <set>
#include <string>


namespace fuse_core
{

/**
 * @brief A sensor model base class that provides node handles and a private callback queue.
 *
 * A sensor model plugin is responsible for generating new constraints and passing them along to the optimizer, where
 * the actual sensor fusion takes place. The asynchronous sensor model plugin is designed similar to the nodelet
 * plugin, attempting to be as generic and easy to use as a standard ROS node.
 *
 * There are a few notable differences between the asynchronous sensor model plugin and a standard ROS node. First
 * and most obvious, the sensor model is designed as a plugin, with all of the stipulations and requirements that
 * come with all ROS plugins (must be derived from a known base class, will be default constructed). Second, the base
 * AsyncSensorModel class provides a global and private node handle, both hooked to a local callback queue and local
 * spinner. This makes it act like a full ROS node -- subscriptions trigger message callbacks, callbacks will fire
 * sequentially, etc. However, authors of derived sensor models should be aware of this fact and avoid creating
 * additional node handles, or at least take care when creating new node handles and additional callback queues.
 * Finally, the concept of "publishing a transaction" has been abused slightly. A base class
 * SensorModel::injectCallback() method has been provided. This serves the same basic purpose as publishing a
 * Transaction using a standard ROS publisher, but this method is specific to sensor models and the fuse ecosystem.
 * Under the hood, this inserts a call into the fuse optimizer's callback queue, just as publishing a message would,
 * but the memory copy and serialization are avoided.
 * 
 * Derived classes:
 * - _must_ implement the onInit() method. This method is used to configure the sensor model for operation.
 *   This includes things like accessing the parameter server and subscribing to sensor topics.
 * - may _optionally_ implement the onGraphUpdate() method. This should only be done if the derived sensor needs
 *   access to the latest values of the state variables. In many cases, sensors will simply not need that information.
 *   If the sensor does need access the to graph, the most common implementation will simply be to move the provided
 *   pointer into a class memebr variable, for use in other callbacks.
 *   @code{.cpp}
 *   void onGraphUpdate(Graph::ConstSharedPtr graph) override { this->graph_ = std::move(graph); }
 *   @endcode
 * - will _probably_ subscribe to a sensor message topic and write a custom message callback function. Within that
 *   function, the derived class will generate new constraints based on the received sensor data.
 * - _must_ call injectCallback() everytime a new constraints are generated. This is how constraints are sent to the
 *   optimizer. Otherwise, the optimizer will not know about the derived sensor's constraints, and the sensor will
 *   have no effect.
 */
class AsyncSensorModel : public SensorModel
{
public:
  SMART_PTR_ALIASES_ONLY(AsyncSensorModel);

  /**
   * @brief Destructor
   */
  virtual ~AsyncSensorModel() = default;

  /**
   * @brief Function to be executed whenever the optimizer has completed a Graph update
   *
   * This method will be called by the optimizer, in the optimizer's thread, after each Graph update is complete. This
   * implementation repackages the provided \p graph, and inserts a call to onGraphUpdate() into this sensor's
   * callback queue. This is meant to simplify thread synchronization. If this sensor uses a single-threaded spinner,
   * then all callbacks will fire sequentially and no semaphores are needed. If this sensor uses a multi-threaded
   * spinner, then normal multithreading rules apply and data accessed in more than one place should be guarded.
   *
   * @param[in] graph A read-only pointer to the graph object, allowing queries to be performed whenever needed.
   */
  void graphCallback(Graph::ConstSharedPtr graph) final;

  /**
   * @brief Perform any required post-construction initialization, such as subscribing to topics or reading from the
   * parameter server.
   *
   * This will be called for each plugin after construction and after the ROS node has been initialized. The provided
   * private node handle will be in a namespace based on the plugin's name. This should prevent conflicts and allow
   * the same plugin to be used multiple times with different settings and topics.
   *
   * @param[in] name                       A unique name to give this plugin instance
   * @param[in] transaction_callback       The function to call every time a transaction is published
   * @param[in] transaction_callback_queue The callback queue the callback function should be inserted into. This
   *                                       will likely belong to the parent of the plugin, as no attempt to spin this
   *                                       queue is performed.
   */
  void initialize(
    const std::string& name,
    TransactionCallback transaction_callback,
    ros::CallbackQueue* transaction_callback_queue) final;

  /**
   * @brief Inject the Transaction into the callback queue registered during initialize()
   *
   * It is expected that sensor model plugins will generate new constraints (packaged inside a Transaction) as a result
   * of received sensor data. Instead of the Optimizer periodically checking for new transactions, we provide a
   * "push" mechanism for the sensor model to send the transaction to the Optimizer immediately by injecting the
   * transaction into the Optimizer's callback queue. The Optimizer's transaction callback function will fire within
   * the Optimizer's callback thread(s).
   * 
   * This should be called by derived classes whenever a new Transaction is generated, probably from within the sensor
   * message callback function.
   *
   * @param[in] stamps      Any timestamps associated with the added variables. These are sent to the motion models.
   * @param[in] transaction A Transaction object describing the set of variables that have been added and removed.
   */
  void injectCallback(
    const std::set<ros::Time>& stamps,
    const Transaction::SharedPtr& transaction);

  /**
   * @brief Get the unique name of this sensor
   */
  const std::string& name() const final { return name_; }

protected:
  ros::CallbackQueue callback_queue_;  //!< The local callback queue used for all subscriptions
  std::string name_;  //!< The unique name for this sensor model instance
  ros::NodeHandle node_handle_;  //!< A node handle in the global namespace using the local callback queue
  ros::NodeHandle private_node_handle_;  //!< A node handle in the private namespace using the local callback queue
  ros::AsyncSpinner spinner_;  //!< A single/multi-threaded spinner assigned to the local callback queue
  TransactionCallback transaction_callback_;  //!< The function to be executed every time a Transaction is "published"
  ros::CallbackQueue* transaction_callback_queue_;  //!< The callback queue used for transaction callbacks. This will
                                                    //!< likely belong to the parent of the plugin, as no attempt to
                                                    //!< spin this queue is performed.

  /**
   * @brief Constructor
   *
   * Construct a new sensor model and create a local callback queue and thread spinner.
   *
   * @param[in] thread_count The number of threads used to service the local callback queue
   */
  explicit AsyncSensorModel(size_t thread_count = 1);

  /**
   * @brief Callback fired in the local callback queue thread(s) whenever a new Graph is received from the optimizer
   * 
   * Receiving a new Graph object generally means that new variables have been inserted into the Graph, and new
   * optimized values are available. To simplify synchronization between the sensor models and other consumers of
   * Graph data, the provided Graph object will never be updated be updated by anyone. Thus, only read access to the
   * Graph is provided. Information may be accessed or computed, but it cannot be changed. The optimizer provides
   * the sensors with Graph updates by sending a new Graph object, not by modifying the Graph object.
   * 
   * If the derived sensor model does not need access to the Graph object, there is not reason to overload this
   * empty implementation.
   * 
   * @param[in] graph A read-only pointer to the graph object, allowing queries to be performed whenever needed.
   */
  virtual void onGraphUpdate(Graph::ConstSharedPtr graph) {}

  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or subscribing to topics. The class's node
   * handles will be properly initialized before onInit() is called. Spinning of the callback queue will not begin
   * until after the call to onInit() completes.
   * 
   * Derived sensor models classes must implement this function, because otherwise I'm not sure how the derived
   * sensor model would actually do anything.
   */
  virtual void onInit() = 0;
};

}  // namespace fuse_core

#endif  // FUSE_CORE_ASYNC_SENSOR_MODEL_H
