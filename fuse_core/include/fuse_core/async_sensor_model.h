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

#include <fuse_core/sensor_model.h>

#include <fuse_core/transaction.h>
#include <fuse_core/graph.h>
#include <fuse_core/callback_wrapper.h>

//#include <fuse_core/fuse_macros.h>

#include <functional>
#include <string>


namespace fuse_core
{

/**
 * @brief A sensor model base class that provides node handles and a private callback queue.
 *
 * A sensor model plugin is responsible for generating new constraints, packaging them in a fuse_core::Transaction
 * object, and passing them along to the optimizer. The asynchronous sensor model plugin is designed similar to the
 * nodelet plugin, attempting to be as generic and easy to use as a standard ROS node.
 *
 * There are a few notable differences between the asynchronous sensor model plugin and a standard ROS node. First
 * and most obvious, the sensor model is designed as a plugin, with all of the stipulations and requirements that
 * come with all ROS plugins (must be derived from a known base class, will be default constructed). Second, the base
 * AsyncSensorModel class provides a global and private node handle, both hooked to a local callback queue and local
 * spinner. This makes it act like a full ROS node -- subscriptions trigger message callbacks, callbacks will fire
 * sequentially, etc. However, authors of derived sensor models should be aware of this fact and avoid creating
 * additional node handles, or at least take care when creating new node handles and additional callback queues.
 * Finally, instead of publishing the generated constraints using a ROS publisher, the asynchronous sensor model
 * provides a "sendTransaction()" method that should be called whenever the sensor is ready to send a
 * fuse_core::Transaction object to the Optimizer. Under the hood, this executes the TransactionCallback that was
 * provided during plugin construction.
 *
 * Derived classes:
 * - _must_ implement the onInit() method. This method is used to configure the sensor model for operation.
 *   This includes things like accessing the parameter server and subscribing to sensor topics.
 * - may _optionally_ implement the onGraphUpdate() method. This should only be done if the derived sensor needs
 *   access to the latest values of the state variables. In many cases, sensors will simply not need that information.
 *   If the sensor does need access to the graph, the most common implementation will simply be to move the provided
 *   pointer into a class member variable, for use in other callbacks.
 *   @code{.cpp}
 *   void onGraphUpdate(Graph::ConstSharedPtr graph) override { this->graph_ = std::move(graph); }
 *   @endcode
 * - will _probably_ subscribe to a sensor message topic and write a custom message callback function. Within that
 *   function, the derived class will generate new constraints based on the received sensor data.
 * - _must_ call sendTransaction() every time new constraints are generated. This is how constraints are sent to the
 *   optimizer. Otherwise, the optimizer will not know about the derived sensor's constraints, and the sensor will
 *   have no effect.
 */
class AsyncSensorModel : public SensorModel
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(AsyncSensorModel)

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
  void graphCallback(Graph::ConstSharedPtr graph) override;

  /**
   * @brief Perform any required post-construction initialization, such as subscribing to topics or reading from the
   * parameter server.
   *
   * This will be called for each plugin after construction and after the ROS node has been initialized. The provided
   * private node handle will be in a namespace based on the plugin's name. This should prevent conflicts and allow
   * the same plugin to be used multiple times with different settings and topics.
   *
   * @param[in] name                 A unique name to give this plugin instance
   * @param[in] transaction_callback The function to call every time a transaction is published
   */
  void initialize(
    const std::string& name,
    TransactionCallback transaction_callback) override;

  /**
   * @brief Send a transaction to the Optimizer
   *
   * It is expected that sensor model plugins will generate new constraints (packaged inside a Transaction) as a result
   * of received sensor data. Instead of the Optimizer periodically checking for new transactions, we provide a
   * "push" mechanism for the sensor model to send the transaction to the Optimizer immediately by calling the callback
   * function provided by the Optimizer. This function will be executed by the SensorModel's thread.
   *
   * This should be called by derived classes whenever a new Transaction is generated, probably from within the sensor
   * message callback function.
   *
   * @param[in] transaction A Transaction object describing the set of variables that have been added and removed.
   */
  void sendTransaction(Transaction::SharedPtr transaction);

  /**
   * @brief Get the unique name of this sensor
   */
  const std::string& name() const override { return name_; }

  /**
   * @brief Function to be executed whenever the optimizer is ready to receive transactions
   *
   * This method will be called by the optimizer, in the optimizer's thread, once the optimizer has been initialized
   * and is ready to receive transactions. It may also be called as part of a stop-start cycle when the optimizer
   * has been requested to reset itself. This allows the sensor model to reset any internal state before the
   * optimizer begins processing after a reset.
   *
   * This implementation inserts a call to onStart() into this sensor model's callback queue. This method then blocks
   * until the call to onStart() has completed. This is meant to simplify thread synchronization. If this sensor model
   * uses a single-threaded spinner, then all callbacks will fire sequentially and no semaphores are needed. If this
   * sensor model uses a multithreaded spinner, then normal multithreading rules apply and data accessed in more than
   * one place should be guarded.
   */
  void start() override;

  /**
   * @brief Function to be executed whenever the optimizer is no longer ready to receive transactions
   *
   * This method will be called by the optimizer, in the optimizer's thread, before the optimizer shutdowns. It may
   * also be called as part of a stop-start cycle when the optimizer has been requested to reset itself. This allows
   * the sensor model to reset any internal state before the optimizer begins processing after a reset.
   *
   * This implementation inserts a call to onStop() into this sensor model's callback queue. This method then blocks
   * until the call to onStop() has completed. This is meant to simplify thread synchronization. If this sensor model
   * uses a single-threaded spinner, then all callbacks will fire sequentially and no semaphores are needed. If this
   * sensor model uses a multithreaded spinner, then normal multithreading rules apply and data accessed in more than
   * one place should be guarded.
   */
  void stop() override;

protected:
  std::shared_ptr<fuse_core::CallbackAdapter> callback_queue_; //!< The callback queue used for fuse internal callbacks
  std::string name_;  //!< The unique name for this sensor model instance
  rclcpp::Node::SharedPtr node_;  //!< The node for this sensor model
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;  //!< A single/multi-threaded spinner assigned to the local callback queue
  TransactionCallback transaction_callback_;  //!< The function to be executed every time a Transaction is "published"
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables_interface_;
  size_t executor_thread_count_;

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
  virtual void onGraphUpdate(Graph::ConstSharedPtr /*graph*/) {}

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
  virtual void onInit() {}

  /**
   * @brief Perform any required operations to prepare for sending transactions to the optimizer
   *
   * This function will be called once after initialize(). It may also be called at any time after a call to stop().
   *
   * The sensor model must not send any transactions to the optimizer before onStart() is called.
   */
  virtual void onStart() {}

  /**
   * @brief Perform any required operations to stop sending transactions to the optimizer
   *
   * This function will be called once before destruction. It may also be called at any time after a call to start().
   *
   * The sensor model must not send any transactions to the optimizer after stop() is called.
   */
  virtual void onStop() {}
};

}  // namespace fuse_core

#endif  // FUSE_CORE_ASYNC_SENSOR_MODEL_H
