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
#ifndef FUSE_CORE_ASYNC_MOTION_MODEL_H
#define FUSE_CORE_ASYNC_MOTION_MODEL_H

#include <fuse_core/motion_model.h>

#include <fuse_core/transaction.h>
#include <fuse_core/graph.h>
#include <fuse_core/callback_wrapper.h>

//#include <fuse_core/fuse_macros.h>

#include <rclcpp/rclcpp.hpp>

#include <string>


namespace fuse_core
{

/**
 * @brief A motion model base class that provides node handles and a private callback queue.
 *
 * A model model plugin is responsible for generating constraints that link together timestamps introduced by other
 * sensors in the system. The AsyncMotionModel class is designed similar to a nodelet, attempting to be as generic
 * and easy to use as a standard ROS node.
 *
 * There are a few notable differences between the AsyncMotionModel class and a standard ROS node. First and most
 * obvious, the AsyncMotionModel class is designed as a plugin, with all of the stipulations and requirements that
 * come with all ROS plugins (must be derived from a known base class, will be default constructed). Second, the
 * AsyncMotionModel class provides a global and private node handle, both hooked to a local callback queue and local
 * spinner. This makes it act like a full ROS node -- subscriptions trigger message callbacks, callbacks will fire
 * sequentially, etc. However, authors of derived classes should be aware of this fact and avoid creating additional
 * node handles, or at least take care when creating new node handles and additional callback queues. Finally, the
 * interaction of motion model nodes is best compared to a service call -- an external actor will provide a set of
 * timestamps and wait for the motion model to respond with the required set of constraints to link the timestamps
 * together (along with any previously existing timestamps). In lieu of a ROS service callback function, the
 * AsyncMotionModel class requires the applyCallback() function to be implemented. This callback will be executed
 * from the same callback queue as any other subscriptions or service callbacks.
 *
 * Derived classes:
 * - _probably_ need to implement the onInit() method. This method is used to configure the motion model for operation.
 *   This includes things like accessing the parameter server and subscribing to sensor topics.
 * - _must_ implement the applyCallback() method. This is the communication mechanism between the parent/optimizer and
 *   the derived motion model. This is how the optimizer tells the motion model what timestamps have been added, and
 *   how the motion model sends motion model constraints to the optimizer.
 * - may _optionally_ implement the onGraphUpdate() method. This should only be done if the derived motion model needs
 *   access to the latest values of the state variables. In many cases, motion models will simply not need that
 *   information. If the motion model does need access the to graph, the most common implementation will simply be to
 *   move the provided pointer into a class memebr variable, for use in other callbacks.
 *   @code{.cpp}
 *   void onGraphUpdate(Graph::ConstSharedPtr graph) override { this->graph_ = std::move(graph); }
 *   @endcode
 */
class AsyncMotionModel : public MotionModel
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(AsyncMotionModel)

  /**
   * @brief Destructor
   */
  virtual ~AsyncMotionModel() = default;

  /**
   * @brief Augment a transaction object such that all involved timestamps are connected by motion model constraints.
   *
   * This method will be called by the optimizer, in the optimizer's thread, before each sensor transaction is applied
   * to the Graph. This implementation packages a call to the pure virtual method applyCallback() and inserts it into
   * this motion model's local callback queue. This allows the applyCallback() function to be executed from the same
   * thread as any other configured callbacks. Despite the fact that the queryCallback() function call runs in a
   * different thread than this function, this function blocks until the query callback returns.
   *
   * @param[in,out] transaction The transaction object that should be augmented with motion model constraints
   * @return                    True if the motion models were generated successfully, false otherwise
   */
  bool apply(Transaction& transaction) override;

  /**
   * @brief Function to be executed whenever the optimizer has completed a Graph update
   *
   * This method will be called by the optimizer, in the optimizer's thread, after each Graph update is complete. This
   * implementation repackages the provided \p graph, and inserts a call to onGraphUpdate() into this motion model's
   * callback queue. This is meant to simplify thread synchronization. If this motion model uses a single-threaded
   * spinner, then all callbacks will fire sequentially and no semaphores are needed. If this motion model uses a
   * multi-threaded spinner, then normal multithreading rules apply and data accessed in more than one place should be
   * guarded.
   *
   * @param[in] graph A read-only pointer to the graph object, allowing queries to be performed whenever needed.
   */
  void graphCallback(Graph::ConstSharedPtr graph) override;

  /**
   * @brief Perform any required post-construction initialization, such as subscribing to topics or reading from the
   * parameter server.
   *
   * This will be called for each plugin after construction and after the ros node has been initialized. The provided
   * private node handle will be in a namespace based on the plugin's name. This should prevent conflicts and allow
   * the same plugin to be used multiple times with different settings and topics.
   *
   * @param[in] name A unique name to give this plugin instance
   */
  void initialize(const std::string& name) override;

  /**
   * @brief Get the unique name of this motion model
   */
  const std::string& name() const override { return name_; }

  /**
   * @brief Function to be executed whenever the optimizer is ready to receive transactions
   *
   * This method will be called by the optimizer, in the optimizer's thread, once the optimizer has been initialized
   * and is ready to receive transactions. It may also be called as part of a stop-start cycle when the optimizer
   * has been requested to reset itself. This allows the motion model to reset any internal state back before the
   * optimizer begins processing after a reset. No calls to apply() will happen before the optimizer calls start().
   *
   * This implementation inserts a call to onStart() into this motion model's callback queue. This method then blocks
   * until the call to onStart() has completed. This is meant to simplify thread synchronization. If this motion model
   * uses a single-threaded spinner, then all callbacks will fire sequentially and no semaphores are needed. If this
   * motion model uses a multithreaded spinner, then normal multithreading rules apply and data accessed in more than
   * one place should be guarded.
   */
  void start() override;

  /**
   * @brief Function to be executed whenever the optimizer is no longer ready to receive transactions
   *
   * This method will be called by the optimizer, in the optimizer's thread, before the optimizer shutdowns. It may
   * also be called as part of a stop-start cycle when the optimizer has been requested to reset itself. This allows
   * the motion model to reset any internal state back before the optimizer begins processing after a reset. No calls
   * to apply() will happen until start() has been called again.
   *
   * This implementation inserts a call to onStop() into this motion model's callback queue. This method then blocks
   * until the call to onStop() has completed. This is meant to simplify thread synchronization. If this motion model
   * uses a single-threaded spinner, then all callbacks will fire sequentially and no semaphores are needed. If this
   * motion model uses a multithreaded spinner, then normal multithreading rules apply and data accessed in more than
   * one place should be guarded.
   */
  void stop() override;

protected:
  std::shared_ptr<fuse_core::CallbackAdapter> callback_queue_; //!< The callback queue used for fuse internal callbacks
  std::string name_;  //!< The unique name for this motion model instance
  rclcpp::Node::SharedPtr node_;  //!< The node for this motion model
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;  //!< A single/multi-threaded spinner assigned to the local callback queue
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables_interface_;
  size_t executor_thread_count_;
  /**
   * @brief Constructor
   *
   * Construct a new motion model and create a local callback queue and thread spinner.
   *
   * @param[in] thread_count The number of threads used to service the local callback queue
   */
  explicit AsyncMotionModel(size_t thread_count = 1);

  /**
   * @brief Augment a transaction object such that all involved timestamps are connected by motion model constraints.
   *
   * This is not as straightforward as it would seem. Depending on the history of previously generated constraints,
   * fulfilling the request may require removing previously generated constraints and creating several new
   * constraints, such that all involved timestamps are linked together in a sequential chain. This function is called
   * by the MotionModel::apply() function, but it is done in such a way that *this* function will run inside
   * the derived AsyncMotionModel's local callback queue. This function is roughly analogous to providing a service
   * callback, where the caller makes a request and blocks until the request is completed.
   *
   * @param[in,out] transaction The transaction object that should be augmented with motion model constraints
   * @return                    True if the motion models were generated successfully, false otherwise
   */
  virtual bool applyCallback(Transaction& transaction) = 0;

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
   * @brief Perform any required initialization for the motion model
   *
   * This could include things like reading from the parameter server or subscribing to topics. The class's node
   * handles will be properly initialized before onInit() is called. Spinning of the callback queue will
   * not begin until after the call to onInit() completes.
   */
  virtual void onInit() {}

  /**
   * @brief Perform any required operations to prepare for servicing calls to apply()
   *
   * This function will be called once after initialize() but before any calls to apply(). It may also be called
   * at any time after a call to stop().
   */
  virtual void onStart() {}

  /**
   * @brief Perform any required operations to clean up the internal state
   *
   * This function will be called once before destruction. It may also be called at any time after a call to start().
   * No calls to apply() will occur after stop() is called, but before start() is called.
   */
  virtual void onStop() {}
};

}  // namespace fuse_core

#endif  // FUSE_CORE_ASYNC_MOTION_MODEL_H
