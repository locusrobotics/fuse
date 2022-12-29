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
#ifndef FUSE_CORE__ASYNC_PUBLISHER_HPP_
#define FUSE_CORE__ASYNC_PUBLISHER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <fuse_core/callback_wrapper.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/node_interfaces/node_interfaces.hpp>
#include <fuse_core/publisher.hpp>
#include <fuse_core/transaction.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fuse_core
{

/**
 * @brief A publisher base class that provides an internal callback queue attached to a ROS 2 Node
 *        serviced by a local executor.
 *
 * This allows publishers derived from this base class to operate similarly to a stand alone node
 * -- any subscription or service callbacks will be executed within an independent thread. The
 *    notable differences include:
 *  - a default constructor is required (because this is a plugin)
 *  - subscriptions, services, parameter server interactions, etc. should be placed in the onInit()
 *    call instead of in the constructor
 *  - a node has already been created and attached to a local callback queue
 *  - a special callback, notifyCallback(), will be fired every time the optimizer completes an
 *    optimization cycle
 */
class AsyncPublisher : public Publisher
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(AsyncPublisher)

  /**
   * @brief Destructor
   */
  virtual ~AsyncPublisher();

  /**
   * @brief Initialize the AsyncPublisher object
   *
   * This will store the provided name and graph object, and create an internal node for this
   * instance that will use an internal callback queue serviced by a local thread. The
   * AsyncPublisher::onInit() method will be called from here, once the internal node is properly
   * configured.
   *
   * @param[in] interfaces The node interfaces to be used with the publisher
   * @param[in] name A unique name to give this plugin instance
   * @param[in] name The number of threads to use with this publisher
   * @throws runtime_error if already initialized
   */
  void initialize(
    node_interfaces::NodeInterfaces<
      node_interfaces::Base,
      node_interfaces::Clock,
      node_interfaces::Graph,
      node_interfaces::Logging,
      node_interfaces::Parameters,
      node_interfaces::Services,
      node_interfaces::TimeSource,
      node_interfaces::Timers,
      node_interfaces::Topics,
      node_interfaces::Waitables
    > interfaces, const std::string & name) override;

  /**
   * @brief Get the unique name of this publisher
   */
  const std::string & name() const override {return name_;}

  /**
   * @brief Notify the publisher that an optimization cycle is complete, and about changes to the
   *        Graph.
   *
   * This function is called by the optimizer (and in the optimizer's thread) after every
   * optimization cycle is complete. To minimize the time spent by the optimizer, this function
   * merely injects a call to AsyncPublisher::notifyCallback() into the internal callback
   * queue. This has the added benefit of simplifying threading, as all callbacks made by this
   * publisher will be executed from the same thread (or group of threads if a \p thread_count
   * > 1 is used in the constructor).
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been
   *                        added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be
   *                        performed whenever needed
   */
  void notify(Transaction::ConstSharedPtr transaction, Graph::ConstSharedPtr graph) override;

  /**
   * @brief Function to be executed whenever the optimizer is ready to receive transactions
   *
   * This method will be called by the optimizer, in the optimizer's thread, once the optimizer
   * has been initialized and is ready to receive transactions. It may also be called as part
   * of a stop-start cycle when the optimizer has been requested to reset itself. This allows
   * the publisher to reset any internal state before the optimizer begins processing after a
   * reset. No calls to notify() will happen before the optimizer calls start().
   *
   * This implementation inserts a call to onStart() into this publisher's callback queue. This
   * method then blocks until the call to onStart() has completed. This is meant to simplify
   * thread synchronization. If this publisher uses a single-threaded executor, then all
   * callbacks will fire sequentially and no semaphores are needed. If this publisher uses a
   * multithreaded executor, then normal multithreading rules apply and data accessed in more
   * than one place should be guarded.
   */
  void start() override;

  /**
   * @brief Function to be executed whenever the optimizer is no longer ready to receive
   *        transactions
   *
   * This method will be called by the optimizer, in the optimizer's thread, before the
   * optimizer shutdowns. It may also be called as part of a stop-start cycle when the
   * optimizer has been requested to reset itself. This allows the publisher to reset any
   * internal state before the optimizer begins processing after a reset. No calls to notify()
   * will happen until start() has been called again.
   *
   * This implementation inserts a call to onStop() into this publisher's callback queue. This
   * method then blocks until the call to onStop() has completed. This is meant to simplify
   * thread synchronization. If this publisher uses a single-threaded executor, then all
   * callbacks will fire sequentially and no semaphores are needed. If this publisher uses a
   * multithreaded spinner, then normal multithreading rules apply and data accessed in more
   * than one place should be guarded.
   */
  void stop() override;

protected:
  //! The callback queue used for fuse internal callbacks
  std::shared_ptr<fuse_core::CallbackAdapter> callback_queue_;

  std::string name_;  //!< The unique name for this publisher instance
  rclcpp::CallbackGroup::SharedPtr cb_group_;  //!< Internal re-entrant callback group

  //! The node interfaces
  node_interfaces::NodeInterfaces<node_interfaces::Base, node_interfaces::Waitables> interfaces_;

  //! A single/multi-threaded executor assigned to the local callback queue
  rclcpp::Executor::SharedPtr executor_;
  size_t executor_thread_count_{1};
  std::thread spinner_;  //!< Internal thread for spinning the executor
  std::atomic<bool> initialized_ = false;  //!< True if instance has been fully initialized

  /**
   * @brief Constructor
   *
   * Constructs a new publisher with a local node, a local callback queue, and internal executor.
   *
   * @param[in] thread_count The number of threads used to service the local callback queue
   */
  explicit AsyncPublisher(size_t thread_count = 1);

  /**
   * @brief Perform any required initialization for the publisher
   *
   * The node_ and member variables will be completely initialized before this call occurs.
   * Spinning of the callback queue will not begin until after the call to
   * AsyncPublisher::onInit() completes.
   *
   * Derived classes should override this method to implement any additional initialization
   * steps needed (access the parameter server, advertise, subscribe, etc.).
   */
  virtual void onInit() {}

  /**
   * @brief Callback method executed in response to the optimizer completing an optimization cycle.
   *        All variables will now have updated values.
   *
   * This method is executed using the internal callback queue and local thread(s). Derived
   * classes should override this method to implement the desired publisher behavior.
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been
   *                        added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be
   *                        performed whenever needed
   */
  virtual void notifyCallback(
    Transaction::ConstSharedPtr /*transaction*/,
    Graph::ConstSharedPtr /*graph*/) {}

  /**
   * @brief Perform any required operations to prepare for servicing calls to notify()
   *
   * This function will be called once after initialize() but before any calls to notify(). It
   * may also be called at any time after a call to stop().
   */
  virtual void onStart() {}

  /**
   * @brief Perform any required operations to clean up the internal state
   *
   * This function will be called once before destruction. It may also be called at any time
   * after a call to start(). No calls to notify() will occur after stop() is called, but
   * before start() is called.
   */
  virtual void onStop() {}

private:
  //! Stop the internal executor thread (in order to use this class again it must be re-initialized)
  void internal_stop();
};

}  // namespace fuse_core

#endif  // FUSE_CORE__ASYNC_PUBLISHER_HPP_
