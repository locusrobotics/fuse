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
#include <fuse_core/async_publisher.hpp>
#include <rclcpp/contexts/default_context.hpp>

namespace fuse_core
{

AsyncPublisher::AsyncPublisher(size_t thread_count)
: name_("uninitialized"),
  executor_thread_count_(thread_count)
{
}

AsyncPublisher::~AsyncPublisher()
{
  internal_stop();
}

void AsyncPublisher::initialize(const std::string & name)
{
  if (initialized_) {
    throw std::runtime_error("Calling initialize on an already initialized AsyncPublisher!");
  }

  // Initialize internal state
  name_ = name;
  std::string node_namespace = "";

  // TODO(CH3): Pass in the context or a node to get the context from
  rclcpp::Context::SharedPtr ros_context = rclcpp::contexts::get_global_default_context();
  auto node_options = rclcpp::NodeOptions();
  node_options.context(ros_context);  // set a context to generate the node in

  // TODO(CH3): Potentially pass in the optimizer node instead of spinning a new one
  node_ = rclcpp::Node::make_shared(name_, node_namespace, node_options);

  auto executor_options = rclcpp::ExecutorOptions();
  executor_options.context = ros_context;
  executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared(
    executor_options,
    executor_thread_count_);

  callback_queue_ = std::make_shared<CallbackAdapter>(ros_context);

  // This callback group MUST be re-entrant in order to support parallelization
  cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  node_->get_node_waitables_interface()->add_waitable(
    callback_queue_, cb_group_);

  // Call the derived onInit() function to perform implementation-specific initialization
  onInit();

  // Start the executor to service the local callback queue
  executor_->add_node(node_);
  spinner_ = std::thread(
    [&]() {
      executor_->spin();
    });

  initialized_ = true;
}

void AsyncPublisher::notify(Transaction::ConstSharedPtr transaction, Graph::ConstSharedPtr graph)
{
  // Insert a call to the `notifyCallback` method into the internal callback queue.
  // This minimizes the time spent by the optimizer's thread calling this function.
  auto callback = std::make_shared<CallbackWrapper<void>>(
    std::bind(&AsyncPublisher::notifyCallback, this, std::move(transaction), std::move(graph)));
  callback_queue_->addCallback(callback);
}

void AsyncPublisher::start()
{
  auto callback =
    std::make_shared<CallbackWrapper<void>>(std::bind(&AsyncPublisher::onStart, this));
  auto result = callback->getFuture();
  callback_queue_->addCallback(callback);
  result.wait();
}

void AsyncPublisher::stop()
{
  // Optimizers can sometimes ask the models to stop -> start without initializing.
  // So we only fully wrap up if the context is no longer valid.
  // Otherwise, we only reset the internal state by clearing the callback queue and calling onStop()
  if (node_->get_node_base_interface()->get_context()->is_valid()) {
    callback_queue_->removeAllCallbacks();

    auto callback = std::make_shared<CallbackWrapper<void>>(
      std::bind(&AsyncPublisher::onStop, this)
    );
    auto result = callback->getFuture();
    callback_queue_->addCallback(callback);
    result.wait();
  } else {
    internal_stop();
    onStop();
  }
}

void AsyncPublisher::internal_stop()
{
  executor_->cancel();  // Try to cancel ASAP

  // Just in case the executor wasn't spinning, schedule a call to "stop"
  auto callback = std::make_shared<CallbackWrapper<void>>(
    [&]() {executor_->cancel();}
  );
  callback_queue_->addCallback(callback);

  if (spinner_.joinable()) {
    spinner_.join();
  }
  executor_->remove_node(node_);

  // Reset callback queue, which also removes maybe-unused call to cancel()
  callback_queue_->removeAllCallbacks();
}

}  // namespace fuse_core
