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
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/callback_wrapper.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/transaction.hpp>
#include <rclcpp/contexts/default_context.hpp>

namespace fuse_core
{

AsyncSensorModel::AsyncSensorModel(size_t thread_count)
: name_("uninitialized"),
  executor_thread_count_(thread_count)
{
}

AsyncSensorModel::~AsyncSensorModel()
{
  internal_stop();
}

void AsyncSensorModel::initialize(
  node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name,
  TransactionCallback transaction_callback)
{
  if (initialized_) {
    throw std::runtime_error("Calling initialize on an already initialized AsyncSensorModel!");
  }

  // Initialize internal state
  name_ = name;
  interfaces_ = interfaces;

  auto context = interfaces_.get_node_base_interface()->get_context();
  auto executor_options = rclcpp::ExecutorOptions();
  executor_options.context = context;

  if (executor_thread_count_ == 1) {
    executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared(executor_options);
  } else {
    executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared(
      executor_options, executor_thread_count_);
  }

  callback_queue_ = std::make_shared<CallbackAdapter>(context);

  // This callback group MUST be re-entrant in order to support parallelization
  cb_group_ = interfaces_.get_node_base_interface()->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant, false);
  interfaces_.get_node_waitables_interface()->add_waitable(callback_queue_, cb_group_);

  transaction_callback_ = transaction_callback;

  // Call the derived onInit() function to perform implementation-specific initialization
  onInit();

  // Make sure the executor will service the given node
  // We can add this without any guards because the callback group was set to not get automatically
  // added to executors
  executor_->add_callback_group(cb_group_, interfaces_.get_node_base_interface());

  // Start the executor
  spinner_ = std::thread(
    [&]() {
      executor_->spin();
    });

  // Wait for the executor to start spinning.
  // This avoids a race where the destructor blocks waiting for the spinner_
  // thread to be joined when the class is destroyed before the thread is ever
  // scheduled.
  auto callback = std::make_shared<CallbackWrapper<void>>(
    [&]() {
      initialized_ = true;
    });
  auto result = callback->getFuture();
  callback_queue_->addCallback(callback);
  result.wait();
}

void AsyncSensorModel::graphCallback(Graph::ConstSharedPtr graph)
{
  auto callback = std::make_shared<CallbackWrapper<void>>(
    std::bind(&AsyncSensorModel::onGraphUpdate, this, std::move(graph))
  );
  callback_queue_->addCallback(callback);
}

void AsyncSensorModel::sendTransaction(Transaction::SharedPtr transaction)
{
  transaction_callback_(std::move(transaction));
}

void AsyncSensorModel::start()
{
  auto callback = std::make_shared<CallbackWrapper<void>>(
    std::bind(&AsyncSensorModel::onStart, this)
  );
  auto result = callback->getFuture();
  callback_queue_->addCallback(callback);
  result.wait();
}

void AsyncSensorModel::stop()
{
  // Prefer to call onStop in executor's thread so downstream users don't have
  // to worry about threads in ROS callbacks when there's only 1 thread.
  if (interfaces_.get_node_base_interface()->get_context()->is_valid()) {
    auto callback = std::make_shared<CallbackWrapper<void>>(
      std::bind(&AsyncSensorModel::onStop, this)
    );
    auto result = callback->getFuture();
    callback_queue_->addCallback(callback);
    result.wait();
  } else {
    // Can't run in executor's thread because the executor won't service more
    // callbacks after the context is shutdown.
    // Join executor's threads right away.
    internal_stop();
    onStop();
  }
}

void AsyncSensorModel::internal_stop()
{
  if (spinner_.joinable()) {
    executor_->cancel();
    spinner_.join();
  }

  // Reset callback queue
  callback_queue_->removeAllCallbacks();
  initialized_ = false;
}

}  // namespace fuse_core
