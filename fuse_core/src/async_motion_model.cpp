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
#include <fuse_core/async_motion_model.h>

#include <fuse_core/callback_wrapper.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <utility>
#include <string>


namespace fuse_core
{

AsyncMotionModel::AsyncMotionModel(size_t thread_count) :
  name_("uninitialized"),
  executor_(rclcpp::ExecutorOptions(), thread_count)
{
}

bool AsyncMotionModel::apply(Transaction& transaction)
{
  // Insert a call to the motion model's queryCallback() function into the motion model's callback queue. While this
  // makes this particular function more difficult to write, it does simplify the threading model on all derived
  // MotionModel objects, as the queryCallback will run within the same callback queue as subscriptions, timers, etc.
  // Thus, it is functionally similar to a service callback, and should be a familiar pattern for ROS developers.
  // This function blocks until the queryCallback() call completes, thus enforcing that motion models are generated
  // in order.
  auto callback = std::make_shared<CallbackWrapper<bool>>(
    std::bind(&AsyncMotionModel::applyCallback, this, std::ref(transaction)));
  auto result = callback->get_future();
  waitables_interface_->add_waitable(callback, nullptr);
  result.wait();
  waitables_interface_->remove_waitable(callback, nullptr);

  return result.get();
}

void AsyncMotionModel::graphCallback(Graph::ConstSharedPtr graph)
{
  auto callback = std::make_shared<CallbackWrapper<void>>(std::bind(&AsyncMotionModel::onGraphUpdate, this, std::move(graph)));
  auto result = callback->get_future();
  waitables_interface_->add_waitable(callback, nullptr);
  result.wait();
  waitables_interface_->remove_waitable(callback, nullptr);

  // @todo TAM: figure out whether we needed the second argument to addCallback here
  //callback_queue_.addCallback(
  //  boost::make_shared<CallbackWrapper<void>>(std::bind(&AsyncMotionModel::onGraphUpdate, this, std::move(graph))),
  //  reinterpret_cast<uint64_t>(this));
}

void AsyncMotionModel::initialize(const std::string& name)
{
  // Initialize internal state
  name_ = name;
  node_ = rclcpp::Node::make_shared(name);
  waitables_interface_ = node_->get_node_waitables_interface();

  // Call the derived onInit() function to perform implementation-specific initialization
  onInit();

  executor_.add_node(node_);
}

void AsyncMotionModel::start()
{
  auto callback = std::make_shared<CallbackWrapper<void>>(std::bind(&AsyncMotionModel::onStart, this));
  auto result = callback->get_future();
  waitables_interface_->add_waitable(callback, nullptr);
  result.wait();
  waitables_interface_->remove_waitable(callback, nullptr);
}

void AsyncMotionModel::stop()
{
  if (rclcpp::ok())
  {
    auto callback = std::make_shared<CallbackWrapper<void>>(std::bind(&AsyncMotionModel::onStop, this));
    auto result = callback->get_future();
    waitables_interface_->add_waitable(callback, nullptr);
    result.wait();
    waitables_interface_->remove_waitable(callback, nullptr);
  }
  else
  {
    executor_.cancel();
    executor_.remove_node(node_);

    onStop();
  }
}

}  // namespace fuse_core
