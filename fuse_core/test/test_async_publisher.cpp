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
#include <gtest/gtest.h>

#include <set>

#include <fuse_core/async_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Derived AsyncPublisher used to verify the functions get called when expected
 */
class MyPublisher : public fuse_core::AsyncPublisher
{
public:
  MyPublisher()
  : fuse_core::AsyncPublisher(),
    callback_processed(false),
    initialized(false)
  {
  }

  virtual ~MyPublisher() = default;

  void notifyCallback(
    fuse_core::Transaction::ConstSharedPtr /*transaction*/,
    fuse_core::Graph::ConstSharedPtr /*graph*/)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    callback_processed = true;
  }

  void onInit() override
  {
    initialized = true;
  }

  bool callback_processed;
  bool initialized;
};

class TestAsyncPublisher : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestAsyncPublisher, OnInit)
{
  for (int i = 0; i < 50; i++) {
    auto node = rclcpp::Node::make_shared("test_async_pub_node");
    MyPublisher publisher;
    publisher.initialize(node, "my_publisher_" + std::to_string(i));
    EXPECT_TRUE(publisher.initialized);
  }
}

TEST_F(TestAsyncPublisher, DoubleInit)
{
  auto node = rclcpp::Node::make_shared("test_async_pub_node");
  MyPublisher publisher;
  publisher.initialize(node, "my_publisher");
  EXPECT_TRUE(publisher.initialized);
  EXPECT_THROW(publisher.initialize(node, "test"), std::runtime_error);
}

TEST_F(TestAsyncPublisher, notifyCallback)
{
  auto node = rclcpp::Node::make_shared("test_async_pub_node");
  MyPublisher publisher;
  publisher.initialize(node, "my_publisher");

  // Execute the notify() method in this thread. This should push a call to
  // MyPublisher::notifyCallback() into MyPublisher's callback queue, which will get executed by
  // MyPublisher's async spinner. There is a time delay there. So, this call should return almost
  // immediately, then we have to wait a bit before the "callback_processed" flag gets flipped.
  fuse_core::Transaction::ConstSharedPtr transaction;  // nullptr is ok as we don't actually use it
  fuse_core::Graph::ConstSharedPtr graph;  // nullptr is ok as we don't actually use it
  auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);

  // Test for multiple cycles of notify to be sure
  for (int i = 0; i < 50; i++) {
    publisher.callback_processed = false;
    publisher.notify(transaction, graph);
    EXPECT_FALSE(publisher.callback_processed);

    rclcpp::Time wait_time_elapsed = clock.now() + rclcpp::Duration::from_seconds(10);
    while (!publisher.callback_processed && clock.now() < wait_time_elapsed) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(publisher.callback_processed);
  }
}
