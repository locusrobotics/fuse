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

#include <fuse_core/async_motion_model.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Derived AsyncMotionModel used to verify the functions get called when expected
 */
class MyMotionModel : public fuse_core::AsyncMotionModel
{
public:
  MyMotionModel()
  : fuse_core::AsyncMotionModel(1),
    initialized(false)
  {
  }

  virtual ~MyMotionModel() = default;

  bool applyCallback(fuse_core::Transaction & /*transaction*/)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    transaction_received = true;
    return true;
  }

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr /*graph*/) override
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    graph_received = true;
  }

  void onInit() override
  {
    initialized = true;
  }

  bool graph_received = false;
  bool initialized = false;
  bool transaction_received = false;
};

class TestAsyncMotionModel : public ::testing::Test
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

TEST_F(TestAsyncMotionModel, OnInit)
{
  for (int i = 0; i < 50; i++) {
    MyMotionModel motion_model;
    auto node = rclcpp::Node::make_shared("test_async_motion_model_node");
    motion_model.initialize(*node, "my_motion_model_" + std::to_string(i));
    EXPECT_TRUE(motion_model.initialized);
  }
}

TEST_F(TestAsyncMotionModel, DoubleInit)
{
  MyMotionModel motion_model;
  auto node = rclcpp::Node::make_shared("test_async_motion_model_node");
  motion_model.initialize(*node, "my_motion_model");
  EXPECT_TRUE(motion_model.initialized);
  EXPECT_THROW(motion_model.initialize(*node, "test"), std::runtime_error);
}

TEST_F(TestAsyncMotionModel, OnGraphUpdate)
{
  MyMotionModel motion_model;
  auto node = rclcpp::Node::make_shared("test_async_motion_model_node");
  motion_model.initialize(*node, "my_motion_model");

  // Execute the graph callback in this thread. This should push a call to
  // MyMotionModel::onGraphUpdate() into MyMotionModel's callback queue, which will get executed by
  // MyMotionModel's async spinner. There is a time delay there. So, this call should return almost
  // immediately, then we have to wait a bit before the "graph_received" flag gets flipped.
  fuse_core::Graph::ConstSharedPtr graph;  // nullptr is ok as we don't actually use it
  auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);

  // Test for multiple cycles of graphCallback to be sure
  for (int i = 0; i < 50; i++) {
    motion_model.graph_received = false;
    motion_model.graphCallback(graph);
    EXPECT_FALSE(motion_model.graph_received);

    rclcpp::Time wait_time_elapsed = clock.now() + rclcpp::Duration::from_seconds(10);
    while (!motion_model.graph_received && clock.now() < wait_time_elapsed) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(motion_model.graph_received);
  }
}

TEST_F(TestAsyncMotionModel, ApplyCallback)
{
  MyMotionModel motion_model;
  auto node = rclcpp::Node::make_shared("test_async_motion_model_node");
  motion_model.initialize(*node, "my_motion_model");

  // Call the motion model base class "apply()" method to send a transaction to the derived model.
  // The AsyncMotionModel will then inject a call to applyCallback() into the motion model's
  // callback queue. There is a time delay there, so this call should block for *at least* 1.0
  // second. Once it returns, the "received_transaction" flag should be set.
  fuse_core::Transaction transaction;
  auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);
  rclcpp::Time before_apply = clock.now();
  motion_model.apply(transaction);
  rclcpp::Time after_apply = clock.now();
  EXPECT_TRUE(motion_model.transaction_received);
  EXPECT_LE(rclcpp::Duration::from_seconds(1.0), after_apply - before_apply);
}
