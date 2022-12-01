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

#include <functional>
#include <memory>
#include <numeric>
#include <vector>

#include <fuse_core/callback_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

class MyClass
{
public:
  double processData(const std::vector<double> & data)
  {
    return std::accumulate(data.begin(), data.end(), 0.0);
  }

  void processDataInPlace(const std::vector<double> & data, double & output)
  {
    output = std::accumulate(data.begin(), data.end(), 0.0);
  }
};

class TestCallbackWrapper : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestCallbackWrapper, Double)
{
  MyClass my_object;
  std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};

  auto node = rclcpp::Node::make_shared("callback_wrapper_double_test_node");
  auto callback_queue =
    std::make_shared<fuse_core::CallbackAdapter>(node->get_node_base_interface()->get_context());
  node->get_node_waitables_interface()->add_waitable(
    callback_queue, (rclcpp::CallbackGroup::SharedPtr) nullptr);

  auto callback = std::make_shared<fuse_core::CallbackWrapper<double>>(
    std::bind(&MyClass::processData, &my_object, std::ref(data)));
  auto result = callback->getFuture();

  callback_queue->addCallback(callback);
  rclcpp::spin_until_future_complete(node, result);

  // This is technically redundant but this is here to mimic how the callbacks are actually used
  EXPECT_EQ(std::future_status::ready, result.wait_for(std::chrono::seconds(10)));
  EXPECT_EQ(15.0, result.get());
}

TEST_F(TestCallbackWrapper, Void)
{
  MyClass my_object;
  std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
  double output;

  auto node = rclcpp::Node::make_shared("callback_wrapper_void_test_node");
  auto callback_queue =
    std::make_shared<fuse_core::CallbackAdapter>(node->get_node_base_interface()->get_context());
  node->get_node_waitables_interface()->add_waitable(
    callback_queue, (rclcpp::CallbackGroup::SharedPtr) nullptr);

  auto callback = std::make_shared<fuse_core::CallbackWrapper<void>>(
    std::bind(&MyClass::processDataInPlace, &my_object, std::ref(data), std::ref(output)));
  auto result = callback->getFuture();

  callback_queue->addCallback(callback);
  rclcpp::spin_until_future_complete(node, result);

  // This is technically redundant but this is here to mimic how the callbacks are actually used
  EXPECT_EQ(std::future_status::ready, result.wait_for(std::chrono::seconds(10)));
  EXPECT_EQ(15.0, output);
}
