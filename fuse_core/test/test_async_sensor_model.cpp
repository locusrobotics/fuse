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

#include <fuse_core/async_sensor_model.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Flag used to track the execution of the transaction callback
 */
static bool received_transaction = false;

/**
 * @brief Transaction callback function used to verify the function gets called in the background
 */
void transactionCallback(fuse_core::Transaction::SharedPtr /*transaction*/)
{
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  received_transaction = true;
}

/**
 * @brief Derived AsyncSensorModel used to verify the functions get called when expected
 */
class MySensor : public fuse_core::AsyncSensorModel
{
public:
  MySensor()
  : fuse_core::AsyncSensorModel(0),
    initialized(false)
  {
  }

  virtual ~MySensor() = default;

  void onInit() override
  {
    initialized = true;
  }

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr /*graph*/) override
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    graph_received = true;
  }

  bool graph_received = false;
  bool initialized = false;
};

class TestAsyncSensorModel : public ::testing::Test
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

TEST_F(TestAsyncSensorModel, OnInit)
{
  for (int i = 0; i < 50; i++) {
    MySensor sensor;
    sensor.initialize("my_sensor_" + std::to_string(i), &transactionCallback);
    EXPECT_TRUE(sensor.initialized);
  }
}

TEST_F(TestAsyncSensorModel, DoubleInit)
{
  MySensor sensor_model;
  sensor_model.initialize("my_sensor_model", &transactionCallback);
  EXPECT_TRUE(sensor_model.initialized);
  EXPECT_THROW(sensor_model.initialize("test", &transactionCallback), std::runtime_error);
}

TEST_F(TestAsyncSensorModel, OnGraphUpdate)
{
  MySensor sensor;
  sensor.initialize("my_sensor", &transactionCallback);

  // Execute the graph callback in this thread. This should push a call to MySensor::onGraphUpdate()
  // into MySensor's callback queue, which will get executed by MySensor's async spinner.
  // There is a time delay there. So, this call should return almost immediately, then we have to
  // wait a bit before the "received_graph" flag gets flipped.
  fuse_core::Graph::ConstSharedPtr graph;  // nullptr is ok as we don't actually use it
  auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);

  // Test for multiple cycles of graphCallback to be sure
  for (int i = 0; i < 50; i++) {
    sensor.graph_received = false;
    sensor.graphCallback(graph);
    EXPECT_FALSE(sensor.graph_received);
    rclcpp::Time wait_time_elapsed = clock.now() + rclcpp::Duration::from_seconds(10);
    while (!sensor.graph_received && clock.now() < wait_time_elapsed) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(sensor.graph_received);
  }
}

TEST_F(TestAsyncSensorModel, SendTransaction)
{
  MySensor sensor;
  sensor.initialize("my_sensor", &transactionCallback);

  // Use the sensor "sendTransaction()" method to execute the transaction callback. This will get
  // executed immediately.
  fuse_core::Transaction::SharedPtr transaction;  // nullptr is ok as we don't actually use it
  sensor.sendTransaction(transaction);
  EXPECT_TRUE(received_transaction);
}
