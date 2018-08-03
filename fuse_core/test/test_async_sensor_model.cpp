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
#include <fuse_core/async_sensor_model.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <set>


/**
 * @brief Flag used to track the execution of the transaction callback
 */
static bool received_transaction = false;

/**
 * @brief Transaction callback function used to verify the function gets called in the background
 */
void transactionCallback(const std::set<ros::Time>& stamps, const fuse_core::Transaction::SharedPtr& transaction)
{
  ros::Duration(1.0).sleep();
  received_transaction = true;
}

/**
 * @brief Derived AsyncSensorModel used to verify the functions get called when expected
 */
class MySensor : public fuse_core::AsyncSensorModel
{
public:
  MySensor() :
    fuse_core::AsyncSensorModel(1),
    initialized(false)
  {
  }

  virtual ~MySensor() = default;

  void onInit() override
  {
    initialized = true;
  }

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) override
  {
    ros::Duration(1.0).sleep();
    graph_received = true;
  }

  bool graph_received;
  bool initialized;
};

TEST(AsyncSensorModel, OnInit)
{
  MySensor sensor;
  sensor.initialize("my_sensor", &transactionCallback, ros::getGlobalCallbackQueue());
  EXPECT_TRUE(sensor.initialized);
}

TEST(AsyncSensorModel, OnGraphUpdate)
{
  MySensor sensor;
  sensor.initialize("my_sensor", &transactionCallback, ros::getGlobalCallbackQueue());

  // Execute the graph callback in this thread. This should push a call to MySensor::onGraphUpdate()
  // into MySensor's callback queue, which will get executed by MySensor's async spinner.
  // There is a time delay there. So, this call should return almost immediately, then we have to wait
  // a bit before the "received_graph" flag gets flipped.
  fuse_core::Graph::ConstSharedPtr graph;  // nullptr...which is fine because we do not actually use it
  sensor.graphCallback(graph);
  EXPECT_FALSE(sensor.graph_received);
  ros::Time wait_time_elapsed = ros::Time::now() + ros::Duration(10.0);
  while (!sensor.graph_received && ros::Time::now() < wait_time_elapsed)
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_TRUE(sensor.graph_received);
}

TEST(AsyncSensorModel, InjectCallback)
{
  MySensor sensor;
  sensor.initialize("my_sensor", &transactionCallback, ros::getGlobalCallbackQueue());

  // Use the sensor to inject a callback into the global callback queue. This will get executed by main's spinner.
  // There is a time delay there. So, this call should return almost immediately, then we have to wait
  // a bit before the "received_transaction" flag gets flipped.
  std::set<ros::Time> stamps;
  fuse_core::Transaction::SharedPtr transaction;  // nullptr, okay because we don't actually use it for anything
  sensor.injectCallback(stamps, transaction);
  EXPECT_FALSE(received_transaction);
  ros::Time wait_time_elapsed = ros::Time::now() + ros::Duration(10.0);
  while (!received_transaction && ros::Time::now() < wait_time_elapsed)
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_TRUE(received_transaction);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_async_sensor_model");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
