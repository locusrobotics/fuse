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
#include <ros/ros.h>

#include <gtest/gtest.h>


/**
 * @brief Derived AsyncMotionModel used to verify the functions get called when expected
 */
class MyMotionModel : public fuse_core::AsyncMotionModel
{
public:
  MyMotionModel() :
    fuse_core::AsyncMotionModel(1),
    initialized(false)
  {
  }

  virtual ~MyMotionModel() = default;

  bool applyCallback(fuse_core::Transaction& /*transaction*/)
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(1.0));
    transaction_received = true;
    return true;
  }

  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr /*graph*/) override
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(1.0));
    graph_received = true;
  }

  void onInit() override
  {
    initialized = true;
  }

  bool graph_received;
  bool initialized;
  bool transaction_received;
};

TEST(AsyncMotionModel, OnInit)
{
  MyMotionModel motion_model;
  motion_model.initialize("my_motion_model");
  EXPECT_TRUE(motion_model.initialized);
}

TEST(AsyncMotionModel, OnGraphUpdate)
{
  MyMotionModel motion_model;
  motion_model.initialize("my_motion_model");

  // Execute the graph callback in this thread. This should push a call to MyMotionModel::onGraphUpdate()
  // into MyMotionModel's callback queue, which will get executed by MyMotionModel's async spinner.
  // There is a time delay there. So, this call should return almost immediately, then we have to wait
  // a bit before the "graph_received" flag gets flipped.
  fuse_core::Graph::ConstSharedPtr graph;  // nullptr...which is fine because we do not actually use it
  motion_model.graphCallback(graph);
  EXPECT_FALSE(motion_model.graph_received);
  rclcpp::Time wait_time_elapsed = rclcpp::Clock(RCL_SYSTEM_TIME).now() + rclcpp::Duration::from_seconds(10.0);
  while (!motion_model.graph_received && rclcpp::Clock(RCL_SYSTEM_TIME).now() < wait_time_elapsed)
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.1));
  }
  EXPECT_TRUE(motion_model.graph_received);
}

TEST(AsyncMotionModel, ApplyCallback)
{
  MyMotionModel motion_model;
  motion_model.initialize("my_motion_model");

  // Call the motion model base class "apply()" method to send a transaction to the derived model. The AsyncMotionModel
  // will then inject a call to applyCallback() into the motion model's callback queue. There is a time delay there, so
  // this call should block for *at least* 1.0 second. Once it returns, the "received_transaction" flag should be set.
  fuse_core::Transaction transaction;
  rclcpp::Time before_apply = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  motion_model.apply(transaction);
  rclcpp::Time after_apply = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  EXPECT_TRUE(motion_model.transaction_received);
  EXPECT_LE(rclcpp::Duration::from_seconds(1.0), after_apply - before_apply);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_async_motion_model");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
