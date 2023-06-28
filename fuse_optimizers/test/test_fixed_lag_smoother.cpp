/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Clearpath Robotics
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
#include <fuse_models/SetPose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <gtest/gtest.h>


TEST(FixedLagIgnition, SetInitialStateToPiOrientation)
{
  // Time should be valid after ros::init() returns in main(). But it doesn't hurt to verify.
  ASSERT_TRUE(ros::Time::waitForValid(ros::WallDuration(1.0)));

  auto node_handle = ros::NodeHandle();
  auto relative_pose_publisher = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/relative_pose", 1);

  // Wait for the optimizer to be ready
  ASSERT_TRUE(ros::service::waitForService("/fixed_lag/set_pose", ros::Duration(1.0)));
  ASSERT_TRUE(ros::service::waitForService("/fixed_lag/reset", ros::Duration(1.0)));

  // Set the initial pose to something far away from zero
  fuse_models::SetPose::Request req;
  req.pose.header.frame_id = "map";
  req.pose.header.stamp = ros::Time(1, 0);
  req.pose.pose.pose.position.x = 100.1;
  req.pose.pose.pose.position.y = 100.2;
  req.pose.pose.pose.position.z = 0.0;
  req.pose.pose.pose.orientation.x = 0.0;
  req.pose.pose.pose.orientation.y = 0.0;
  req.pose.pose.pose.orientation.z = 1.0; // yaw = pi rad
  req.pose.pose.pose.orientation.w = 0.0;
  req.pose.pose.covariance[0] = 1.0;
  req.pose.pose.covariance[7] = 1.0;
  req.pose.pose.covariance[35] = 1.0;
  fuse_models::SetPose::Response res;
  ros::service::call("/fixed_lag/set_pose", req, res);
  ASSERT_TRUE(res.success);

  // Wait for the optimizer to process all queued transactions
  ros::Time result_timeout = ros::Time::now() + ros::Duration(3.0);
  auto odom_msg = nav_msgs::Odometry::ConstPtr();
  while ((!odom_msg || odom_msg->header.stamp != ros::Time(3, 0)) &&
         (ros::Time::now() < result_timeout))
  {
    odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(1.0));
  }
  ASSERT_TRUE(static_cast<bool>(odom_msg));
  ASSERT_EQ(odom_msg->header.stamp, ros::Time(1, 0));

  // If we did our job correctly, the initial variable values should be the same as the service call state, give or
  // take the motion model forward prediction.
  EXPECT_NEAR(100.1, odom_msg->pose.pose.position.x, 0.1);
  EXPECT_NEAR(100.2, odom_msg->pose.pose.position.y, 0.1);
  EXPECT_NEAR(1.0, std::abs(odom_msg->pose.pose.orientation.z), 0.1);  // pi == -pi
  EXPECT_NEAR(0.0, odom_msg->pose.pose.orientation.w, 0.1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "fixed_lag_smoother_test");
  auto spinner = ros::AsyncSpinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
