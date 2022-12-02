/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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


TEST(FixedLagIgnition, SetInitialState)
{
  // TODO(CH3): Make this an rclcpp node
  auto node_handle = ros::NodeHandle();
  auto relative_pose_publisher = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/relative_pose", 1);

  // Time should be valid after rclcpp::init() returns in main(). But it doesn't hurt to verify.
  ASSERT_TRUE(fuse_core::wait_for_valid(node->get_clock(), rclcpp::Duration::from_seconds(1.0)));

  // Wait for the optimizer to be ready
  ASSERT_TRUE(ros::service::waitForService("/fixed_lag/set_pose", rclcpp::Duration::from_seconds(1.0)));
  ASSERT_TRUE(ros::service::waitForService("/fixed_lag/reset", rclcpp::Duration::from_seconds(1.0)));

  // Set the initial pose to something far away from zero
  fuse_models::SetPose::Request req;
  req.pose.header.frame_id = "map";
  req.pose.header.stamp = rclcpp::Time(1, 0);
  req.pose.pose.pose.position.x = 100.1;
  req.pose.pose.pose.position.y = 100.2;
  req.pose.pose.pose.position.z = 0.0;
  req.pose.pose.pose.orientation.x = 0.0;
  req.pose.pose.pose.orientation.y = 0.0;
  req.pose.pose.pose.orientation.z = 0.8660;
  req.pose.pose.pose.orientation.w = 0.5000;
  req.pose.pose.covariance[0] = 1.0;
  req.pose.pose.covariance[7] = 1.0;
  req.pose.pose.covariance[35] = 1.0;
  fuse_models::SetPose::Response res;
  ros::service::call("/fixed_lag/set_pose", req, res);
  ASSERT_TRUE(res.success);

  // The 'set_pose' service call triggers all of the sensors to resubscribe to their topics.
  // I need to wait for those subscribers to be ready before sending them sensor data.
  rclcpp::Time subscriber_timeout = this->node->now() + rclcpp::Duration::from_seconds(1.0);
  while ((relative_pose_publisher.getNumSubscribers() < 1u) &&
         (this->node->now() < subscriber_timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.01);
  }
  ASSERT_GE(relative_pose_publisher.getNumSubscribers(), 1u);

  // Publish a relative pose
  auto pose_msg1 = geometry_msgs::PoseWithCovarianceStamped();
  pose_msg1.header.stamp = rclcpp::Time(2, 0);
  pose_msg1.header.frame_id = "base_link";
  pose_msg1.pose.pose.position.x = 5.0;
  pose_msg1.pose.pose.position.y = 6.0;
  pose_msg1.pose.pose.position.z = 0.0;
  pose_msg1.pose.pose.orientation.x = 0.0;
  pose_msg1.pose.pose.orientation.y = 0.0;
  pose_msg1.pose.pose.orientation.z = 0.900;
  pose_msg1.pose.pose.orientation.w = 0.436;
  pose_msg1.pose.covariance[0] = 1.0;
  pose_msg1.pose.covariance[7] = 1.0;
  pose_msg1.pose.covariance[35] = 1.0;
  relative_pose_publisher.publish(pose_msg1);

  auto pose_msg2 = geometry_msgs::PoseWithCovarianceStamped();
  pose_msg2.header.stamp = rclcpp::Time(3, 0);
  pose_msg2.header.frame_id = "base_link";
  pose_msg2.pose.pose.position.x = 10.0;
  pose_msg2.pose.pose.position.y = 20.0;
  pose_msg2.pose.pose.position.z = 0.0;
  pose_msg2.pose.pose.orientation.x = 0.0;
  pose_msg2.pose.pose.orientation.y = 0.0;
  pose_msg2.pose.pose.orientation.z = 0.5000;
  pose_msg2.pose.pose.orientation.w = 0.8660;
  pose_msg2.pose.covariance[0] = 1.0;
  pose_msg2.pose.covariance[7] = 1.0;
  pose_msg2.pose.covariance[35] = 1.0;
  relative_pose_publisher.publish(pose_msg2);

  // Wait for the optimizer to process all queued transactions
  rclcpp::Time result_timeout = node->now() + rclcpp::Duration::from_seconds(3.0);
  auto odom_msg = nav_msgs::Odometry::ConstPtr();
  while ((!odom_msg || odom_msg->header.stamp != rclcpp::Time(3, 0)) &&
         (node->now() < result_timeout))
  {
    odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", rclcpp::Duration::from_seconds(1.0));
  }
  ASSERT_TRUE(static_cast<bool>(odom_msg));
  ASSERT_EQ(odom_msg->header.stamp, rclcpp::Time(3, 0));

  // The optimizer is configured for 0 iterations, so it should return the initial variable values
  // If we did our job correctly, the initial variable values should be the same as the service call state, give or
  // take the motion model forward prediction.
  EXPECT_NEAR(100.1, odom_msg->pose.pose.position.x, 0.10);
  EXPECT_NEAR(100.2, odom_msg->pose.pose.position.y, 0.10);
  EXPECT_NEAR(0.8660, odom_msg->pose.pose.orientation.z, 0.10);
  EXPECT_NEAR(0.5000, odom_msg->pose.pose.orientation.w, 0.10);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "fixed_lag_ignition_test");
  auto spinner = ros::AsyncSpinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
