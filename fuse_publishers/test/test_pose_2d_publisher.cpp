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
#include <fuse_constraints/absolute_pose_2d_stamped_constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_graphs/hash_graph.hpp>
#include <fuse_publishers/pose_2d_publisher.h>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <gtest/gtest.h>

#include <vector>


/**
 * @brief Test fixture for the LatestStampedPose2DPublisher
 *
 * This test fixture provides a populated graph for testing the publish() function, and subscriber callbacks
 * for each of the different possible output topics.
 */
class Pose2DPublisherTestFixture : public ::testing::Test
{
public:
  Pose2DPublisherTestFixture() :
    private_node_handle_("~"),
    graph_(fuse_graphs::HashGraph::make_shared()),
    transaction_(fuse_core::Transaction::make_shared()),
    received_pose_msg_(false),
    received_pose_with_covariance_msg_(false),
    received_tf_msg_(false)
  {
    // Add a few pose variables
    auto position1 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1234, 10));
    position1->x() = 1.01;
    position1->y() = 2.01;
    auto orientation1 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1234, 10));
    orientation1->yaw() = 3.01;
    auto position2 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 10));
    position2->x() = 1.02;
    position2->y() = 2.02;
    auto orientation2 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 10));
    orientation2->yaw() = 3.02;
    auto position3 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 9));
    position3->x() = 1.03;
    position3->y() = 2.03;
    auto orientation3 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 9));
    orientation3->yaw() = 3.03;
    auto position4 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 11),
                                                                    fuse_core::uuid::generate("kitt"));
    position4->x() = 1.04;
    position4->y() = 2.04;
    auto orientation4 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 11),
                                                                          fuse_core::uuid::generate("kitt"));
    orientation4->yaw() = 3.04;

    transaction_->addInvolvedStamp(position1->stamp());
    transaction_->addInvolvedStamp(orientation1->stamp());
    transaction_->addInvolvedStamp(position2->stamp());
    transaction_->addInvolvedStamp(orientation2->stamp());
    transaction_->addInvolvedStamp(position3->stamp());
    transaction_->addInvolvedStamp(orientation3->stamp());
    transaction_->addInvolvedStamp(position4->stamp());
    transaction_->addInvolvedStamp(orientation4->stamp());
    transaction_->addVariable(position1);
    transaction_->addVariable(orientation1);
    transaction_->addVariable(position2);
    transaction_->addVariable(orientation2);
    transaction_->addVariable(position3);
    transaction_->addVariable(orientation3);
    transaction_->addVariable(position4);
    transaction_->addVariable(orientation4);
    // Add some priors on the variables some we can optimize the graph
    fuse_core::Vector3d mean1;
    mean1 << 1.01, 2.01, 3.01;
    fuse_core::Matrix3d cov1;
    cov1 << 1.01, 0.0, 0.0,  0.0, 2.01, 0.0,  0.0, 0.0, 3.01;
    auto constraint1 = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
      "test", *position1, *orientation1, mean1, cov1);
    fuse_core::Vector3d mean2;
    mean2 << 1.02, 2.02, 3.02;
    fuse_core::Matrix3d cov2;
    cov2 << 1.02, 0.0, 0.0,  0.0, 2.02, 0.0,  0.0, 0.0, 3.02;
    auto constraint2 = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
      "test", *position2, *orientation2, mean2, cov2);
    fuse_core::Vector3d mean3;
    mean3 << 1.03, 2.03, 3.03;
    fuse_core::Matrix3d cov3;
    cov3 << 1.03, 0.0, 0.0,  0.0, 2.03, 0.0,  0.0, 0.0, 3.03;
    auto constraint3 = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
      "test", *position3, *orientation3, mean3, cov3);
    fuse_core::Vector3d mean4;
    mean4 << 1.04, 2.04, 3.04;
    fuse_core::Matrix3d cov4;
    cov4 << 1.04, 0.0, 0.0,  0.0, 2.04, 0.0,  0.0, 0.0, 3.04;
    auto constraint4 = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
      "test", *position4, *orientation4, mean4, cov4);
    transaction_->addConstraint(constraint1);
    transaction_->addConstraint(constraint2);
    transaction_->addConstraint(constraint3);
    transaction_->addConstraint(constraint4);
    // Add the transaction to the graph
    graph_->update(*transaction_);
    // Optimize the graph
    graph_->optimize();

    // Publish a odom->base transform so tf lookups will succeed
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = rclcpp::Time(0, 0);
    odom_to_base.header.frame_id = "test_odom";
    odom_to_base.child_frame_id = "test_base";
    odom_to_base.transform.translation.x = -0.10;
    odom_to_base.transform.translation.y = -0.20;
    odom_to_base.transform.translation.z = -0.30;
    odom_to_base.transform.rotation.x = 0.0;
    odom_to_base.transform.rotation.y = 0.0;
    odom_to_base.transform.rotation.z = -0.1986693307950612164;
    odom_to_base.transform.rotation.w = 0.98006657784124162625;  // -0.4rad in yaw
    static_broadcaster_.sendTransform(odom_to_base);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped& msg)
  {
    received_pose_msg_ = true;
    pose_msg_ = msg;
  }

  void poseWithCovarianceCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
  {
    received_pose_with_covariance_msg_ = true;
    pose_with_covariance_msg_ = msg;
  }

  void tfCallback(const tf2_msgs::msg::TFMessage& msg)
  {
    received_tf_msg_ = true;
    tf_msg_ = msg;
  }

protected:
  // TODO(CH3): Replace with node_ (also, we don't need to support node interfaces here since it's a test...)
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  fuse_graphs::HashGraph::SharedPtr graph_;
  fuse_core::Transaction::SharedPtr transaction_;
  bool received_pose_msg_;
  geometry_msgs::msg::PoseStamped pose_msg_;
  bool received_pose_with_covariance_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_msg_;
  bool received_tf_msg_;
  tf2_msgs::msg::TFMessage tf_msg_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};

TEST_F(Pose2DPublisherTestFixture, PublishPose)
{
  // Test that the expected PoseStamped message is published

  // Create a publisher and send it the graph
  private_node_handle_.setParam("test_publisher/map_frame", "test_map");
  private_node_handle_.setParam("test_publisher/odom_frame", "test_odom");
  private_node_handle_.setParam("test_publisher/base_frame", "test_base");
  private_node_handle_.setParam("test_publisher/publish_to_tf", false);
  fuse_publishers::Pose2DPublisher publisher;
  publisher.initialize("test_publisher");
  publisher.start();

  // Subscribe to the "pose" topic
  ros::Subscriber subscriber = private_node_handle_.subscribe(
    "test_publisher/pose",
    1,
    &Pose2DPublisherTestFixture::poseCallback,
    reinterpret_cast<Pose2DPublisherTestFixture*>(this));

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node_->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_pose_msg_) && (node_->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10));
  }

  ASSERT_TRUE(received_pose_msg_);
  EXPECT_EQ(rclcpp::Time(1235, 10), pose_msg_.header.stamp);
  EXPECT_EQ("test_map", pose_msg_.header.frame_id);
  EXPECT_NEAR(1.02, pose_msg_.pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.02, pose_msg_.pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, pose_msg_.pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.02, tf2::getYaw(pose_msg_.pose.orientation), 1.0e-9);
}

TEST_F(Pose2DPublisherTestFixture, PublishPoseWithCovariance)
{
  // Test that the expected PoseWithCovarianceStamped message is published

  // Create a publisher and send it the graph
  private_node_handle_.setParam("test_publisher/map_frame", "test_map");
  private_node_handle_.setParam("test_publisher/odom_frame", "test_odom");
  private_node_handle_.setParam("test_publisher/base_frame", "test_base");
  private_node_handle_.setParam("test_publisher/publish_to_tf", false);
  fuse_publishers::Pose2DPublisher publisher;
  publisher.initialize("test_publisher");
  publisher.start();

  // Subscribe to the "pose_with_covariance" topic
  ros::Subscriber subscriber = private_node_handle_.subscribe(
    "test_publisher/pose_with_covariance",
    1,
    &Pose2DPublisherTestFixture::poseWithCovarianceCallback,
    reinterpret_cast<Pose2DPublisherTestFixture*>(this));

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node_->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_pose_with_covariance_msg_) && (node_->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10));
  }

  ASSERT_TRUE(received_pose_with_covariance_msg_);
  EXPECT_EQ(rclcpp::Time(1235, 10), pose_with_covariance_msg_.header.stamp);
  EXPECT_EQ("test_map", pose_with_covariance_msg_.header.frame_id);
  EXPECT_NEAR(1.02, pose_with_covariance_msg_.pose.pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.02, pose_with_covariance_msg_.pose.pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, pose_with_covariance_msg_.pose.pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.02, tf2::getYaw(pose_with_covariance_msg_.pose.pose.orientation), 1.0e-9);
  std::vector<double> expected_covariance =
  {
    1.02, 0.00, 0.00, 0.00, 0.00, 0.00,
    0.00, 2.02, 0.00, 0.00, 0.00, 0.00,
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    0.00, 0.00, 0.00, 0.00, 0.00, 3.02
  };
  for (size_t i = 0; i < 36; ++i)
  {
    EXPECT_NEAR(expected_covariance[i], pose_with_covariance_msg_.pose.covariance[i], 1.0e-9);
  }
}

TEST_F(Pose2DPublisherTestFixture, PublishTfWithoutOdom)
{
  // Test that the expected TFMessage is published

  // Create a publisher and send it the graph
  private_node_handle_.setParam("test_publisher/map_frame", "test_map");
  private_node_handle_.setParam("test_publisher/odom_frame", "test_base");
  private_node_handle_.setParam("test_publisher/base_frame", "test_base");
  private_node_handle_.setParam("test_publisher/publish_to_tf", true);
  fuse_publishers::Pose2DPublisher publisher;
  publisher.initialize("test_publisher");
  publisher.start();

  // Subscribe to the "pose" topic
  ros::Subscriber subscriber = private_node_handle_.subscribe(
    "/tf",
    1,
    &Pose2DPublisherTestFixture::tfCallback,
    reinterpret_cast<Pose2DPublisherTestFixture*>(this));

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node_->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_tf_msg_) && (node_->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10));
  }

  ASSERT_TRUE(received_tf_msg_);
  ASSERT_EQ(1ul, tf_msg_.transforms.size());
  EXPECT_EQ("test_map", tf_msg_.transforms[0].header.frame_id);
  EXPECT_EQ("test_base", tf_msg_.transforms[0].child_frame_id);
  EXPECT_NEAR(1.02, tf_msg_.transforms[0].transform.translation.x, 1.0e-9);
  EXPECT_NEAR(2.02, tf_msg_.transforms[0].transform.translation.y, 1.0e-9);
  EXPECT_NEAR(0.00, tf_msg_.transforms[0].transform.translation.z, 1.0e-9);
  EXPECT_NEAR(3.02, tf2::getYaw(tf_msg_.transforms[0].transform.rotation), 1.0e-9);
}

TEST_F(Pose2DPublisherTestFixture, PublishTfWithOdom)
{
  // Test that the expected TFMessage is published

  // Create a publisher and send it the graph
  private_node_handle_.setParam("test_publisher/map_frame", "test_map");
  private_node_handle_.setParam("test_publisher/odom_frame", "test_odom");
  private_node_handle_.setParam("test_publisher/base_frame", "test_base");
  private_node_handle_.setParam("test_publisher/publish_to_tf", true);
  fuse_publishers::Pose2DPublisher publisher;
  publisher.initialize("test_publisher");
  publisher.start();

  // Subscribe to the "pose" topic
  ros::Subscriber subscriber = private_node_handle_.subscribe(
    "/tf",
    1,
    &Pose2DPublisherTestFixture::tfCallback,
    reinterpret_cast<Pose2DPublisherTestFixture*>(this));

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node_->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_tf_msg_) && (node_->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10));
  }

  ASSERT_TRUE(received_tf_msg_);
  ASSERT_EQ(1ul, tf_msg_.transforms.size());
  EXPECT_EQ("test_map", tf_msg_.transforms[0].header.frame_id);
  EXPECT_EQ("test_odom", tf_msg_.transforms[0].child_frame_id);
  EXPECT_NEAR(0.9788154983, tf_msg_.transforms[0].transform.translation.x, 1.0e-9);
  EXPECT_NEAR(1.8002186614, tf_msg_.transforms[0].transform.translation.y, 1.0e-9);
  EXPECT_NEAR(0.3000000000, tf_msg_.transforms[0].transform.translation.z, 1.0e-9);
  EXPECT_NEAR(-2.8631853072, tf2::getYaw(tf_msg_.transforms[0].transform.rotation), 1.0e-9);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_pose_2d_publisher");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
