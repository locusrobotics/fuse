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
#include <rclcpp/rclcpp.hpp>
// Workaround ros2/geometry2#242
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // NOLINT(build/include_order)
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
    graph_(fuse_graphs::HashGraph::make_shared()),
    transaction_(fuse_core::Transaction::make_shared()),
    received_pose_msg_(false),
    received_pose_with_covariance_msg_(false),
    received_tf_msg_(false)
  {
    // Add a few pose variables
    auto position1 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1234, 10, RCL_ROS_TIME));
    position1->x() = 1.01;
    position1->y() = 2.01;
    auto orientation1 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1234, 10, RCL_ROS_TIME));
    orientation1->yaw() = 3.01;
    auto position2 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 10, RCL_ROS_TIME));
    position2->x() = 1.02;
    position2->y() = 2.02;
    auto orientation2 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 10, RCL_ROS_TIME));
    orientation2->yaw() = 3.02;
    auto position3 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 9, RCL_ROS_TIME));
    position3->x() = 1.03;
    position3->y() = 2.03;
    auto orientation3 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 9, RCL_ROS_TIME));
    orientation3->yaw() = 3.03;
    auto position4 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1235, 11, RCL_ROS_TIME),
                                                                    fuse_core::uuid::generate("kitt"));
    position4->x() = 1.04;
    position4->y() = 2.04;
    auto orientation4 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1235, 11, RCL_ROS_TIME),
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
    odom_to_base_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    odom_to_base_.header.frame_id = "test_odom";
    odom_to_base_.child_frame_id = "test_base";
    odom_to_base_.transform.translation.x = -0.10;
    odom_to_base_.transform.translation.y = -0.20;
    odom_to_base_.transform.translation.z = -0.30;
    odom_to_base_.transform.rotation.x = 0.0;
    odom_to_base_.transform.rotation.y = 0.0;
    odom_to_base_.transform.rotation.z = -0.1986693307950612164;
    odom_to_base_.transform.rotation.w = 0.98006657784124162625;  // -0.4rad in yaw
  }

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    spinner_ = std::thread(
      [&]() {
        executor_->spin();
      });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {
     spinner_.join();
    }
    executor_.reset();
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

   std::thread spinner_;  //!< Internal thread for spinning the executor
   rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

protected:
  geometry_msgs::msg::TransformStamped odom_to_base_;
  fuse_graphs::HashGraph::SharedPtr graph_;
  fuse_core::Transaction::SharedPtr transaction_;
  bool received_pose_msg_;
  geometry_msgs::msg::PoseStamped pose_msg_;
  bool received_pose_with_covariance_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_msg_;
  bool received_tf_msg_;
  tf2_msgs::msg::TFMessage tf_msg_;
};

TEST_F(Pose2DPublisherTestFixture, PublishPose)
{
  // Test that the expected PoseStamped message is published
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "-p", "map_frame:=test_map",
    "-p", "odom_frame:=test_odom",
    "-p", "base_frame:=test_base",
    "-p", "publish_to_tf:=false"});
  auto node = rclcpp::Node::make_shared("test_pose_2d_publisher_node", options);
  executor_->add_node(node);

  // Create a publisher and send it the graph
  fuse_publishers::Pose2DPublisher publisher;
  publisher.initialize(node, "test_publisher");
  publisher.start();

  // Subscribe to the "pose" topic
  auto subscriber1 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/test_publisher/pose", 1,
    std::bind(&Pose2DPublisherTestFixture::poseCallback, this, std::placeholders::_1));

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_pose_msg_) && (node->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10).to_chrono<std::chrono::nanoseconds>());
  }

  ASSERT_TRUE(received_pose_msg_);
  EXPECT_EQ(rclcpp::Time(1235, 10, RCL_ROS_TIME), pose_msg_.header.stamp);
  EXPECT_EQ("test_map", pose_msg_.header.frame_id);
  EXPECT_NEAR(1.02, pose_msg_.pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.02, pose_msg_.pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, pose_msg_.pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.02, tf2::getYaw(pose_msg_.pose.orientation), 1.0e-9);
}

TEST_F(Pose2DPublisherTestFixture, PublishPoseWithCovariance)
{
  // Test that the expected PoseWithCovarianceStamped message is published
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "-p", "map_frame:=test_map",
    "-p", "odom_frame:=test_odom",
    "-p", "base_frame:=test_base",
    "-p", "publish_to_tf:=false"});
  auto node = rclcpp::Node::make_shared("test_pose_2d_publisher_node", options);
  executor_->add_node(node);

  // Create a publisher and send it the graph
  fuse_publishers::Pose2DPublisher publisher;
  publisher.initialize(node, "test_publisher");
  publisher.start();

  // Subscribe to the "pose_with_covariance" topic
  auto subscriber1 = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/test_publisher/pose_with_covariance", 1,
    std::bind(&Pose2DPublisherTestFixture::poseWithCovarianceCallback, this, std::placeholders::_1));

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_pose_with_covariance_msg_) && (node->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10).to_chrono<std::chrono::nanoseconds>());
  }

  ASSERT_TRUE(received_pose_with_covariance_msg_);
  EXPECT_EQ(rclcpp::Time(1235, 10, RCL_ROS_TIME), pose_with_covariance_msg_.header.stamp);
  EXPECT_EQ("test_map", pose_with_covariance_msg_.header.frame_id);
  EXPECT_NEAR(1.02, pose_with_covariance_msg_.pose.pose.position.x, 1.0e-9);
  EXPECT_NEAR(2.02, pose_with_covariance_msg_.pose.pose.position.y, 1.0e-9);
  EXPECT_NEAR(0.00, pose_with_covariance_msg_.pose.pose.position.z, 1.0e-9);
  EXPECT_NEAR(3.02, tf2::getYaw(pose_with_covariance_msg_.pose.pose.orientation), 1.0e-9);
  std::vector<double> expected_covariance =
  {
    /* *INDENT-OFF* */
    1.02, 0.00, 0.00, 0.00, 0.00, 0.00,
    0.00, 2.02, 0.00, 0.00, 0.00, 0.00,
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    0.00, 0.00, 0.00, 0.00, 0.00, 3.02
    /* *INDENT-ON* */
  };
  for (size_t i = 0; i < 36; ++i)
  {
    EXPECT_NEAR(expected_covariance[i], pose_with_covariance_msg_.pose.covariance[i], 1.0e-9);
  }
}

TEST_F(Pose2DPublisherTestFixture, PublishTfWithoutOdom)
{
  // Test that the expected TFMessage is published
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "-p", "map_frame:=test_map",
    "-p", "odom_frame:=test_base",
    "-p", "base_frame:=test_base",
    "-p", "publish_to_tf:=true"});
  auto node = rclcpp::Node::make_shared("test_pose_2d_publisher_node", options);
  executor_->add_node(node);

  auto tf_node_ = rclcpp::Node::make_shared("tf_pub_node");
  executor_->add_node(tf_node_);
  auto static_broadcaster_ = tf2_ros::StaticTransformBroadcaster(tf_node_);
  static_broadcaster_.sendTransform(odom_to_base_);

  // Subscribe to the "pose" topic
  auto subscriber1 = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 1,
    std::bind(&Pose2DPublisherTestFixture::tfCallback, this, std::placeholders::_1));

  // Create a publisher and send it the graph
  fuse_publishers::Pose2DPublisher publisher;
  publisher.initialize(node, "test_publisher");
  publisher.start();

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_tf_msg_) && (node->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10).to_chrono<std::chrono::nanoseconds>());
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
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "-p", "map_frame:=test_map",
    "-p", "odom_frame:=test_odom",
    "-p", "base_frame:=test_base",
    "-p", "publish_to_tf:=true"});
  auto node = rclcpp::Node::make_shared("test_pose_2d_publisher_node", options);
  executor_->add_node(node);

  // Subscribe to the "pose" topic
  auto subscriber1 = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 1,
    std::bind(&Pose2DPublisherTestFixture::tfCallback, this, std::placeholders::_1));

  auto tf_node_ = rclcpp::Node::make_shared("tf_pub_node");
  executor_->add_node(tf_node_);
  auto static_broadcaster_ = tf2_ros::StaticTransformBroadcaster(tf_node_);
  static_broadcaster_.sendTransform(odom_to_base_);

  // Create a publisher and send it the graph
  fuse_publishers::Pose2DPublisher publisher;
  publisher.initialize(node, "test_publisher");
  publisher.start();

  // Send the graph to the Publisher to trigger message publishing
  publisher.notify(transaction_, graph_);

  // Verify the subscriber received the expected pose
  rclcpp::Time timeout = node->now() + rclcpp::Duration::from_seconds(10.0);
  while ((!received_tf_msg_) && (node->now() < timeout))
  {
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.10).to_chrono<std::chrono::nanoseconds>());
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

// NOTE(CH3): This main is required because the test is manually run by a launch test
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
