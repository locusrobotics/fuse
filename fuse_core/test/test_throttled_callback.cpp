/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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
#include <fuse_core/throttled_callback.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <gtest/gtest.h>


/**
 * @brief A helper class to publish a given number geometry_msgs::Point messages at a given frequency.
 *
 * The messages published are geometry_msgs::Point because it is simple. The 'x' field is set to the number of messages
 * published so far, starting at 0.
 */
class PointPublisher
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] frequency The publishing frequency in Hz
   */
  explicit PointPublisher(const double frequency)
    : frequency_(frequency)
  {
    publisher_ = node_handle_.advertise<geometry_msgs::Point>("point", 1);
  }

  /**
   * @brief Publish the given number of messages
   *
   * @param[in] num_messages The number of messages to publish
   */
  void publish(const size_t num_messages)
  {
    // Wait for the subscribers to be ready before sending them data:
    rclcpp::Time subscriber_timeout = this->node_->now() + rclcpp::Duration::from_seconds(1.0);
    while (publisher_.getNumSubscribers() < 1u && this->node_->now() < subscriber_timeout)
    {
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.01);
    }

    ASSERT_GE(publisher_.getNumSubscribers(), 1u);

    // Send data:
    ros::Rate rate(frequency_);
    for (size_t i = 0; i < num_messages; ++i)
    {
      geometry_msgs::Point point_message;
      point_message.x = i;

      publisher_.publish(point_message);

      rate.sleep();
    }
  }

private:
  // TODO(CH3): Make this an rclcpp node. It's a test, we don't need the node interfaces.
  ros::NodeHandle node_handle_;  //!< The node handle
  ros::Publisher publisher_;     //!< The publisher
  double frequency_{ 10.0 };     //!< The publish rate frequency
};

/**
 * @brief A dummy point sensor model that uses a fuse_core::ThrottledMessageCallback<geometry_msgs::Point> with a keep
 * and drop callback.
 *
 * The callbacks simply count the number of times they are called, for testing purposes. The keep callback also caches
 * the last message received, also for testing purposes.
 */
class PointSensorModel
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] throttle_period The throttle period duration in seconds
   */
  explicit PointSensorModel(const rclcpp::Duration& throttle_period)
    : throttled_callback_(std::bind(&PointSensorModel::keepCallback, this, std::placeholders::_1),
                          std::bind(&PointSensorModel::dropCallback, this, std::placeholders::_1), throttle_period)
  {
    subscriber_ = node_handle_.subscribe<geometry_msgs::Point>(
        "point", 10, &PointThrottledCallback::callback, &throttled_callback_);
  }

  /**
   * @brief The kept messages counter getter
   *
   * @return The number of messages kept
   */
  size_t getKeptMessages() const
  {
    return kept_messages_;
  }

  /**
   * @brief The dropped messages counter getter
   *
   * @return The number of messages dropped
   */
  size_t getDroppedMessages() const
  {
    return dropped_messages_;
  }

  /**
   * @brief The last message kept getter
   *
   * @return The last message kept. It would be nullptr if no message has been kept so far
   */
  const geometry_msgs::Point::ConstPtr getLastKeptMessage() const
  {
    return last_kept_message_;
  }

private:
  /**
   * @brief Keep callback, that counts the number of times it has been called and caches the last message received
   *
   * @param[in] msg A geometry_msgs::Point message
   */
  void keepCallback(const geometry_msgs::Point::ConstPtr& msg)
  {
    ++kept_messages_;
    last_kept_message_ = msg;
  }

  /**
   * @brief Drop callback, that counts the number of times it has been called
   *
   * @param[in] msg A geometry_msgs::Point message (not used)
   */
  void dropCallback(const geometry_msgs::Point::ConstPtr& /*msg*/)
  {
    ++dropped_messages_;
  }

  ros::NodeHandle node_handle_;  //!< The node handle
  ros::Subscriber subscriber_;   //!< The subscriber

  using PointThrottledCallback = fuse_core::ThrottledMessageCallback<geometry_msgs::Point>;
  PointThrottledCallback throttled_callback_;  //!< The throttled callback

  size_t kept_messages_{ 0 };                         //!< Messages kept
  size_t dropped_messages_{ 0 };                      //!< Messages dropped
  geometry_msgs::Point::ConstPtr last_kept_message_;  //!< The last message kept
};


TEST(ThrottledCallback, NoDroppedMessagesIfThrottlePeriodIsZero)
{
  // Time should be valid after ros::init() returns in main(). But it doesn't hurt to verify.
  // TODO(CH3): Refactor this with a passed in node
  // ASSERT_TRUE(fuse_core::wait_for_valid(<NODE_HERE>->get_clock(), rclcpp::Duration::from_seconds(1.0)));

  // Start sensor model to listen to messages:
  const rclcpp::Duration throttled_period(0.0);
  PointSensorModel sensor_model(throttled_period);

  // Publish some messages:
  const size_t num_messages = 10;
  const double frequency = 10.0;

  PointPublisher publisher(frequency);
  publisher.publish(num_messages);

  // Check all messages are kept and none are dropped, because when the throttle period is zero, throttling is disabled:
  EXPECT_EQ(num_messages, sensor_model.getKeptMessages());
  EXPECT_EQ(0u, sensor_model.getDroppedMessages());
}

TEST(ThrottledCallback, DropMessagesIfThrottlePeriodIsGreaterThanPublishPeriod)
{
  // Time should be valid after ros::init() returns in main(). But it doesn't hurt to verify.
  // TODO(CH3): Refactor this with a passed in node
  // ASSERT_TRUE(fuse_core::wait_for_valid(<NODE_HERE>->get_clock(), rclcpp::Duration::from_seconds(1.0)));

  // Start sensor model to listen to messages:
  const rclcpp::Duration throttled_period(0.2);
  PointSensorModel sensor_model(throttled_period);

  // Publish some messages at half the throttled period:
  const size_t num_messages = 10;
  const double period_factor = 0.25;
  const double period = period_factor * throttled_period.seconds();
  const double frequency = 1.0 / period;

  PointPublisher publisher(frequency);
  publisher.publish(num_messages);

  // Check the number of kept and dropped callbacks:
  const auto expected_kept_messages = period_factor * num_messages;
  const auto expected_dropped_messages = num_messages - expected_kept_messages;

  EXPECT_NEAR(expected_kept_messages, sensor_model.getKeptMessages(), 1.0);
  EXPECT_NEAR(expected_dropped_messages, sensor_model.getDroppedMessages(), 1.0);
}

TEST(ThrottledCallback, AlwaysKeepFirstMessageEvenIfThrottlePeriodIsTooLarge)
{
  // Time should be valid after ros::init() returns in main(). But it doesn't hurt to verify.
  // TODO(CH3): Refactor this with a passed in node
  // ASSERT_TRUE(fuse_core::wait_for_valid(<NODE_HERE>->get_clock(), rclcpp::Duration::from_seconds(1.0)));

  // Start sensor model to listen to messages:
  const rclcpp::Duration throttled_period(10.0);
  PointSensorModel sensor_model(throttled_period);

  ASSERT_EQ(nullptr, sensor_model.getLastKeptMessage());

  // Publish some messages:
  const size_t num_messages = 10;
  const double period = 0.1 * num_messages / throttled_period.seconds();
  const double frequency = 1.0 / period;

  PointPublisher publisher(frequency);
  publisher.publish(num_messages);

  // Check that regardless of the large throttled period, at least one message is ketpt:
  EXPECT_EQ(1u, sensor_model.getKeptMessages());
  EXPECT_EQ(num_messages - 1u, sensor_model.getDroppedMessages());

  // Check the message kept was the first message:
  const auto last_kept_message = sensor_model.getLastKeptMessage();
  ASSERT_NE(nullptr, last_kept_message);
  EXPECT_EQ(0.0, last_kept_message->x);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "throttled_callback_test");
  auto spinner = ros::AsyncSpinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
