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
#include <gtest/gtest.h>

#include <fuse_core/throttled_callback.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief A helper class to publish a given number geometry_msgs::msg::Point messages at a given
 *        frequency.
 *
 * The messages published are geometry_msgs::msg::Point because it is simple. The 'x' field is
 * set to the number of messages published so far, starting at 0.
 */
class PointPublisher : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] frequency The publishing frequency in Hz
   */
  explicit PointPublisher(const double frequency)
  : Node("point_publisher_node")
    , frequency_(frequency)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("point", 1);
  }

  /**
   * @brief Get the internal node pointer
   *
   * @return the node pointer
   */
  rclcpp::Node::SharedPtr getNode()
  {
    return shared_from_this();
  }

  /**
   * @brief Publish the given number of messages
   *
   * @param[in] num_messages The number of messages to publish
   */
  void publish(const size_t num_messages)
  {
    // Wait for the subscriptions to be ready before sending them data:
    rclcpp::Time subscription_timeout = this->now() + rclcpp::Duration::from_seconds(1.0);
    while (publisher_->get_subscription_count() < 1u && this->now() < subscription_timeout) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_GE(publisher_->get_subscription_count(), 1u);

    // Send data:
    rclcpp::Rate rate(frequency_);
    for (size_t i = 0; i < num_messages; ++i) {
      geometry_msgs::msg::Point point_message;
      point_message.x = i;
      publisher_->publish(point_message);
      rate.sleep();
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;  //!< The publisher
  double frequency_{10.0};  //!< The publish rate frequency
};

/**
 * @brief A dummy point sensor model that uses a
 *        fuse_core::ThrottledMessageCallback<geometry_msgs::msg::Point> with a keep and drop
 *        callback.
 *
 * The callbacks simply count the number of times they are called, for testing purposes. The keep
 * callback also caches the last message received, also for testing purposes.
 */
class PointSensorModel : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] throttle_period The throttle period duration in seconds
   */
  explicit PointSensorModel(const rclcpp::Duration & throttle_period)
  : Node("point_sensor_model_node")
    , throttled_callback_(
      std::bind(&PointSensorModel::keepCallback, this, std::placeholders::_1),
      std::bind(&PointSensorModel::dropCallback, this, std::placeholders::_1),
      throttle_period
    )
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "point", 10,
      std::bind(
        &PointThrottledCallback::callback<const geometry_msgs::msg::Point &>,
        &throttled_callback_, std::placeholders::_1)
    );
  }

  /**
   * @brief Get the internal node pointer
   *
   * @return the node pointer
   */
  rclcpp::Node::SharedPtr getNode()
  {
    return shared_from_this();
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
  const geometry_msgs::msg::Point::SharedPtr getLastKeptMessage() const
  {
    return last_kept_message_;
  }

private:
  /**
   * @brief Keep callback, that counts the number of times it has been called and caches the last
   *        message received
   *
   * @param[in] msg A geometry_msgs::msg::Point message
   */
  void keepCallback(const geometry_msgs::msg::Point & msg)
  {
    ++kept_messages_;
    last_kept_message_ = std::make_shared<geometry_msgs::msg::Point>(msg);
  }

  /**
   * @brief Drop callback, that counts the number of times it has been called
   *
   * @param[in] msg A geometry_msgs::msg::Point message (not used)
   */
  // NOTE(CH3): The msg arg here is necessary to allow binding the throttled callback
  void dropCallback(const geometry_msgs::msg::Point & /*msg*/)
  {
    ++dropped_messages_;
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;   //!< The subscription

  using PointThrottledCallback = fuse_core::ThrottledMessageCallback<geometry_msgs::msg::Point>;
  PointThrottledCallback throttled_callback_;  //!< The throttled callback

  size_t kept_messages_{0};                           //!< Messages kept
  size_t dropped_messages_{0};                        //!< Messages dropped

  // We use a SharedPtr to check for nullptr just for this test
  geometry_msgs::msg::Point::SharedPtr last_kept_message_;  //!< The last message kept
};

class TestThrottledCallback : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner_ = std::thread(
      [&]() {
        executor_->spin();
      });
  }

  void TearDown() override
  {
    executor_->cancel();
    rclcpp::shutdown();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    executor_.reset();
  }

  std::thread spinner_;  //!< Internal thread for spinning the executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(TestThrottledCallback, NoDroppedMessagesIfThrottlePeriodIsZero)
{
  // Start sensor model to listen to messages:
  const rclcpp::Duration throttled_period(0, 0);
  auto sensor_model = std::make_shared<PointSensorModel>(throttled_period);
  executor_->add_node(sensor_model);

  // Time should be valid after the context is initialized. But it doesn't hurt to verify.
  ASSERT_TRUE(
    sensor_model->getNode()->get_clock()->wait_until_started(
      rclcpp::Duration::from_seconds(1.0)));

  // Publish some messages:
  const size_t num_messages = 10;
  const double frequency = 10.0;

  auto publisher = std::make_shared<PointPublisher>(frequency);
  executor_->add_node(publisher);
  publisher->publish(num_messages);

  // Check all messages are kept and none are dropped, because when the throttle period is zero,
  // throttling is disabled:
  EXPECT_EQ(num_messages, sensor_model->getKeptMessages());
  EXPECT_EQ(0u, sensor_model->getDroppedMessages());
}

TEST_F(TestThrottledCallback, DropMessagesIfThrottlePeriodIsGreaterThanPublishPeriod)
{
  // Start sensor model to listen to messages:
  const rclcpp::Duration throttled_period(0, static_cast<uint32_t>(RCUTILS_S_TO_NS(0.2)));
  auto sensor_model = std::make_shared<PointSensorModel>(throttled_period);
  executor_->add_node(sensor_model);

  // Time should be valid after the context is initialized. But it doesn't hurt to verify.
  ASSERT_TRUE(
    sensor_model->getNode()->get_clock()->wait_until_started(
      rclcpp::Duration::from_seconds(1.0)));

  // Publish some messages at half the throttled period:
  const size_t num_messages = 10;
  const double period_factor = 0.25;
  const double period = period_factor * throttled_period.seconds();
  const double frequency = 1.0 / period;

  auto publisher = std::make_shared<PointPublisher>(frequency);
  executor_->add_node(publisher);
  publisher->publish(num_messages);

  // Check the number of kept and dropped callbacks:
  const auto expected_kept_messages = period_factor * num_messages;
  const auto expected_dropped_messages = num_messages - expected_kept_messages;

  EXPECT_NEAR(expected_kept_messages, sensor_model->getKeptMessages(), 1.0);
  EXPECT_NEAR(expected_dropped_messages, sensor_model->getDroppedMessages(), 1.0);
}

TEST_F(TestThrottledCallback, AlwaysKeepFirstMessageEvenIfThrottlePeriodIsTooLarge)
{
  // Start sensor model to listen to messages:
  const rclcpp::Duration throttled_period(10, 0);
  auto sensor_model = std::make_shared<PointSensorModel>(throttled_period);
  executor_->add_node(sensor_model);

  // Time should be valid after the context is initialized. But it doesn't hurt to verify.
  ASSERT_TRUE(
    sensor_model->getNode()->get_clock()->wait_until_started(
      rclcpp::Duration::from_seconds(1.0)));

  ASSERT_EQ(nullptr, sensor_model->getLastKeptMessage());

  // Publish some messages:
  const size_t num_messages = 10;
  const double period = 0.1 * num_messages / throttled_period.seconds();
  const double frequency = 1.0 / period;

  auto publisher = std::make_shared<PointPublisher>(frequency);
  publisher->publish(num_messages);
  executor_->add_node(publisher);

  // Check that regardless of the large throttled period, at least one message is ketpt:
  EXPECT_EQ(1u, sensor_model->getKeptMessages());
  EXPECT_EQ(num_messages - 1u, sensor_model->getDroppedMessages());

  // Check the message kept was the first message:
  const auto last_kept_message = sensor_model->getLastKeptMessage();
  ASSERT_NE(nullptr, last_kept_message);
  EXPECT_EQ(0.0, last_kept_message->x);
}
