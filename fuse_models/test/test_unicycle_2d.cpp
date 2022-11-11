/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#include <fuse_models/unicycle_2d.h>

#include <fuse_graphs/hash_graph.h>
#include <fuse_variables/acceleration_linear_2d_stamped.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_variables/velocity_angular_2d_stamped.h>
#include <fuse_variables/velocity_linear_2d_stamped.h>
#include <rclcpp/duration.hpp>
#include <fuse_core/time.h>

#include <gtest/gtest.h>


/**
 * @brief Derived class used in unit tests to expose protected functions
 */
class Unicycle2DModelTest : public fuse_models::Unicycle2D
{
public:
  using fuse_models::Unicycle2D::updateStateHistoryEstimates;
  using fuse_models::Unicycle2D::StateHistoryElement;
  using fuse_models::Unicycle2D::StateHistory;
};

TEST(Unicycle2D, UpdateStateHistoryEstimates)
{
  // Create some variables
  auto position1 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(1, 0));
  auto yaw1 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(1, 0));
  auto linear_velocity1 = fuse_variables::VelocityLinear2DStamped::make_shared(rclcpp::Time(1, 0));
  auto yaw_velocity1 = fuse_variables::VelocityAngular2DStamped::make_shared(rclcpp::Time(1, 0));
  auto linear_acceleration1 = fuse_variables::AccelerationLinear2DStamped::make_shared(rclcpp::Time(1, 0));
  position1->x() = 1.1;
  position1->y() = 2.1;
  yaw1->yaw() = 3.1;
  linear_velocity1->x() = 1.0;
  linear_velocity1->y() = 0.0;
  yaw_velocity1->yaw() = 0.0;
  linear_acceleration1->x() = 1.0;
  linear_acceleration1->y() = 0.0;
  auto position2 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(2, 0));
  auto yaw2 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(2, 0));
  auto linear_velocity2 = fuse_variables::VelocityLinear2DStamped::make_shared(rclcpp::Time(2, 0));
  auto yaw_velocity2 = fuse_variables::VelocityAngular2DStamped::make_shared(rclcpp::Time(2, 0));
  auto linear_acceleration2 = fuse_variables::AccelerationLinear2DStamped::make_shared(rclcpp::Time(2, 0));
  position2->x() = 1.2;
  position2->y() = 2.2;
  yaw2->yaw() = M_PI / 2.0;
  linear_velocity2->x() = 0.0;
  linear_velocity2->y() = 1.0;
  yaw_velocity2->yaw() = 0.0;
  linear_acceleration2->x() = 0.0;
  linear_acceleration2->y() = 1.0;
  auto position3 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(3, 0));
  auto yaw3 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(3, 0));
  auto linear_velocity3 = fuse_variables::VelocityLinear2DStamped::make_shared(rclcpp::Time(3, 0));
  auto yaw_velocity3 = fuse_variables::VelocityAngular2DStamped::make_shared(rclcpp::Time(3, 0));
  auto linear_acceleration3 = fuse_variables::AccelerationLinear2DStamped::make_shared(rclcpp::Time(3, 0));
  position3->x() = 1.3;
  position3->y() = 2.3;
  yaw3->yaw() = 3.3;
  linear_velocity3->x() = 4.3;
  linear_velocity3->y() = 5.3;
  yaw_velocity3->yaw() = 6.3;
  linear_acceleration3->x() = 7.3;
  linear_acceleration3->y() = 8.3;
  auto position4 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(4, 0));
  auto yaw4 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(4, 0));
  auto linear_velocity4 = fuse_variables::VelocityLinear2DStamped::make_shared(rclcpp::Time(4, 0));
  auto yaw_velocity4 = fuse_variables::VelocityAngular2DStamped::make_shared(rclcpp::Time(4, 0));
  auto linear_acceleration4 = fuse_variables::AccelerationLinear2DStamped::make_shared(rclcpp::Time(4, 0));
  position4->x() = 1.4;
  position4->y() = 2.4;
  yaw4->yaw() = 3.4;
  linear_velocity4->x() = 4.4;
  linear_velocity4->y() = 5.4;
  yaw_velocity4->yaw() = 6.4;
  linear_acceleration4->x() = 7.4;
  linear_acceleration4->y() = 8.4;
  auto position5 = fuse_variables::Position2DStamped::make_shared(rclcpp::Time(5, 0));
  auto yaw5 = fuse_variables::Orientation2DStamped::make_shared(rclcpp::Time(5, 0));
  auto linear_velocity5 = fuse_variables::VelocityLinear2DStamped::make_shared(rclcpp::Time(5, 0));
  auto yaw_velocity5 = fuse_variables::VelocityAngular2DStamped::make_shared(rclcpp::Time(5, 0));
  auto linear_acceleration5 = fuse_variables::AccelerationLinear2DStamped::make_shared(rclcpp::Time(5, 0));
  position5->x() = 1.5;
  position5->y() = 2.5;
  yaw5->yaw() = 3.5;
  linear_velocity5->x() = 4.5;
  linear_velocity5->y() = 5.5;
  yaw_velocity5->yaw() = 6.5;
  linear_acceleration5->x() = 7.5;
  linear_acceleration5->y() = 8.5;

  // Add a subset of the variables to a graph
  fuse_graphs::HashGraph graph;
  graph.addVariable(position2);
  graph.addVariable(yaw2);
  graph.addVariable(linear_velocity2);
  graph.addVariable(yaw_velocity2);
  graph.addVariable(linear_acceleration2);

  graph.addVariable(position4);
  graph.addVariable(yaw4);
  graph.addVariable(linear_velocity4);
  graph.addVariable(yaw_velocity4);
  graph.addVariable(linear_acceleration4);

  // Add all of the variables to the state history
  Unicycle2DModelTest::StateHistory state_history;
  state_history.emplace(
    position1->stamp(),
    Unicycle2DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
      position1->uuid(),
      yaw1->uuid(),
      linear_velocity1->uuid(),
      yaw_velocity1->uuid(),
      linear_acceleration1->uuid(),
      tf2_2d::Transform(1.0, 0.0, 0.0),
      tf2_2d::Vector2(0.0, 0.0),
      0.0,
      tf2_2d::Vector2(0.0, 0.0)});  // NOLINT(whitespace/braces)
  state_history.emplace(
    position2->stamp(),
    Unicycle2DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
      position2->uuid(),
      yaw2->uuid(),
      linear_velocity2->uuid(),
      yaw_velocity2->uuid(),
      linear_acceleration2->uuid(),
      tf2_2d::Transform(2.0, 0.0, 0.0),
      tf2_2d::Vector2(0.0, 0.0),
      0.0,
      tf2_2d::Vector2(0.0, 0.0)});  // NOLINT(whitespace/braces)
  state_history.emplace(
    position3->stamp(),
    Unicycle2DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
      position3->uuid(),
      yaw3->uuid(),
      linear_velocity3->uuid(),
      yaw_velocity3->uuid(),
      linear_acceleration3->uuid(),
      tf2_2d::Transform(3.0, 0.0, 0.0),
      tf2_2d::Vector2(0.0, 0.0),
      0.0,
      tf2_2d::Vector2(0.0, 0.0)});  // NOLINT(whitespace/braces)
  state_history.emplace(
    position4->stamp(),
    Unicycle2DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
      position4->uuid(),
      yaw4->uuid(),
      linear_velocity4->uuid(),
      yaw_velocity4->uuid(),
      linear_acceleration4->uuid(),
      tf2_2d::Transform(4.0, 0.0, 0.0),
      tf2_2d::Vector2(0.0, 0.0),
      0.0,
      tf2_2d::Vector2(0.0, 0.0)});  // NOLINT(whitespace/braces)
  state_history.emplace(
    position5->stamp(),
    Unicycle2DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
      position5->uuid(),
      yaw5->uuid(),
      linear_velocity5->uuid(),
      yaw_velocity5->uuid(),
      linear_acceleration5->uuid(),
      tf2_2d::Transform(5.0, 0.0, 0.0),
      tf2_2d::Vector2(0.0, 0.0),
      0.0,
      tf2_2d::Vector2(0.0, 0.0)});  // NOLINT(whitespace/braces)

  // Update the state history
  Unicycle2DModelTest::updateStateHistoryEstimates(graph, state_history, rclcpp::Duration::from_seconds(10.0));

  // Check the state estimates in the state history
  {
    // The first entry is missing from the graph. It will not get updated.
    auto expected_pose = tf2_2d::Transform(1.0, 0.0, 0.0);  // <-- original value in StateHistory
    auto actual_pose = state_history[rclcpp::Time(1, 0)].pose;
    EXPECT_NEAR(expected_pose.x(), actual_pose.x(), 1.0e-9);
    EXPECT_NEAR(expected_pose.y(), actual_pose.y(), 1.0e-9);
    EXPECT_NEAR(expected_pose.angle(), actual_pose.angle(), 1.0e-9);

    auto expected_linear_velocity = tf2_2d::Vector2(0.0, 0.0);
    auto actual_linear_velocity = state_history[rclcpp::Time(1, 0)].velocity_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);

    auto expected_yaw_velocity = 0.0;
    auto actual_yaw_velocity = state_history[rclcpp::Time(1, 0)].velocity_yaw;
    EXPECT_NEAR(expected_yaw_velocity, actual_yaw_velocity, 1.0e-9);

    auto expected_linear_acceleration = tf2_2d::Vector2(0.0, 0.0);
    auto actual_linear_acceleration = state_history[rclcpp::Time(1, 0)].acceleration_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
  }
  {
    // The second entry is included in the graph. It will get updated directly.
    auto expected_pose = tf2_2d::Transform(1.2, 2.2, M_PI / 2.0);  // <-- value in the Graph
    auto actual_pose = state_history[rclcpp::Time(2, 0)].pose;
    EXPECT_NEAR(expected_pose.x(), actual_pose.x(), 1.0e-9);
    EXPECT_NEAR(expected_pose.y(), actual_pose.y(), 1.0e-9);
    EXPECT_NEAR(expected_pose.angle(), actual_pose.angle(), 1.0e-9);

    auto expected_linear_velocity = tf2_2d::Vector2(0.0, 1.0);
    auto actual_linear_velocity = state_history[rclcpp::Time(2, 0)].velocity_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);

    auto expected_yaw_velocity = 0.0;
    auto actual_yaw_velocity = state_history[rclcpp::Time(2, 0)].velocity_yaw;
    EXPECT_NEAR(expected_yaw_velocity, actual_yaw_velocity, 1.0e-9);

    auto expected_linear_acceleration = tf2_2d::Vector2(0.0, 1.0);
    auto actual_linear_acceleration = state_history[rclcpp::Time(2, 0)].acceleration_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
  }
  {
    // The third entry is missing from the graph. It will get predicted from previous state.
    auto expected_pose = tf2_2d::Transform(-0.3, 2.2, M_PI / 2.0);
    auto actual_pose = state_history[rclcpp::Time(3, 0)].pose;
    EXPECT_NEAR(expected_pose.x(), actual_pose.x(), 1.0e-9);
    EXPECT_NEAR(expected_pose.y(), actual_pose.y(), 1.0e-9);
    EXPECT_NEAR(expected_pose.angle(), actual_pose.angle(), 1.0e-9);

    auto expected_linear_velocity = tf2_2d::Vector2(0.0, 2.0);
    auto actual_linear_velocity = state_history[rclcpp::Time(3, 0)].velocity_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);

    auto expected_yaw_velocity = 0.0;
    auto actual_yaw_velocity = state_history[rclcpp::Time(3, 0)].velocity_yaw;
    EXPECT_NEAR(expected_yaw_velocity, actual_yaw_velocity, 1.0e-9);

    auto expected_linear_acceleration = tf2_2d::Vector2(0.0, 1.0);
    auto actual_linear_acceleration = state_history[rclcpp::Time(3, 0)].acceleration_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
  }
  {
    // The forth entry is included in the graph. It will get updated directly.
    auto expected_pose = tf2_2d::Transform(1.4, 2.4, 3.4);  // <-- value in the Graph
    auto actual_pose = state_history[rclcpp::Time(4, 0)].pose;
    EXPECT_NEAR(expected_pose.x(), actual_pose.x(), 1.0e-9);
    EXPECT_NEAR(expected_pose.y(), actual_pose.y(), 1.0e-9);
    EXPECT_NEAR(expected_pose.angle(), actual_pose.angle(), 1.0e-9);

    auto expected_linear_velocity = tf2_2d::Vector2(4.4, 5.4);
    auto actual_linear_velocity = state_history[rclcpp::Time(4, 0)].velocity_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);

    auto expected_yaw_velocity = 6.4;
    auto actual_yaw_velocity = state_history[rclcpp::Time(4, 0)].velocity_yaw;
    EXPECT_NEAR(expected_yaw_velocity, actual_yaw_velocity, 1.0e-9);

    auto expected_linear_acceleration = tf2_2d::Vector2(7.4, 8.4);
    auto actual_linear_acceleration = state_history[rclcpp::Time(4, 0)].acceleration_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
  }
  {
    // The fifth entry is missing from the graph. It will get predicted from previous state.
    // These values were verified with Octave
    auto expected_pose = tf2_2d::Transform(-3.9778707804360529, -8.9511455751801616, -2.7663706143591722);
    auto actual_pose = state_history[rclcpp::Time(5, 0)].pose;
    EXPECT_NEAR(expected_pose.x(), actual_pose.x(), 1.0e-9);
    EXPECT_NEAR(expected_pose.y(), actual_pose.y(), 1.0e-9);
    EXPECT_NEAR(expected_pose.angle(), actual_pose.angle(), 1.0e-9);

    auto expected_linear_velocity = tf2_2d::Vector2(11.8, 13.8);
    auto actual_linear_velocity = state_history[rclcpp::Time(5, 0)].velocity_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);

    auto expected_yaw_velocity = 6.4;
    auto actual_yaw_velocity = state_history[rclcpp::Time(5, 0)].velocity_yaw;
    EXPECT_NEAR(expected_yaw_velocity, actual_yaw_velocity, 1.0e-9);

    auto expected_linear_acceleration = tf2_2d::Vector2(7.4, 8.4);
    auto actual_linear_acceleration = state_history[rclcpp::Time(5, 0)].acceleration_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
