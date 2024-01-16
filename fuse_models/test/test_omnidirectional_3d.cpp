/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#include <gtest/gtest.h>

#include <fuse_graphs/hash_graph.hpp>
#include <fuse_models/omnidirectional_3d.hpp>
#include <fuse_variables/acceleration_linear_3d_stamped.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>
#include <fuse_variables/velocity_angular_3d_stamped.hpp>
#include <fuse_variables/velocity_linear_3d_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

/**
 * @brief Derived class used in unit tests to expose protected functions
 */
class Omnidirectional3DModelTest : public fuse_models::Omnidirectional3D
{
public:
  using fuse_models::Omnidirectional3D::updateStateHistoryEstimates;
  using fuse_models::Omnidirectional3D::StateHistoryElement;
  using fuse_models::Omnidirectional3D::StateHistory;
};

TEST(Omnidirectional3D, UpdateStateHistoryEstimates)
{
  // Create some variables
  auto position1 = fuse_variables::Position3DStamped::make_shared(rclcpp::Time(1, 0));
  auto orientation1 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(1, 0));
  auto linear_velocity1 = fuse_variables::VelocityLinear3DStamped::make_shared(rclcpp::Time(1, 0));
  auto angular_velocity1 = fuse_variables::VelocityAngular3DStamped::make_shared(rclcpp::Time(1, 0));
  auto linear_acceleration1 =
    fuse_variables::AccelerationLinear3DStamped::make_shared(rclcpp::Time(1, 0));
  position1->x() = 1.1;
  position1->y() = 2.1;
  position1->z() = 0.0;
  orientation1->w() = 1.0;
  orientation1->x() = 0.0;
  orientation1->y() = 0.0;
  orientation1->z() = 0.0;
  linear_velocity1->x() = 1.0;
  linear_velocity1->y() = 0.0;
  linear_velocity1->z() = 0.0;
  angular_velocity1->roll() = 0.0;
  angular_velocity1->pitch() = 0.0;
  angular_velocity1->yaw() = 0.0;
  linear_acceleration1->x() = 1.0;
  linear_acceleration1->y() = 0.0;
  linear_acceleration1->z() = 0.0;
  auto position2 = fuse_variables::Position3DStamped::make_shared(rclcpp::Time(2, 0));
  auto orientation2 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(2, 0));
  auto linear_velocity2 = fuse_variables::VelocityLinear3DStamped::make_shared(rclcpp::Time(2, 0));
  auto angular_velocity2 = fuse_variables::VelocityAngular3DStamped::make_shared(rclcpp::Time(2, 0));
  auto linear_acceleration2 =
    fuse_variables::AccelerationLinear3DStamped::make_shared(rclcpp::Time(2, 0));
  position2->x() = 1.2;
  position2->y() = 2.2;
  position2->z() = 0.0;
  orientation2->w() = 1.0;
  orientation2->x() = 0.0;
  orientation2->y() = 0.0;
  orientation2->z() = 0.0;
  linear_velocity2->x() = 0.0;
  linear_velocity2->y() = 1.0;
  linear_velocity2->z() = 0.0;
  angular_velocity2->roll() = 0.0;
  angular_velocity2->pitch() = 0.0;
  angular_velocity2->yaw() = 0.0;
  linear_acceleration2->x() = 0.0;
  linear_acceleration2->y() = 1.0;
  linear_acceleration2->z() = 0.0;
  auto position3 = fuse_variables::Position3DStamped::make_shared(rclcpp::Time(3, 0));
  auto orientation3 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(3, 0));
  auto linear_velocity3 = fuse_variables::VelocityLinear3DStamped::make_shared(rclcpp::Time(3, 0));
  auto angular_velocity3 = fuse_variables::VelocityAngular3DStamped::make_shared(rclcpp::Time(3, 0));
  auto linear_acceleration3 =
    fuse_variables::AccelerationLinear3DStamped::make_shared(rclcpp::Time(3, 0));
  position3->x() = 1.3;
  position3->y() = 2.3;
  position3->z() = 0.0;
  orientation3->w() = 1.0;
  orientation3->x() = 0.0;
  orientation3->y() = 0.0;
  orientation3->z() = 0.0;
  linear_velocity3->x() = 4.3;
  linear_velocity3->y() = 5.3;
  linear_velocity3->z() = 0.0;
  angular_velocity3->roll() = 0.0;
  angular_velocity3->pitch() = 0.0;
  angular_velocity3->yaw() = 0.0;
  linear_acceleration3->x() = 7.3;
  linear_acceleration3->y() = 8.3;
  linear_acceleration3->z() = 0.0;
  auto position4 = fuse_variables::Position3DStamped::make_shared(rclcpp::Time(4, 0));
  auto orientation4 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(4, 0));
  auto linear_velocity4 = fuse_variables::VelocityLinear3DStamped::make_shared(rclcpp::Time(4, 0));
  auto angular_velocity4 = fuse_variables::VelocityAngular3DStamped::make_shared(rclcpp::Time(4, 0));
  auto linear_acceleration4 =
    fuse_variables::AccelerationLinear3DStamped::make_shared(rclcpp::Time(4, 0));
  position4->x() = 1.4;
  position4->y() = 2.4;
  position4->z() = 0.0;
  orientation4->w() = 1.0;
  orientation4->x() = 0.0;
  orientation4->y() = 0.0;
  orientation4->z() = 0.0;
  linear_velocity4->x() = 4.4;
  linear_velocity4->y() = 5.4;
  linear_velocity4->z() = 0.0;
  angular_velocity4->roll() = 0.0;
  angular_velocity4->pitch() = 0.0;
  angular_velocity4->yaw() = 0.0;
  linear_acceleration4->x() = 7.4;
  linear_acceleration4->y() = 8.4;
  linear_acceleration4->z() = 0.0;
  auto position5 = fuse_variables::Position3DStamped::make_shared(rclcpp::Time(5, 0));
  auto orientation5 = fuse_variables::Orientation3DStamped::make_shared(rclcpp::Time(5, 0));
  auto linear_velocity5 = fuse_variables::VelocityLinear3DStamped::make_shared(rclcpp::Time(5, 0));
  auto angular_velocity5 = fuse_variables::VelocityAngular3DStamped::make_shared(rclcpp::Time(5, 0));
  auto linear_acceleration5 =
    fuse_variables::AccelerationLinear3DStamped::make_shared(rclcpp::Time(5, 0));
  position5->x() = 1.5;
  position5->y() = 2.5;
  position5->z() = 0.0;
  orientation5->w() = 1.0;
  orientation5->x() = 0.0;
  orientation5->y() = 0.0;
  orientation5->z() = 0.0;
  linear_velocity5->x() = 4.5;
  linear_velocity5->y() = 5.5;
  linear_velocity5->z() = 0.0;
  angular_velocity5->roll() = 0.0;
  angular_velocity5->pitch() = 0.0;
  angular_velocity5->yaw() = 0.0;
  linear_acceleration5->x() = 7.5;
  linear_acceleration5->y() = 8.5;
  linear_acceleration5->z() = 0.0;

  // Add a subset of the variables to a graph
  fuse_graphs::HashGraph graph;
  graph.addVariable(position2);
  graph.addVariable(orientation2);
  graph.addVariable(linear_velocity2);
  graph.addVariable(angular_velocity2);
  graph.addVariable(linear_acceleration2);

  graph.addVariable(position4);
  graph.addVariable(orientation4);
  graph.addVariable(linear_velocity4);
  graph.addVariable(angular_velocity4);
  graph.addVariable(linear_acceleration4);

  // Add all of the variables to the state history
  Omnidirectional3DModelTest::StateHistory state_history;
  state_history.emplace(
    position1->stamp(),
    Omnidirectional3DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
    position1->uuid(),
    orientation1->uuid(),
    linear_velocity1->uuid(),
    angular_velocity1->uuid(),
    linear_acceleration1->uuid(),
    fuse_core::Vector3d(1.0, 0.0, 0.0),
    Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0)});    // NOLINT(whitespace/braces)
  state_history.emplace(
    position2->stamp(),
    Omnidirectional3DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
    position2->uuid(),
    orientation2->uuid(),
    linear_velocity2->uuid(),
    angular_velocity2->uuid(),
    linear_acceleration2->uuid(),
    fuse_core::Vector3d(2.0, 0.0, 0.0),
    Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0)});    // NOLINT(whitespace/braces)
  state_history.emplace(
    position3->stamp(),
    Omnidirectional3DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
    position3->uuid(),
    orientation3->uuid(),
    linear_velocity3->uuid(),
    angular_velocity3->uuid(),
    linear_acceleration3->uuid(),
    fuse_core::Vector3d(3.0, 0.0, 0.0),
    Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0)});    // NOLINT(whitespace/braces)
  state_history.emplace(
    position4->stamp(),
    Omnidirectional3DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
    position4->uuid(),
    orientation4->uuid(),
    linear_velocity4->uuid(),
    angular_velocity4->uuid(),
    linear_acceleration4->uuid(),
    fuse_core::Vector3d(4.0, 0.0, 0.0),
    Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0)});    // NOLINT(whitespace/braces)
  state_history.emplace(
    position5->stamp(),
    Omnidirectional3DModelTest::StateHistoryElement{  // NOLINT(whitespace/braces)
    position5->uuid(),
    orientation5->uuid(),
    linear_velocity5->uuid(),
    angular_velocity5->uuid(),
    linear_acceleration5->uuid(),
    fuse_core::Vector3d(5.0, 0.0, 0.0),
    Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0),
    fuse_core::Vector3d(0.0, 0.0, 0.0)});    // NOLINT(whitespace/braces)

  // Update the state history
  Omnidirectional3DModelTest::updateStateHistoryEstimates(
    graph, state_history, rclcpp::Duration::from_seconds(
      10.0));

  // Check the state estimates in the state history
  {
    // The first entry is missing from the graph. It will not get updated.
    auto expected_position = fuse_core::Vector3d(1.0, 0.0, 0.0);
    auto expected_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    auto actual_position = state_history[rclcpp::Time(1, 0)].position;
    auto actual_orientation = state_history[rclcpp::Time(1, 0)].orientation;
    EXPECT_NEAR(expected_position.x(), actual_position.x(), 1.0e-9);
    EXPECT_NEAR(expected_position.y(), actual_position.y(), 1.0e-9);
    EXPECT_NEAR(expected_position.z(), actual_position.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.x(), actual_orientation.x(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.y(), actual_orientation.y(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.z(), actual_orientation.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.w(), actual_orientation.w(), 1.0e-9);

    auto expected_linear_velocity = fuse_core::Vector3d(0.0, 0.0, 0.0);
    auto actual_linear_velocity = state_history[rclcpp::Time(1, 0)].vel_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.z(), actual_linear_velocity.z(), 1.0e-9);

    auto expected_angular_velocity = fuse_core::Vector3d(0.0, 0.0, 0.0);
    auto actual_angular_velocity = state_history[rclcpp::Time(1, 0)].vel_angular;
    EXPECT_NEAR(expected_angular_velocity.x(), actual_angular_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.y(), actual_angular_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.z(), actual_angular_velocity.z(), 1.0e-9);

    auto expected_linear_acceleration = fuse_core::Vector3d(0.0, 0.0, 0.0);
    auto actual_linear_acceleration = state_history[rclcpp::Time(1, 0)].acc_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.z(), actual_linear_acceleration.z(), 1.0e-9);
  }
  {
    // The second entry is included in the graph. It will get updated directly.
    auto expected_position = fuse_core::Vector3d(1.2, 2.2, 0.0); // <-- value in the Graph
    auto expected_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    auto actual_position = state_history[rclcpp::Time(2, 0)].position;
    auto actual_orientation = state_history[rclcpp::Time(2, 0)].orientation;
    EXPECT_NEAR(expected_position.x(), actual_position.x(), 1.0e-9);
    EXPECT_NEAR(expected_position.y(), actual_position.y(), 1.0e-9);
    EXPECT_NEAR(expected_position.z(), actual_position.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.x(), actual_orientation.x(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.y(), actual_orientation.y(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.z(), actual_orientation.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.w(), actual_orientation.w(), 1.0e-9);

    auto expected_linear_velocity = fuse_core::Vector3d(0.0, 1.0, 0.0);
    auto actual_linear_velocity = state_history[rclcpp::Time(2, 0)].vel_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.z(), actual_linear_velocity.z(), 1.0e-9);

    auto expected_angular_velocity = fuse_core::Vector3d(0.0, 0.0, 0.0);
    auto actual_angular_velocity = state_history[rclcpp::Time(2, 0)].vel_angular;
    EXPECT_NEAR(expected_angular_velocity.x(), actual_angular_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.y(), actual_angular_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.z(), actual_angular_velocity.z(), 1.0e-9);

    auto expected_linear_acceleration = fuse_core::Vector3d(0.0, 1.0, 0.0);
    auto actual_linear_acceleration = state_history[rclcpp::Time(2, 0)].acc_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.z(), actual_linear_acceleration.z(), 1.0e-9);
  }
  {
    // The third entry is missing from the graph. It will get predicted from previous state.
    auto expected_position = fuse_core::Vector3d(1.2, 3.7, 0.0);
    auto expected_orientation = 
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) * 
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    auto actual_position = state_history[rclcpp::Time(3, 0)].position;
    auto actual_orientation = state_history[rclcpp::Time(3, 0)].orientation;
    EXPECT_NEAR(expected_position.x(), actual_position.x(), 1.0e-9);
    EXPECT_NEAR(expected_position.y(), actual_position.y(), 1.0e-9);
    EXPECT_NEAR(expected_position.z(), actual_position.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.x(), actual_orientation.x(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.y(), actual_orientation.y(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.z(), actual_orientation.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.w(), actual_orientation.w(), 1.0e-9);

    auto expected_linear_velocity = fuse_core::Vector3d(0.0, 2.0, 0.0);    
    auto actual_linear_velocity = state_history[rclcpp::Time(3, 0)].vel_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.z(), actual_linear_velocity.z(), 1.0e-9);

    auto expected_angular_velocity = fuse_core::Vector3d(0.0, 0.0, 0.0);
    auto actual_angular_velocity = state_history[rclcpp::Time(3, 0)].vel_angular;
    EXPECT_NEAR(expected_angular_velocity.x(), actual_angular_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.y(), actual_angular_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.z(), actual_angular_velocity.z(), 1.0e-9);

    auto expected_linear_acceleration = fuse_core::Vector3d(0.0, 1.0, 0.0);
    auto actual_linear_acceleration = state_history[rclcpp::Time(3, 0)].acc_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.z(), actual_linear_acceleration.z(), 1.0e-9);
  }
  {
    // The forth entry is included in the graph. It will get updated directly.
    auto expected_position = fuse_core::Vector3d(1.4, 2.4, 0.0); // <-- value in the Graph
    auto expected_orientation = 
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) * 
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    auto actual_position = state_history[rclcpp::Time(4, 0)].position;
    auto actual_orientation = state_history[rclcpp::Time(4, 0)].orientation;
    EXPECT_NEAR(expected_position.x(), actual_position.x(), 1.0e-9);
    EXPECT_NEAR(expected_position.y(), actual_position.y(), 1.0e-9);
    EXPECT_NEAR(expected_position.z(), actual_position.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.x(), actual_orientation.x(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.y(), actual_orientation.y(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.z(), actual_orientation.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.w(), actual_orientation.w(), 1.0e-9);

    auto expected_linear_velocity = fuse_core::Vector3d(4.4, 5.4, 0.0);
    auto actual_linear_velocity = state_history[rclcpp::Time(4, 0)].vel_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.z(), actual_linear_velocity.z(), 1.0e-9);

    auto expected_angular_velocity = fuse_core::Vector3d(0.0, 0.0, 0.0);
    auto actual_angular_velocity = state_history[rclcpp::Time(4, 0)].vel_angular;
    EXPECT_NEAR(expected_angular_velocity.x(), actual_angular_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.y(), actual_angular_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.z(), actual_angular_velocity.z(), 1.0e-9);

    auto expected_linear_acceleration = fuse_core::Vector3d(7.4, 8.4, 0.0);
    auto actual_linear_acceleration = state_history[rclcpp::Time(4, 0)].acc_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.z(), actual_linear_acceleration.z(), 1.0e-9);
  }
  {
    // The fifth entry is missing from the graph. It will get predicted from previous state.
    auto expected_position = fuse_core::Vector3d(9.5, 12.0, 0.0);
    auto expected_orientation = 
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) * 
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    auto actual_position = state_history[rclcpp::Time(5, 0)].position;
    auto actual_orientation = state_history[rclcpp::Time(5, 0)].orientation;
    EXPECT_NEAR(expected_position.x(), actual_position.x(), 1.0e-9);
    EXPECT_NEAR(expected_position.y(), actual_position.y(), 1.0e-9);
    EXPECT_NEAR(expected_position.z(), actual_position.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.x(), actual_orientation.x(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.y(), actual_orientation.y(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.z(), actual_orientation.z(), 1.0e-9);
    EXPECT_NEAR(expected_orientation.w(), actual_orientation.w(), 1.0e-9);

    auto expected_linear_velocity = fuse_core::Vector3d(11.8, 13.8, 0.0);
    auto actual_linear_velocity = state_history[rclcpp::Time(5, 0)].vel_linear;
    EXPECT_NEAR(expected_linear_velocity.x(), actual_linear_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.y(), actual_linear_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_velocity.z(), actual_linear_velocity.z(), 1.0e-9);

    auto expected_angular_velocity = fuse_core::Vector3d(0.0, 0.0, 0.0);
    auto actual_angular_velocity = state_history[rclcpp::Time(5, 0)].vel_angular;
    EXPECT_NEAR(expected_angular_velocity.x(), actual_angular_velocity.x(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.y(), actual_angular_velocity.y(), 1.0e-9);
    EXPECT_NEAR(expected_angular_velocity.z(), actual_angular_velocity.z(), 1.0e-9);  

    auto expected_linear_acceleration = fuse_core::Vector3d(7.4, 8.4, 0.0);
    auto actual_linear_acceleration = state_history[rclcpp::Time(5, 0)].acc_linear;
    EXPECT_NEAR(expected_linear_acceleration.x(), actual_linear_acceleration.x(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.y(), actual_linear_acceleration.y(), 1.0e-9);
    EXPECT_NEAR(expected_linear_acceleration.z(), actual_linear_acceleration.z(), 1.0e-9);
  }
}
