/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Clearpath Robotics
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

#include <angles/angles.h>
#include <fuse_constraints/absolute_constraint.h>
#include <fuse_constraints/relative_pose_2d_stamped_constraint.h>
#include <fuse_core/transaction.h>
#include <fuse_models/imu_2d.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_variables/velocity_angular_2d_stamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

/**
 * @brief Imu2D test fixture
 */
class Imu2DTestFixture : public ::testing::Test
{
public:
  Imu2DTestFixture()
  {
    cmd_vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/diffbot_controller/cmd_vel", 10);
    odom_subscriber_ = node_handle_.subscribe("odom", 10, &Imu2DTestFixture::odomCallback, this);

    cmd_vel_timer_ = node_handle_.createTimer(ros::Duration(0.1), &Imu2DTestFixture::cmdVelTimerCallback, this);
  }

  void transactionCallback(const fuse_core::Transaction::SharedPtr& transaction)
  {
    std::lock_guard<std::mutex> lock(last_transaction_mutex_);
    last_transaction_ = *transaction;
  }

  void setCmdVel(const geometry_msgs::Twist& cmd_vel)
  {
    cmd_vel_ = cmd_vel;
  }

  nav_msgs::Odometry getLastOdom() const
  {
    std::lock_guard<std::mutex> lock(last_odom_mutex_);
    return last_odom_;
  }

  fuse_core::Transaction getLastTransaction() const
  {
    std::lock_guard<std::mutex> lock(last_transaction_mutex_);
    return last_transaction_;
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(last_odom_mutex_);
    last_odom_ = *msg;
  }

  void cmdVelTimerCallback(const ros::TimerEvent&)
  {
    cmd_vel_publisher_.publish(cmd_vel_);
  }

  ros::NodeHandle node_handle_;       //!< A node handle
  ros::Publisher cmd_vel_publisher_;  //!< The command velocity publisher
  ros::Subscriber odom_subscriber_;   //!< The wheel odometry subscriber
  ros::Timer cmd_vel_timer_;  //!< A timer to send the command velocity to the robot continuously; if the command
                              //!< velocity is only sent once, after some time the controller considers the command
                              //!< velocity publisher hang up and resets the command velocity it uses to zero

  geometry_msgs::Twist cmd_vel_;  //!< The command velocity to send to the robot
  nav_msgs::Odometry last_odom_;  //!< The last odometry measurement received

  fuse_core::Transaction last_transaction_;  //!< The last transaction generated

  mutable std::mutex last_odom_mutex_;         //!< A mutex to protect last_odom_ access
  mutable std::mutex last_transaction_mutex_;  //!< A mutex to protect last_transaction_ access
};

TEST_F(Imu2DTestFixture, BaseFrameYawAngularVelocity)
{
  // Create an IMU sensor and register the transaction callback
  const std::string imu_2d_name{ "imu" };

  fuse_models::Imu2D imu_2d;
  imu_2d.initialize(imu_2d_name, std::bind(&Imu2DTestFixture::transactionCallback, this, std::placeholders::_1));
  imu_2d.start();

  // Make the robot turn in place for a few seconds
  geometry_msgs::Twist twist;
  twist.angular.z = 0.5;

  setCmdVel(twist);

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(3s);

  // Confirm the last transaction has the expect number of variables and constraints generated by the imu sensor model
  // source; if no transaction was received, the last transaction would not have any added variables
  const auto last_transaction = getLastTransaction();

  // That is:
  // * 5 variables:
  //   * 1 variable for the angular velocity
  //   * 4 variables for the current and previous position and orientation
  // * 2 constraints:
  //   * 1 constraints for the absolute angular velocity
  //   * 1 constraints for the relative pose between the current and previous position and orientation
  //
  const auto added_variables = last_transaction.addedVariables();
  const auto added_constraints = last_transaction.addedConstraints();

  ASSERT_EQ(5u, boost::size(added_variables));
  ASSERT_EQ(2u, boost::size(added_constraints));

  // With all constraints generated by the imu sensor model source
  for (const auto& constraint : added_constraints)
  {
    ASSERT_EQ(imu_2d_name, constraint.source());
  }

  // And no variables or constraints are removed
  ASSERT_TRUE(boost::empty(last_transaction.removedVariables()));
  ASSERT_TRUE(boost::empty(last_transaction.removedConstraints()));

  // Confirm the last transaction generated matches the last wheel odometry received
  //
  // Note that we are not synchronizing the odometry measurements with the last transaction stamp or involved stamps,
  // under the assumption the velocity is steady at the time it is being sampled. However, this is likely the reason the
  // tolerance in the checks below is too high. With 1.0e-3 most of the checks fail even when things are working
  // properly.
  const auto last_odom = getLastOdom();

  // First, confirm the fuse_variables::VelocityAngular2DStamped variable yaw is the same as the wheel odometry twist
  // angular velocity along the z axis
  const auto& w = last_odom.twist.twist.angular.z;

  // That is, ensure there is one fuse_variables::VelocityAngular2DStamped variable
  EXPECT_EQ(1, std::count_if(added_variables.begin(), added_variables.end(), [](const auto& variable) -> bool {
              return dynamic_cast<const fuse_variables::VelocityAngular2DStamped*>(&variable);
            }));

  // And iterate over all added variables to process it
  for (const auto& variable : added_variables)
  {
    // Skip if not a fuse_variables::VelocityAngular2DStamped variable
    const auto velocity_angular = dynamic_cast<const fuse_variables::VelocityAngular2DStamped*>(&variable);
    if (!velocity_angular)
    {
      continue;
    }

    // Check the angular velocity yaw is the same as the wheel odometry one
    EXPECT_NEAR(w, velocity_angular->yaw(), 1.0e-2);
  }

  // Second, confirm the fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint constraint mean is the same as the
  // wheel odometry twist angular velocity along the z axis

  // That is, ensure there is one fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint constraint
  EXPECT_EQ(1, std::count_if(added_constraints.begin(), added_constraints.end(), [](const auto& constraint) -> bool {
              return dynamic_cast<const fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint*>(&constraint);
            }));

  // And iterate over all added constraints to process it
  for (const auto& constraint : added_constraints)
  {
    // Skip if not a fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint constraint
    const auto velocity_angular =
        dynamic_cast<const fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint*>(&constraint);
    if (!velocity_angular)
    {
      continue;
    }

    // Check the angular velocity mean has a single value and it is the same as the wheel odometry one
    const auto& mean = velocity_angular->mean();
    ASSERT_EQ(1u, mean.size());
    EXPECT_NEAR(w, mean[0], 1.0e-2);
  }

  // Third, confirm the relative yaw orientation velocity between the current and the previous
  // fuse_variables::Orientation2DStamped variables is the same as the wheel odometry twist angular velocity along the z
  // axis

  // That is, ensure there are two fuse_variables::Orientation2DStamped variables
  EXPECT_EQ(2, std::count_if(added_variables.begin(), added_variables.end(), [](const auto& variable) -> bool {
              return dynamic_cast<const fuse_variables::Orientation2DStamped*>(&variable);
            }));

  // Iterate over all added variables to process retrieve the fuse_variables::Orientation2DStamped variables
  std::vector<const fuse_variables::Orientation2DStamped*> orientations;
  for (const auto& variable : added_variables)
  {
    // Skip if not a fuse_variables::VelocityAngular2DStamped variable
    const auto orientation = dynamic_cast<const fuse_variables::Orientation2DStamped*>(&variable);
    if (!orientation)
    {
      continue;
    }

    orientations.push_back(orientation);
  }

  // Sort them by stamp
  std::sort(orientations.begin(), orientations.end(),
            [](const auto& lhs, const auto& rhs) { return lhs->stamp() < rhs->stamp(); });

  // And check the angular velocity is the same as the wheel odometry one
  //
  // We need the time and orientation deltas in order to compute the angular velocity
  const auto dt = (orientations[1]->stamp() - orientations[0]->stamp()).toSec();
  ASSERT_LT(0.0, dt);

  const auto orientation_delta = angles::shortest_angular_distance(orientations[0]->yaw(), orientations[1]->yaw());

  EXPECT_NEAR(w, orientation_delta / dt, 1.0e-2);

  // Finally, confirm the fuse_constraints::RelativePose2DStampedConstraint constraint mean is the same as the wheel
  // odometry twist angular velocity along the z axis

  // That is, ensure there is one fuse_constraints::RelativePose2DStampedConstraint constraint
  EXPECT_EQ(1, std::count_if(added_constraints.begin(), added_constraints.end(), [](const auto& constraint) -> bool {
              return dynamic_cast<const fuse_constraints::RelativePose2DStampedConstraint*>(&constraint);
            }));

  // And iterate over all added constraints to process it
  for (const auto& constraint : added_constraints)
  {
    // Skip if not a fuse_constraints::RelativePose2DStampedConstraint constraint
    const auto relative_pose = dynamic_cast<const fuse_constraints::RelativePose2DStampedConstraint*>(&constraint);
    if (!relative_pose)
    {
      continue;
    }

    // Check the relative pose mean orientation (3rd component) is the same as the wheel odometry one, when multiplied
    // by the time delta between the current and previous position and orientation variables
    //
    // We retrieve the current and previous positions, i.e. position2 and position1, and compute the time delta between
    // them. Alternatively, we could use the time delta previously computed for the orientation variables.
    const auto& delta = relative_pose->delta();

    const auto& variables = relative_pose->variables();
    const auto& position1_uuid = variables[0];
    const auto& position2_uuid = variables[2];

    const auto position1_iter =
        std::find_if(added_variables.begin(), added_variables.end(),
                     [&position1_uuid](const auto& variable) { return variable.uuid() == position1_uuid; });
    ASSERT_NE(added_variables.end(), position1_iter);

    const auto position1 = dynamic_cast<const fuse_variables::Position2DStamped*>(&*position1_iter);
    ASSERT_TRUE(position1);

    const auto position2_iter =
        std::find_if(added_variables.begin(), added_variables.end(),
                     [&position2_uuid](const auto& variable) { return variable.uuid() == position2_uuid; });
    ASSERT_NE(added_variables.end(), position2_iter);

    const auto position2 = dynamic_cast<const fuse_variables::Position2DStamped*>(&*position2_iter);
    ASSERT_TRUE(position2);

    const auto dt = position2->stamp().toSec() - position1->stamp().toSec();
    ASSERT_LT(0.0, dt);

    ASSERT_EQ(3u, delta.size());
    EXPECT_NEAR(w, delta[2] / dt, 1.0e-2);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "imu_2d_test");
  auto spinner = ros::AsyncSpinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
