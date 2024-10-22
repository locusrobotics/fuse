/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Robotics
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
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>
#include <fuse_core/eigen.hpp>
#include <Eigen/Core>
#include <memory>
#include <random>

#include <fuse_core/node_interfaces/node_interfaces.hpp>
#include <fuse_core/util.hpp>
#include <fuse_msgs/srv/set_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

static constexpr char BASELINK_FRAME[] = "base_link";  //!< The base_link frame id used when
                                                       //!< publishing sensor data
static constexpr char MAP_FRAME[] = "map";             //!< The map frame id used when publishing ground truth
                                                       //!< data
static constexpr double IMU_SIGMA = 0.1;               //!< Std dev of simulated Imu measurement noise
static constexpr char ODOM_FRAME[] = "odom";           //!< The odom frame id used when publishing wheel
static constexpr double ODOM_POSITION_SIGMA = 0.5;     //!< Std dev of simulated odom position measurement noise
static constexpr double ODOM_ORIENTATION_SIGMA = 0.1;  //!< Std dev of simulated odom orientation measurement noise

/**
 * @brief The true pose and velocity of the robot
 */
struct Robot
{
  rclcpp::Time stamp;

  double mass;

  double x = 0;
  double y = 0;
  double z = 0;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  double vx = 0;
  double vy = 0;
  double vz = 0;
  double vroll = 0;
  double vpitch = 0;
  double vyaw = 0;
  double ax = 0;
  double ay = 0;
  double az = 0;
};

/**
 * @brief Convert the robot state into a ground truth odometry message
 */
nav_msgs::msg::Odometry::SharedPtr robotToOdometry(const Robot & state)
{
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->header.stamp = state.stamp;
  msg->header.frame_id = MAP_FRAME;
  msg->child_frame_id = BASELINK_FRAME;
  msg->pose.pose.position.x = state.x;
  msg->pose.pose.position.y = state.y;
  msg->pose.pose.position.z = state.z;

  tf2::Quaternion q;
  q.setEuler(state.yaw, state.pitch, state.roll);
  msg->pose.pose.orientation.w = q.w();
  msg->pose.pose.orientation.x = q.x();
  msg->pose.pose.orientation.y = q.y();
  msg->pose.pose.orientation.z = q.z();
  msg->twist.twist.linear.x = state.vx;
  msg->twist.twist.linear.y = state.vy;
  msg->twist.twist.linear.z = state.vz;
  msg->twist.twist.angular.x = state.vroll;
  msg->twist.twist.angular.y = state.vpitch;
  msg->twist.twist.angular.z = state.vyaw;

  // set covariances
  msg->pose.covariance[0] = 0.1;
  msg->pose.covariance[7] = 0.1;
  msg->pose.covariance[14] = 0.1;
  msg->pose.covariance[21] = 0.1;
  msg->pose.covariance[28] = 0.1;
  msg->pose.covariance[35] = 0.1;
  msg->twist.covariance[0] = 0.1;
  msg->twist.covariance[7] = 0.1;
  msg->twist.covariance[14] = 0.1;
  msg->twist.covariance[21] = 0.1;
  msg->twist.covariance[28] = 0.1;
  msg->twist.covariance[35] = 0.1;
  return msg;
}

/**
 * @brief Compute the next robot state given the current robot state and a simulated step time
 */
Robot simulateRobotMotion(
  const Robot & previous_state, const rclcpp::Time & now,
  Eigen::Vector3d external_force)
{
  auto dt = (now - previous_state.stamp).seconds();
  auto next_state = Robot();
  next_state.stamp = now;
  next_state.mass = previous_state.mass;

  // just euler integrate to get the next position and velocity
  next_state.x = previous_state.x + previous_state.vx * dt;
  next_state.y = previous_state.y + previous_state.vy * dt;
  next_state.z = previous_state.z + previous_state.vz * dt;
  next_state.vx = previous_state.vx + previous_state.ax * dt;
  next_state.vy = previous_state.vy + previous_state.ay * dt;
  next_state.vz = previous_state.vz + previous_state.az * dt;

  // let's not deal with 3D orientation dynamics for this tutorial
  next_state.roll = 0;
  next_state.pitch = 0;
  next_state.yaw = 0;
  next_state.vroll = 0;
  next_state.vpitch = 0;
  next_state.vyaw = 0;

  // get the current acceleration from the current applied force
  next_state.ax = external_force.x() / next_state.mass;
  next_state.ay = external_force.y() / next_state.mass;
  next_state.az = external_force.z() / next_state.mass;

  return next_state;
}

/**
 * @brief Create a simulated Imu measurement from the current state
 */
sensor_msgs::msg::Imu::SharedPtr simulateImu(const Robot & robot)
{
  static std::random_device rd{};
  static std::mt19937 generator{rd()};
  static std::normal_distribution<> noise{0.0, IMU_SIGMA};

  auto msg = std::make_shared<sensor_msgs::msg::Imu>();
  msg->header.stamp = robot.stamp;
  msg->header.frame_id = BASELINK_FRAME;

  // measure accel
  msg->linear_acceleration.x = robot.ax + noise(generator);
  msg->linear_acceleration.y = robot.ay + noise(generator);
  msg->linear_acceleration.z = robot.az + noise(generator);
  msg->linear_acceleration_covariance[0] = IMU_SIGMA * IMU_SIGMA;
  msg->linear_acceleration_covariance[4] = IMU_SIGMA * IMU_SIGMA;
  msg->linear_acceleration_covariance[8] = IMU_SIGMA * IMU_SIGMA;

  // Simulated IMU does not provide orientation
  msg->orientation_covariance[0] = -1;
  msg->orientation_covariance[4] = -1;
  msg->orientation_covariance[8] = -1;

  msg->angular_velocity.x = robot.vroll + noise(generator);
  msg->angular_velocity.y = robot.vpitch + noise(generator);
  msg->angular_velocity.z = robot.vyaw + noise(generator);
  msg->angular_velocity_covariance[0] = IMU_SIGMA * IMU_SIGMA;
  msg->angular_velocity_covariance[4] = IMU_SIGMA * IMU_SIGMA;
  msg->angular_velocity_covariance[8] = IMU_SIGMA * IMU_SIGMA;
  return msg;
}

nav_msgs::msg::Odometry::SharedPtr simulateOdometry(const Robot & robot)
{
  static std::random_device rd{};
  static std::mt19937 generator{rd()};
  static std::normal_distribution<> position_noise{0.0, ODOM_POSITION_SIGMA};

  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->header.stamp = robot.stamp;
  msg->header.frame_id = ODOM_FRAME;
  msg->child_frame_id = BASELINK_FRAME;

  // noisy position measurement
  msg->pose.pose.position.x = robot.x + position_noise(generator);
  msg->pose.pose.position.y = robot.y + position_noise(generator);
  msg->pose.pose.position.z = robot.z + position_noise(generator);
  msg->pose.covariance[0] = ODOM_POSITION_SIGMA * ODOM_POSITION_SIGMA;
  msg->pose.covariance[7] = ODOM_POSITION_SIGMA * ODOM_POSITION_SIGMA;
  msg->pose.covariance[14] = ODOM_POSITION_SIGMA * ODOM_POSITION_SIGMA;

  // noisy orientation measurement
  tf2::Quaternion q;
  q.setEuler(robot.yaw, robot.pitch, robot.roll);
  msg->pose.pose.orientation.w = q.w();
  msg->pose.pose.orientation.x = q.x();
  msg->pose.pose.orientation.y = q.y();
  msg->pose.pose.orientation.z = q.z();
  msg->pose.covariance[21] = ODOM_ORIENTATION_SIGMA * ODOM_ORIENTATION_SIGMA;
  msg->pose.covariance[28] = ODOM_ORIENTATION_SIGMA * ODOM_ORIENTATION_SIGMA;
  msg->pose.covariance[35] = ODOM_ORIENTATION_SIGMA * ODOM_ORIENTATION_SIGMA;
  return msg;
}

void initializeStateEstimation(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const Robot & state, const rclcpp::Clock::SharedPtr & clock, const rclcpp::Logger & logger)
{
  // Send the initial localization signal to the state estimator
  auto srv = std::make_shared<fuse_msgs::srv::SetPose::Request>();
  srv->pose.header.frame_id = MAP_FRAME;
  srv->pose.pose.pose.position.x = state.x;
  srv->pose.pose.pose.position.y = state.y;
  srv->pose.pose.pose.position.z = state.z;
  tf2::Quaternion q;
  q.setEuler(state.yaw, state.pitch, state.roll);
  srv->pose.pose.pose.orientation.w = q.w();
  srv->pose.pose.pose.orientation.x = q.x();
  srv->pose.pose.pose.orientation.y = q.y();
  srv->pose.pose.pose.orientation.z = q.z();
  srv->pose.pose.covariance[0] = 1.0;
  srv->pose.pose.covariance[7] = 1.0;
  srv->pose.pose.covariance[14] = 1.0;
  srv->pose.pose.covariance[21] = 1.0;
  srv->pose.pose.covariance[28] = 1.0;
  srv->pose.pose.covariance[35] = 1.0;

  auto client = rclcpp::create_client<fuse_msgs::srv::SetPose>(
    interfaces.get_node_base_interface(), interfaces.get_node_graph_interface(),
    interfaces.get_node_services_interface(), "/state_estimation/set_pose_service",
    rclcpp::ServicesQoS());

  while (!client->wait_for_service(std::chrono::seconds(30)) &&
    interfaces.get_node_base_interface()->get_context()->is_valid())
  {
    RCLCPP_WARN_STREAM(
      logger,
      "Waiting for '" << client->get_service_name() << "' service to become available.");
  }

  auto success = false;
  while (!success) {
    clock->sleep_for(std::chrono::milliseconds(100));
    srv->pose.header.stamp = clock->now();
    auto result_future = client->async_send_request(srv);

    if (rclcpp::spin_until_future_complete(
        interfaces.get_node_base_interface(), result_future,
        std::chrono::seconds(1)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(logger, "service call failed :(");
      client->remove_pending_request(result_future);
      return;
    }
    success = result_future.get()->success;
  }
}

int main(int argc, char ** argv)
{
  // set up our ROS node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("three_dimensional_simulator");

  // create our sensor publishers
  auto imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
  auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

  // create the ground truth publisher
  auto ground_truth_publisher = node->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);

  // Initialize the robot state (state variables are zero-initialized)
  auto state = Robot();
  state.stamp = node->now();
  state.mass = 10;  // kg

  // you can modify the rate at which this loop runs to see the different performance of the estimator and the effect of
  // integration inaccuracy on the ground truth
  auto rate = rclcpp::Rate(100.0);

  // normally we would have to initialize the state estimation, but we included an ignition 'sensor' in our config,
  // which takes care of that.

  // parameters that control the motion pattern of the robot
  double const motion_duration = 5;  // length of time to oscillate on a given axis, in seconds
  double const N_cycles = 2;         // number of oscillations per `motion_duration`

  while (rclcpp::ok()) {
    // store the first time this runs (since it won't start running exactly at a multiple of `motion_duration`)
    static auto const first_time = node->now();
    auto const now = node->now();

    // compensate for the original time offset
    double now_d = (now - first_time).seconds();
    // store how long it has been (resetting at `motion_duration` seconds)
    double mod_time = std::fmod(now_d, motion_duration);

    // apply a harmonic force (oscillates `N_cycles` times per `motion_duration`)
    double const force_magnitude = 100 * std::cos(2 * M_PI * N_cycles * mod_time / motion_duration);
    Eigen::Vector3d external_force = {0, 0, 0};

    // switch oscillation axes every `motion_duration` seconds (with one 'rest period')
    if (std::fmod(now_d, 4 * motion_duration) < motion_duration) {
      external_force.x() = force_magnitude;
    } else if (std::fmod(now_d, 4 * motion_duration) < 2 * motion_duration) {
      external_force.y() = force_magnitude;
    } else if (std::fmod(now_d, 4 * motion_duration) < 3 * motion_duration) {
      external_force.z() = force_magnitude;
    } else {
      // reset the robot's position and velocity, leave the external force as 0
      // we do this so the ground truth doesn't drift (due to inaccuracy from euler integration)
      state.x = 0;
      state.y = 0;
      state.z = 0;
      state.vx = 0;
      state.vy = 0;
      state.vz = 0;
    }

    // Simulate the robot motion
    auto new_state = simulateRobotMotion(state, now, external_force);

    // Publish the new ground truth
    ground_truth_publisher->publish(*robotToOdometry(new_state));

    // Generate and publish simulated measurements from the new robot state
    imu_publisher->publish(*simulateImu(new_state));
    odom_publisher->publish(*simulateOdometry(new_state));

    // Wait for the next time step
    state = new_state;
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
