/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Locus Robotics
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
#include <fuse_core/util.hpp>
<<<<<<< HEAD
#include <fuse_msgs/srv/set_pose.hpp>
#include <nav_msgs/Odometry.h>
=======
#include <fuse_models/SetPose.h>
#include <nav_msgs/msg/odometry.hpp>
>>>>>>> eb51957... msg changes
#include <ros/ros.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


#include <cmath>
#include <memory>
#include <random>
#include <vector>

static constexpr double SITE_WIDTH = 100.0;  //!< The width/length of the test area in meters
static constexpr double BEACON_SPACING = 20.0;  //!< The distance between each range beacon
static constexpr double BEACON_SIGMA = 4.0;  //!< Std dev used to create the database of noisy beacon positions
static constexpr double ROBOT_PATH_RADIUS = 0.35 * SITE_WIDTH;  //!< The radius of the simulated robot's path
static constexpr double ROBOT_VELOCITY = 10.0;  //!< The forward velocity of our simulated robot
static constexpr char BASELINK_FRAME[] = "base_link";  //!< The base_link frame id used when publishing sensor data
static constexpr char ODOM_FRAME[] = "odom";  //!< The odom frame id used when publishing wheel odometry data
static constexpr char MAP_FRAME[] = "map";  //!< The map frame id used when publishing ground truth data
static constexpr double IMU_SIGMA = 0.1;  //!< Std dev of simulated Imu measurement noise
static constexpr double ODOM_VX_SIGMA = 0.5;  //!< Std dev of simulated Odometry linear velocity measurement noise
static constexpr double ODOM_VYAW_SIGMA = 0.5;  //!< Std dev of simulated Odometry angular velocity measurement noise
static constexpr double RANGE_SIGMA = 0.5;  //!< Std dev of simulated beacon range measurement noise

/**
 * @brief The position of a "range beacon"
 */
struct Beacon
{
  double x;
  double y;
};

/**
 * @brief The true pose and velocity of the robot
 */
struct Robot
{
  rclcpp::Time stamp;
  double x;
  double y;
  double yaw;
  double vx;
  double vy;
  double vyaw;
};

/**
 * @brief Create the set of range beacons to simulate
 */
std::vector<Beacon> createBeacons()
{
  auto beacons = std::vector<Beacon>();
  for (auto x = -SITE_WIDTH / 2; x <= SITE_WIDTH / 2; x += BEACON_SPACING)
  {
    for (auto y = -SITE_WIDTH / 2; y <= SITE_WIDTH / 2; y += BEACON_SPACING)
    {
      beacons.push_back({x, y});  // NOLINT[whitespace/braces]
    }
  }
  return beacons;
}

/**
 * @brief Create a noisy set of beacon priors from the true set of beacons
 */
std::vector<Beacon> createNoisyBeacons(const std::vector<Beacon>& beacons)
{
  static std::random_device rd{};
  static std::mt19937 generator{rd()};
  static std::normal_distribution<> noise{0.0, BEACON_SIGMA};

  auto noisy_beacons = std::vector<Beacon>();
  for (const auto& beacon : beacons)
  {
    noisy_beacons.push_back({beacon.x + noise(generator), beacon.y + noise(generator)});  // NOLINT[whitespace/braces]
  }
  return noisy_beacons;
}

/**
 * @brief Convert the set of beacons into a pointcloud for visualization purposes
 */
sensor_msgs::msg::PointCloud2 beaconsToPointcloud(
  const std::vector<Beacon>& beacons,
  const rclcpp::Clock& clock)
{
  auto msg = sensor_msgs::msg::PointCloud2::SharedPtr();
  msg->header.stamp = clock.now();

  msg->header.frame_id = MAP_FRAME;
  sensor_msgs::msg::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "sigma", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "id", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(beacons.size());
  sensor_msgs::msg::PointCloud2Iterator<float> x_it(*msg, "x");
  sensor_msgs::msg::PointCloud2Iterator<float> y_it(*msg, "y");
  sensor_msgs::msg::PointCloud2Iterator<float> z_it(*msg, "z");
  sensor_msgs::msg::PointCloud2Iterator<float> sigma_it(*msg, "sigma");
  sensor_msgs::msg::PointCloud2Iterator<unsigned int> id_it(*msg, "id");
  for (auto id = 0u; id < beacons.size(); ++id)
  {
    // Compute the distance to each beacon
    const auto& beacon = beacons.at(id);
    *x_it = static_cast<float>(beacon.x);
    *y_it = static_cast<float>(beacon.y);
    *z_it = 10.0f;
    *sigma_it = static_cast<float>(BEACON_SIGMA);
    *id_it = id;
    ++x_it; ++y_it; ++z_it; ++sigma_it, ++id_it;
  }
  return sensor_msgs::msg::PointCloud2(msg);
}

/**
 * @brief Convert the robot state into a ground truth odometry message
 */
nav_msgs::msg::Odometry robotToOdometry(const Robot& state)
{
  auto msg = nav_msgs::msg::Odometry::SharedPtr();
  msg->header.stamp = state.stamp;
  msg->header.frame_id = MAP_FRAME;
  msg->child_frame_id = BASELINK_FRAME;
  msg->pose.pose.position.x = state.x;
  msg->pose.pose.position.y = state.y;
  msg->pose.pose.position.z = 10.0;
  msg->pose.pose.orientation.w = std::cos(state.yaw / 2);
  msg->pose.pose.orientation.x = 0.0;
  msg->pose.pose.orientation.y = 0.0;
  msg->pose.pose.orientation.z = std::sin(state.yaw / 2);
  msg->pose.covariance[0] = 0.1;
  msg->pose.covariance[7] = 0.1;
  msg->pose.covariance[14] = 0.1;
  msg->pose.covariance[21] = 0.1;
  msg->pose.covariance[28] = 0.1;
  msg->pose.covariance[35] = 0.1;
  msg->twist.twist.linear.x = state.vx;
  msg->twist.twist.linear.y = state.vy;
  msg->twist.twist.linear.z = 0.0;
  msg->twist.twist.angular.x = 0.0;
  msg->twist.twist.angular.y = 0.0;
  msg->twist.twist.angular.z = state.vyaw;
  msg->twist.covariance[0] = 0.1;
  msg->twist.covariance[7] = 0.1;
  msg->twist.covariance[14] = 0.1;
  msg->twist.covariance[21] = 0.1;
  msg->twist.covariance[28] = 0.1;
  msg->twist.covariance[35] = 0.1;
  return nav_msgs::msg::Odometry(msg);
}

/**
 * @brief Send the starting robot pose to the state estimator
 *
 * The state estimator will not run until it has been sent a starting pose.
 */
void initializeStateEstimation(
  const Robot& state, const rclcpp::Clock& clock)
{
  // Send the initial localization signal to the state estimator
  auto srv = fuse_msgs::srv::SetPose();
  srv.request.pose.header.frame_id = MAP_FRAME;
  srv.request.pose.pose.pose.position.x = state.x;
  srv.request.pose.pose.pose.position.y = state.y;
  srv.request.pose.pose.pose.position.z = 0.0;
  srv.request.pose.pose.pose.orientation.w = std::cos(state.yaw / 2);
  srv.request.pose.pose.pose.orientation.x = 0.0;
  srv.request.pose.pose.pose.orientation.y = 0.0;
  srv.request.pose.pose.pose.orientation.z = std::sin(state.yaw / 2);
  srv.request.pose.pose.covariance[0] = 1.0;
  srv.request.pose.pose.covariance[7] = 1.0;
  srv.request.pose.pose.covariance[14] = 1.0;
  srv.request.pose.pose.covariance[21] = 1.0;
  srv.request.pose.pose.covariance[28] = 1.0;
  srv.request.pose.pose.covariance[35] = 1.0;
  ros::service::waitForService("/state_estimation/set_pose", rclcpp::Duration::from_seconds(30.0));
  auto success = false;
  while (!success)
  {
    rclcpp::Duration::from_seconds(0.1).sleep();
    srv.request.pose.header.stamp = clock.now();
    ros::service::call("/state_estimation/set_pose", srv);
    success = srv.response.success;
  }
}

/**
 * @brief Compute the next robot state given the current robot state and a simulated step time
 */
Robot simulateRobotMotion(const Robot& previous_state, const rclcpp::Time& now)
{
  auto dt = (now - previous_state.stamp).seconds();
  auto theta = std::atan2(previous_state.y, previous_state.x) + (dt * previous_state.vyaw);
  auto next_state = Robot();
  next_state.stamp = now;
  next_state.x = ROBOT_PATH_RADIUS * std::cos(theta);
  next_state.y = ROBOT_PATH_RADIUS * std::sin(theta);
  next_state.yaw = fuse_core::wrapAngle2D(theta + (M_PI / 2));
  next_state.vx = previous_state.vx;
  next_state.vy = 0.0;
  next_state.vyaw = previous_state.vyaw;
  return next_state;
}

/**
 * @brief Create a simulated Imu measurement from the current state
 */
sensor_msgs::msg::Imu simulateImu(const Robot& robot)
{
  static std::random_device rd{};
  static std::mt19937 generator{rd()};
  static std::normal_distribution<> noise{0.0, IMU_SIGMA};

  auto msg = sensor_msgs::msg::Imu::SharedPtr();
  msg->header.stamp = robot.stamp;
  msg->header.frame_id = BASELINK_FRAME;
  msg->orientation_covariance[0] = -1;  // Simulated IMU does not provide orientation
  msg->angular_velocity.z = robot.vyaw + noise(generator);
  msg->angular_velocity_covariance[8] = IMU_SIGMA * IMU_SIGMA;
  msg->linear_acceleration_covariance[0] = -1;  // Simulated IMU does not provide acceleration
  return sensor_msgs::msg::Imu(msg);
}

/**
 * @brief Create a simulated Odometry measurement from the current state
 */
nav_msgs::msg::Odometry simulateWheelOdometry(const Robot& robot)
{
  static std::random_device rd{};
  static std::mt19937 generator{rd()};
  static std::normal_distribution<> vx_noise{0.0, ODOM_VX_SIGMA};
  static std::normal_distribution<> vyaw_noise{0.0, ODOM_VYAW_SIGMA};

  auto msg = nav_msgs::msg::Odometry::SharedPtr();
  msg->header.stamp = robot.stamp;
  msg->header.frame_id = ODOM_FRAME;
  msg->child_frame_id = BASELINK_FRAME;
  msg->twist.twist.linear.x = robot.vx + vx_noise(generator);
  msg->twist.twist.linear.y = 0.0;
  msg->twist.twist.angular.z = robot.vyaw + vyaw_noise(generator);
  msg->twist.covariance[0] = ODOM_VX_SIGMA * ODOM_VX_SIGMA;
  msg->twist.covariance[7] = ODOM_VX_SIGMA * ODOM_VX_SIGMA;
  msg->twist.covariance[35] = ODOM_VYAW_SIGMA * ODOM_VYAW_SIGMA;
  return nav_msgs::msg::Odometry(msg);
}

sensor_msgs::msg::PointCloud2 simulateRangeSensor(const Robot& robot, const std::vector<Beacon>& beacons)
{
  static std::random_device rd{};
  static std::mt19937 generator{rd()};
  static std::normal_distribution<> noise{0.0, RANGE_SIGMA};

  auto msg = sensor_msgs::msg::PointCloud2::SharedPtr();
  msg->header.stamp = robot.stamp;
  msg->header.frame_id = BASELINK_FRAME;

  // Configure the pointcloud to have the following fields: id, range, sigma
  sensor_msgs::msg::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2Fields(3, "id", 1, sensor_msgs::msg::PointField::UINT32,
                                   "range", 1, sensor_msgs::msg::PointField::FLOAT64,
                                   "sigma", 1, sensor_msgs::msg::PointField::FLOAT64);

  // Generate the simulated range to each known beacon
  modifier.resize(beacons.size());
  sensor_msgs::msg::PointCloud2Iterator<unsigned int> id_it(*msg, "id");
  sensor_msgs::msg::PointCloud2Iterator<double> range_it(*msg, "range");
  sensor_msgs::msg::PointCloud2Iterator<double> sigma_it(*msg, "sigma");
  for (auto id = 0u; id < beacons.size(); ++id)
  {
    // Compute the distance to each beacon
    const auto& beacon = beacons.at(id);
    auto dx = robot.x - beacon.x;
    auto dy = robot.y - beacon.y;
    auto range = std::sqrt(dx * dx + dy * dy) + noise(generator);
    // Insert the beacon measurement into the pointcloud
    *id_it = id;
    *range_it = range;
    *sigma_it = RANGE_SIGMA;
    ++id_it; ++range_it; ++sigma_it;
  }
  return sensor_msgs::msg::PointCloud2(msg);
}

/**
 * @brief Simulate a robot traveling in a circular path with wheel encoders, Imu, and a range sensor.
 *
 * The following simulated sensor topics are published:
 *  - The wheel encoders measure forward and rotational velocity and publish nav_msgs::msg::Odometry messages on the
 *    /wheel_odom topic at 10Hz.
 *  - The Imu measures z-axis rotational velocity and publishes sensor_msgs::msg::Imu messages on the /imu topic at 10Hz.
 *  - The "range sensor" publishes special sensor_msgs::msg::PointCloud2 messages on the /ranges topic. The PointCloud2
 *    message contains three channels (id, range, sigma). The id field contains the unique ID of the range beacon
 *    being measured. The range field contains the measured distance between the robot and the range beacon in
 *    meters. The sigma field contains the range measurement uncertainty (standard deviation) in meters.
 *  - A prior known database of noisy beacon positions are published on the /prior_beacons latched topic as a
 *    sensor_msgs::msg::PointCloud2 with the following fields: (x, y, z, sigma, id)
 *
 * In addition to the simulated sensors, the following ground truth topics are published:
 *  - The true position of each beacon is published as a sensor_msgs::msg::PointCloud2 (x, y, z, sigma, id) topic on the
 *    latched topic /true_beacons
 *  - The true position and velocity of the robot is published as a nav_msgs::msg::Odometry message on the /ground_truth
 *    topic at 10Hz
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "range_sensor_simulator");

  // TODO(CH3): Make this an rclcpp node!
  ros::NodeHandle node_handle;
  ros::Publisher imu_publisher = node_handle.advertise<sensor_msgs::msg::Imu>("imu", 1);
  ros::Publisher true_beacons_publisher = node_handle.advertise<sensor_msgs::msg::PointCloud2>("true_beacons", 1, true);
  ros::Publisher prior_beacons_publisher = node_handle.advertise<sensor_msgs::msg::PointCloud2>("prior_beacons", 1, true);
  ros::Publisher wheel_odom_publisher = node_handle.advertise<nav_msgs::msg::Odometry>("wheel_odom", 1);
  ros::Publisher ground_truth_publisher = node_handle.advertise<nav_msgs::msg::Odometry>("ground_truth", 1);
  ros::Publisher range_publisher = node_handle.advertise<sensor_msgs::msg::PointCloud2>("ranges", 1);

  // Create the true set of range beacons
  auto beacons = createBeacons();
  true_beacons_publisher->publish(beaconsToPointcloud(beacons), *node->get_clock());

  // Publish a set of noisy beacon locations to act as the known priors
  auto noisy_beacons = createNoisyBeacons(beacons);
  prior_beacons_publisher->publish(beaconsToPointcloud(noisy_beacons, *node->get_clock());

  // Initialize the robot state
  auto state = Robot();
  state.stamp = node->now();
  state.x = ROBOT_PATH_RADIUS;
  state.y = 0.0;
  state.yaw = M_PI / 2;
  state.vx = ROBOT_VELOCITY;
  state.vy = 0.0;
  state.vyaw = state.vx / ROBOT_PATH_RADIUS;

  // Send the initial localization pose to the state estimator
  initializeStateEstimation(state, *node->get_clock());

  // Simulate the robot traveling in a circular path
  auto rate = rclcpp::Rate(10.0);
  while (ros::ok())
  {
    // Simulate the robot motion
    auto new_state = simulateRobotMotion(state, node->now());
    // Publish the new ground truth
    ground_truth_publisher->publish(robotToOdometry(new_state));
    // Generate and publish simulated measurements from the new robot state
    imu_publisher->publish(simulateImu(new_state));
    wheel_odom_publisher->publish(simulateWheelOdometry(new_state));
    range_publisher->publish(simulateRangeSensor(new_state, beacons));
    // Wait for the next time step
    state = new_state;
    rate.sleep();
  }

  return 0;
}
