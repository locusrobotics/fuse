/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Giacomo Franchini
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_models/common/sensor_proc.hpp>
#include <fuse_models/odometry_3d_publisher.hpp>
#include <fuse_models/omnidirectional_3d_predict.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_2d/tf2_2d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

// #include "covariance_geometry_ros/covariance_geometry_ros.hpp"
// #include "covariance_geometry_ros/utils.hpp"

// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Odometry3DPublisher, fuse_core::Publisher)

namespace fuse_models
{

Odometry3DPublisher::Odometry3DPublisher()
: fuse_core::AsyncPublisher(1),
  device_id_(fuse_core::uuid::NIL),
  logger_(rclcpp::get_logger("uninitialized")),
  latest_stamp_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
  latest_covariance_stamp_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
}

void Odometry3DPublisher::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name)
{
  interfaces_ = interfaces;
  fuse_core::AsyncPublisher::initialize(interfaces, name);
}

void Odometry3DPublisher::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(interfaces_);

  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(interfaces_);

  params_.loadFromROS(interfaces_, name_);

  if (!params_.invert_tf && params_.world_frame_id == params_.map_frame_id) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
      clock_,
      params_.tf_cache_time.to_chrono<std::chrono::nanoseconds>()
      // , interfaces_  // NOTE(methylDragon): This one is pending a change on tf2_ros/buffer.h
      // TODO(methylDragon): See above ^
    );

    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_,
      interfaces_.get_node_base_interface(),
      interfaces_.get_node_logging_interface(),
      interfaces_.get_node_parameters_interface(),
      interfaces_.get_node_topics_interface());
  }

  // Advertise the topics
  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = cb_group_;

  odom_pub_ = rclcpp::create_publisher<nav_msgs::msg::Odometry>(
    interfaces_,
    params_.topic,
    params_.queue_size,
    pub_options);
  acceleration_pub_ = rclcpp::create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    interfaces_,
    params_.acceleration_topic,
    params_.queue_size,
    pub_options);
}

void Odometry3DPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  // Find the most recent common timestamp
  const auto latest_stamp = synchronizer_.findLatestCommonStamp(*transaction, *graph);
  if (0u == latest_stamp.nanoseconds()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_stamp_ = latest_stamp;
    }

    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 10.0 * 1000,
      "Failed to find a matching set of state variables with device id '"
        << device_id_ << "'.");
    return;
  }

  // Get the pose values associated with the selected timestamp
  fuse_core::UUID position_uuid;
  fuse_core::UUID orientation_uuid;
  fuse_core::UUID velocity_linear_uuid;
  fuse_core::UUID velocity_angular_uuid;
  fuse_core::UUID acceleration_linear_uuid;

  nav_msgs::msg::Odometry odom_output;
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration_output;
  if (!getState(
      *graph,
      latest_stamp,
      device_id_,
      position_uuid,
      orientation_uuid,
      velocity_linear_uuid,
      velocity_angular_uuid,
      acceleration_linear_uuid,
      odom_output,
      acceleration_output))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_stamp_ = latest_stamp;
    return;
  }

  odom_output.header.frame_id = params_.world_frame_id;
  odom_output.header.stamp = latest_stamp;
  odom_output.child_frame_id = params_.base_link_output_frame_id;

  acceleration_output.header.frame_id = params_.base_link_output_frame_id;
  acceleration_output.header.stamp = latest_stamp;

  // Don't waste CPU computing the covariance if nobody is listening
  rclcpp::Time latest_covariance_stamp = latest_covariance_stamp_;
  bool latest_covariance_valid = latest_covariance_valid_;
  if (odom_pub_->get_subscription_count() > 0 || acceleration_pub_->get_subscription_count() > 0) {
    // Throttle covariance computation
    if (params_.covariance_throttle_period.nanoseconds() == 0 ||
      latest_stamp - latest_covariance_stamp > params_.covariance_throttle_period)
    {
      latest_covariance_stamp = latest_stamp;

      try {
        std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>> covariance_requests;
        covariance_requests.emplace_back(position_uuid, position_uuid);
        covariance_requests.emplace_back(position_uuid, orientation_uuid);
        covariance_requests.emplace_back(orientation_uuid, orientation_uuid);
        covariance_requests.emplace_back(velocity_linear_uuid, velocity_linear_uuid);
        covariance_requests.emplace_back(velocity_linear_uuid, velocity_angular_uuid);
        covariance_requests.emplace_back(velocity_angular_uuid, velocity_angular_uuid);
        covariance_requests.emplace_back(acceleration_linear_uuid, acceleration_linear_uuid);

        std::vector<std::vector<double>> covariance_matrices;
        graph->getCovariance(covariance_requests, covariance_matrices, params_.covariance_options, true);

        odom_output.pose.covariance[0] = covariance_matrices[0][0]; // cov(x, x)
        odom_output.pose.covariance[1] = covariance_matrices[0][1]; // cov(x, y)
        odom_output.pose.covariance[2] = covariance_matrices[0][2]; // cov(x, z)
        odom_output.pose.covariance[3] = covariance_matrices[1][0]; // cov(x, roll)
        odom_output.pose.covariance[4] = covariance_matrices[1][1]; // cov(x, pitch)
        odom_output.pose.covariance[5] = covariance_matrices[1][2]; // cov(x, yaw)
        
        odom_output.pose.covariance[6] = covariance_matrices[0][3]; // cov(y, x)
        odom_output.pose.covariance[7] = covariance_matrices[0][4]; // cov(y, y)
        odom_output.pose.covariance[8] = covariance_matrices[0][5]; // cov(y, z)
        odom_output.pose.covariance[9] = covariance_matrices[1][3]; // cov(y, roll)
        odom_output.pose.covariance[10] = covariance_matrices[1][4]; // cov(y, pitch)
        odom_output.pose.covariance[11] = covariance_matrices[1][5]; // cov(y, yaw)

        odom_output.pose.covariance[12] = covariance_matrices[0][6]; // cov(z, x)
        odom_output.pose.covariance[13] = covariance_matrices[0][7]; // cov(z, y)
        odom_output.pose.covariance[14] = covariance_matrices[0][8]; // cov(z, z)
        odom_output.pose.covariance[15] = covariance_matrices[1][6]; // cov(z, roll)
        odom_output.pose.covariance[16] = covariance_matrices[1][7]; // cov(z, pitch)
        odom_output.pose.covariance[17] = covariance_matrices[1][8]; // cov(z, yaw)

        odom_output.pose.covariance[18] = covariance_matrices[1][0]; // cov(roll, x)
        odom_output.pose.covariance[19] = covariance_matrices[1][3]; // cov(roll, y)
        odom_output.pose.covariance[20] = covariance_matrices[1][6]; // cov(roll, z)
        odom_output.pose.covariance[21] = covariance_matrices[2][0]; // cov(roll, roll)
        odom_output.pose.covariance[22] = covariance_matrices[2][1]; // cov(roll, pitch)
        odom_output.pose.covariance[23] = covariance_matrices[2][2]; // cov(roll, yaw)

        odom_output.pose.covariance[24] = covariance_matrices[1][1]; // cov(pitch, x)
        odom_output.pose.covariance[25] = covariance_matrices[1][4]; // cov(pitch, y)
        odom_output.pose.covariance[26] = covariance_matrices[1][7]; // cov(pitch, z)
        odom_output.pose.covariance[27] = covariance_matrices[2][1]; // cov(pitch, roll)
        odom_output.pose.covariance[28] = covariance_matrices[2][4]; // cov(pitch, pitch)
        odom_output.pose.covariance[29] = covariance_matrices[2][5]; // cov(pitch, yaw)

        odom_output.pose.covariance[30] = covariance_matrices[1][2]; // cov(yaw, x)
        odom_output.pose.covariance[31] = covariance_matrices[1][5]; // cov(yaw, y)
        odom_output.pose.covariance[32] = covariance_matrices[1][8]; // cov(yaw, z)
        odom_output.pose.covariance[33] = covariance_matrices[2][2]; // cov(yaw, roll)
        odom_output.pose.covariance[34] = covariance_matrices[2][5]; // cov(yaw, pitch)
        odom_output.pose.covariance[35] = covariance_matrices[2][8]; // cov(yaw, yaw)

        odom_output.twist.covariance[0] = covariance_matrices[3][0];
        odom_output.twist.covariance[1] = covariance_matrices[3][1];
        odom_output.twist.covariance[2] = covariance_matrices[3][2];
        odom_output.twist.covariance[3] = covariance_matrices[4][0];
        odom_output.twist.covariance[4] = covariance_matrices[4][1];
        odom_output.twist.covariance[5] = covariance_matrices[4][2];

        odom_output.twist.covariance[6] = covariance_matrices[3][3];
        odom_output.twist.covariance[7] = covariance_matrices[3][4];
        odom_output.twist.covariance[8] = covariance_matrices[3][5];
        odom_output.twist.covariance[9] = covariance_matrices[4][3];
        odom_output.twist.covariance[10] = covariance_matrices[4][4];
        odom_output.twist.covariance[11] = covariance_matrices[4][5];

        odom_output.twist.covariance[12] = covariance_matrices[3][6];
        odom_output.twist.covariance[13] = covariance_matrices[3][7];
        odom_output.twist.covariance[14] = covariance_matrices[3][8];
        odom_output.twist.covariance[15] = covariance_matrices[4][6];
        odom_output.twist.covariance[16] = covariance_matrices[4][7];
        odom_output.twist.covariance[17] = covariance_matrices[4][8];
        
        odom_output.twist.covariance[18] = covariance_matrices[4][0];
        odom_output.twist.covariance[19] = covariance_matrices[4][3];
        odom_output.twist.covariance[20] = covariance_matrices[4][6];
        odom_output.twist.covariance[21] = covariance_matrices[5][0];
        odom_output.twist.covariance[22] = covariance_matrices[5][1];
        odom_output.twist.covariance[23] = covariance_matrices[5][2];

        odom_output.twist.covariance[24] = covariance_matrices[4][1];
        odom_output.twist.covariance[25] = covariance_matrices[4][4];
        odom_output.twist.covariance[26] = covariance_matrices[4][7];
        odom_output.twist.covariance[27] = covariance_matrices[5][1];
        odom_output.twist.covariance[28] = covariance_matrices[5][4];
        odom_output.twist.covariance[29] = covariance_matrices[5][5];

        odom_output.twist.covariance[30] = covariance_matrices[4][2];
        odom_output.twist.covariance[31] = covariance_matrices[4][5];
        odom_output.twist.covariance[32] = covariance_matrices[4][8];
        odom_output.twist.covariance[33] = covariance_matrices[5][2];
        odom_output.twist.covariance[34] = covariance_matrices[5][5];
        odom_output.twist.covariance[35] = covariance_matrices[5][8];
        
        acceleration_output.accel.covariance[0] = covariance_matrices[6][0];
        acceleration_output.accel.covariance[1] = covariance_matrices[6][1];
        acceleration_output.accel.covariance[2] = covariance_matrices[6][2];
        acceleration_output.accel.covariance[6] = covariance_matrices[6][3];
        acceleration_output.accel.covariance[7] = covariance_matrices[6][4];
        acceleration_output.accel.covariance[8] = covariance_matrices[6][5];
        acceleration_output.accel.covariance[12] = covariance_matrices[6][6];
        acceleration_output.accel.covariance[13] = covariance_matrices[6][7];
        acceleration_output.accel.covariance[14] = covariance_matrices[6][8];

        // test if covariances are symmetric
        Eigen::Map<fuse_core::Matrix6d> odom_cov_map(odom_output.pose.covariance.data());
        if (!odom_cov_map.isApprox(odom_cov_map.transpose())) {
          throw std::runtime_error("Odometry covariance matrix is not symmetric");
        }

        Eigen::Map<fuse_core::Matrix6d> twist_cov_map(odom_output.twist.covariance.data());
        if (!twist_cov_map.isApprox(twist_cov_map.transpose())) {
          throw std::runtime_error("Twist covariance matrix is not symmetric");
        }

        Eigen::Map<fuse_core::Matrix6d> accel_cov_map(acceleration_output.accel.covariance.data());
        if (!accel_cov_map.isApprox(accel_cov_map.transpose())) {
          throw std::runtime_error("Acceleration covariance matrix is not symmetric");
        }

        latest_covariance_valid = true;
      } catch (const std::exception & e) {
        RCLCPP_WARN_STREAM(
          logger_,
          "An error occurred computing the covariance information for "
            << latest_stamp.nanoseconds()
            << ". The covariance will be set to zero.\n"
            << e.what());
        std::fill(odom_output.pose.covariance.begin(), odom_output.pose.covariance.end(), 0.0);
        std::fill(odom_output.twist.covariance.begin(), odom_output.twist.covariance.end(), 0.0);
        std::fill(
          acceleration_output.accel.covariance.begin(),
          acceleration_output.accel.covariance.end(), 0.0);

        latest_covariance_valid = false;
      }
    } else {
      // This covariance computation cycle has been skipped, so simply take the last covariance
      // computed
      //
      // We do not propagate the latest covariance forward because it would grow unbounded being
      // very different from the actual covariance we would have computed if not throttling.
      odom_output.pose.covariance = odom_output_.pose.covariance;
      odom_output.twist.covariance = odom_output_.twist.covariance;
      acceleration_output.accel.covariance = acceleration_output_.accel.covariance;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    latest_stamp_ = latest_stamp;
    latest_covariance_stamp_ = latest_covariance_stamp;
    latest_covariance_valid_ = latest_covariance_valid;
    odom_output_ = odom_output;
    acceleration_output_ = acceleration_output;
  }
}

void Odometry3DPublisher::onStart()
{
  synchronizer_ = Synchronizer(device_id_);
  latest_stamp_ = latest_covariance_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latest_covariance_valid_ = false;
  odom_output_ = nav_msgs::msg::Odometry();
  acceleration_output_ = geometry_msgs::msg::AccelWithCovarianceStamped();

  // TODO(CH3): Add this to a separate callback group for async behavior
  publish_timer_ = rclcpp::create_timer(
    interfaces_,
    clock_,
    std::chrono::duration<double>(1.0 / params_.publish_frequency),
    std::move(std::bind(&Odometry3DPublisher::publishTimerCallback, this)),
    cb_group_
  );

  delayed_throttle_filter_.reset();
}

void Odometry3DPublisher::onStop()
{
  publish_timer_->cancel();
}

bool Odometry3DPublisher::getState(
  const fuse_core::Graph & graph,
  const rclcpp::Time & stamp,
  const fuse_core::UUID & device_id,
  fuse_core::UUID & position_uuid,
  fuse_core::UUID & orientation_uuid,
  fuse_core::UUID & velocity_linear_uuid,
  fuse_core::UUID & velocity_angular_uuid,
  fuse_core::UUID & acceleration_linear_uuid,
  nav_msgs::msg::Odometry & odometry,
  geometry_msgs::msg::AccelWithCovarianceStamped & acceleration)
{
  try {
    position_uuid = fuse_variables::Position3DStamped(stamp, device_id).uuid();
    auto position_variable = dynamic_cast<const fuse_variables::Position3DStamped &>(
      graph.getVariable(position_uuid));

    orientation_uuid = fuse_variables::Orientation3DStamped(stamp, device_id).uuid();
    auto orientation_variable = dynamic_cast<const fuse_variables::Orientation3DStamped &>(
      graph.getVariable(orientation_uuid));

    velocity_linear_uuid = fuse_variables::VelocityLinear3DStamped(stamp, device_id).uuid();
    auto velocity_linear_variable = dynamic_cast<const fuse_variables::VelocityLinear3DStamped &>(
      graph.getVariable(velocity_linear_uuid));

    velocity_angular_uuid = fuse_variables::VelocityAngular3DStamped(stamp, device_id).uuid();
    auto velocity_angular_variable = dynamic_cast<const fuse_variables::VelocityAngular3DStamped &>(
      graph.getVariable(velocity_angular_uuid));

    acceleration_linear_uuid = fuse_variables::AccelerationLinear3DStamped(stamp, device_id).uuid();
    auto acceleration_linear_variable =
      dynamic_cast<const fuse_variables::AccelerationLinear3DStamped &>(
      graph.getVariable(acceleration_linear_uuid));
    
    odometry.pose.pose.position.x = position_variable.x();
    odometry.pose.pose.position.y = position_variable.y();
    odometry.pose.pose.position.z = position_variable.z();
    odometry.pose.pose.orientation.w = orientation_variable.w();
    odometry.pose.pose.orientation.x = orientation_variable.x();
    odometry.pose.pose.orientation.y = orientation_variable.y();
    odometry.pose.pose.orientation.z = orientation_variable.z();
    odometry.twist.twist.linear.x = velocity_linear_variable.x();
    odometry.twist.twist.linear.y = velocity_linear_variable.y();
    odometry.twist.twist.linear.z = velocity_linear_variable.z();
    odometry.twist.twist.angular.x = velocity_angular_variable.roll();
    odometry.twist.twist.angular.y = velocity_angular_variable.pitch();
    odometry.twist.twist.angular.z = velocity_angular_variable.yaw();

    acceleration.accel.accel.linear.x = acceleration_linear_variable.x();
    acceleration.accel.accel.linear.y = acceleration_linear_variable.y();
    acceleration.accel.accel.linear.z = acceleration_linear_variable.z();
    acceleration.accel.accel.angular.x = 0.0;
    acceleration.accel.accel.angular.y = 0.0;
    acceleration.accel.accel.angular.z = 0.0;
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 10.0 * 1000,
      "Failed to find a state at time " << stamp.nanoseconds() << ". Error: " << e.what());
    return false;
  } catch (...) {
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 10.0 * 1000,
      "Failed to find a state at time " << stamp.nanoseconds() << ". Error: unknown");
    return false;
  }

  return true;
}

void Odometry3DPublisher::publishTimerCallback()
{
  rclcpp::Time latest_stamp;
  rclcpp::Time latest_covariance_stamp;
  bool latest_covariance_valid;
  nav_msgs::msg::Odometry odom_output;
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration_output;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    latest_stamp = latest_stamp_;
    latest_covariance_stamp = latest_covariance_stamp_;
    latest_covariance_valid = latest_covariance_valid_;
    odom_output = odom_output_;
    acceleration_output = acceleration_output_;
  }

  if (0u == latest_stamp.nanoseconds()) {
    RCLCPP_WARN_STREAM_EXPRESSION(
      logger_, delayed_throttle_filter_.isEnabled(),
      "No valid state data yet. Delaying tf broadcast.");
    return;
  }

  tf2::Transform pose;
  tf2::fromMsg(odom_output.pose.pose, pose);

  // If requested, we need to project our state forward in time using the 3D kinematic model
  if (params_.predict_to_current_time) {
    rclcpp::Time timer_now = interfaces_.get_node_clock_interface()->get_clock()->now();

    // Convert pose in Eigen representation
    fuse_core::Vector3d position, velocity_linear, velocity_angular;
    Eigen::Quaterniond orientation;
    position << pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z();
    orientation.x() = pose.getRotation().x();
    orientation.y() = pose.getRotation().y();
    orientation.z() = pose.getRotation().z();
    orientation.w() = pose.getRotation().w();
    velocity_linear << odom_output.twist.twist.linear.x, 
      odom_output.twist.twist.linear.y, odom_output.twist.twist.linear.z;
    velocity_angular << odom_output.twist.twist.angular.x, 
      odom_output.twist.twist.angular.y, odom_output.twist.twist.angular.z;

    const double dt = timer_now.seconds() - rclcpp::Time(odom_output.header.stamp).seconds();

    fuse_core::Matrix15d jacobian;

    fuse_core::Vector3d acceleration_linear;
    if (params_.predict_with_acceleration) {
      acceleration_linear << acceleration_output.accel.accel.linear.x, 
        acceleration_output.accel.accel.linear.y, acceleration_output.accel.accel.linear.z;
    }

    predict(
      position,
      orientation,
      velocity_linear,
      velocity_angular,
      acceleration_linear,
      dt,
      position,
      orientation,
      velocity_linear,
      velocity_angular,
      acceleration_linear,
      jacobian);

    // Convert back to tf2 representation
    pose.setOrigin(tf2::Vector3(position.x(), position.y(), position.z()));
    pose.setRotation(tf2::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()));

    odom_output.pose.pose.position.x = position.x();
    odom_output.pose.pose.position.y = position.y();
    odom_output.pose.pose.position.z = position.z();
    odom_output.pose.pose.orientation.x = orientation.x();
    odom_output.pose.pose.orientation.y = orientation.y();
    odom_output.pose.pose.orientation.z = orientation.z();
    odom_output.pose.pose.orientation.w = orientation.w();

    odom_output.twist.twist.linear.x = velocity_linear.x();
    odom_output.twist.twist.linear.y = velocity_linear.y();
    odom_output.twist.twist.linear.z = velocity_linear.z();
    odom_output.twist.twist.angular.x = velocity_angular.x();
    odom_output.twist.twist.angular.y = velocity_angular.y();
    odom_output.twist.twist.angular.z = velocity_angular.z();

    if (params_.predict_with_acceleration) {
      acceleration_output.accel.accel.linear.x = acceleration_linear.x();
      acceleration_output.accel.accel.linear.y = acceleration_linear.y();
      acceleration_output.accel.accel.linear.z = acceleration_linear.z();
    }

    odom_output.header.stamp = timer_now;
    acceleration_output.header.stamp = timer_now;

    // Either the last covariance computation was skipped because there was no subscriber,
    // or it failed
    if (latest_covariance_valid) {
      fuse_core::Matrix15d covariance;
      covariance.setZero();
      Eigen::Map<fuse_core::Matrix6d> pose_covariance(odom_output.pose.covariance.data());
      Eigen::Map<fuse_core::Matrix6d> twist_covariance(odom_output.twist.covariance.data());
      Eigen::Map<fuse_core::Matrix3d> acceleration_covariance(acceleration_output.accel.covariance.data());

      covariance.block<6, 6>(0, 0) = pose_covariance;
      covariance.block<6, 6>(6, 6) = twist_covariance;
      covariance.block<3, 3>(12, 12) = acceleration_covariance;

      covariance = jacobian * covariance * jacobian.transpose();

      auto process_noise_covariance = params_.process_noise_covariance;
      if (params_.scale_process_noise) {
        common::scaleProcessNoiseCovariance(
          process_noise_covariance, 
          velocity_linear,
          velocity_angular, 
          params_.velocity_linear_norm_min_,
          params_.velocity_angular_norm_min_);
      }

      covariance.noalias() += dt * process_noise_covariance;

      pose_covariance = covariance.block<6, 6>(0, 0);
      twist_covariance = covariance.block<6, 6>(6, 6);
      acceleration_covariance = covariance.block<3, 3>(12, 12);
    }
  }

  odom_pub_->publish(odom_output);
  acceleration_pub_->publish(acceleration_output);

  if (params_.publish_tf) {
    auto frame_id = odom_output.header.frame_id;
    auto child_frame_id = odom_output.child_frame_id;

    if (params_.invert_tf) {
      pose = pose.inverse();
      std::swap(frame_id, child_frame_id);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = odom_output.header.stamp;
    trans.header.frame_id = frame_id;
    trans.child_frame_id = child_frame_id;
    trans.transform = tf2::toMsg(pose);    
    if (!params_.invert_tf && params_.world_frame_id == params_.map_frame_id) {
      try {
        auto base_to_odom = tf_buffer_->lookupTransform(
          params_.base_link_frame_id,
          params_.odom_frame_id,
          trans.header.stamp,
          params_.tf_timeout);

        geometry_msgs::msg::TransformStamped map_to_odom;
        tf2::doTransform(base_to_odom, map_to_odom, trans);
        map_to_odom.child_frame_id = params_.odom_frame_id;
        trans = map_to_odom;
      } catch (const std::exception & e) {
        RCLCPP_WARN_STREAM_THROTTLE(
          logger_, *clock_, 5.0 * 1000,
          "Could not lookup the " << params_.base_link_frame_id << "->"
                                  << params_.odom_frame_id << " transform. Error: " << e.what());

        return;
      }
    }

    tf_broadcaster_->sendTransform(trans);
  }
}

}  // namespace fuse_models
