/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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
#include <fuse_models/odometry_2d_publisher.h>
#include <fuse_models/unicycle_2d_predict.h>
#include <fuse_models/common/sensor_proc.h>

#include <fuse_core/async_publisher.h>
#include <fuse_core/eigen.h>
#include <fuse_core/uuid.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_2d/tf2_2d.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Odometry2DPublisher, fuse_core::Publisher)

namespace fuse_models
{

Odometry2DPublisher::Odometry2DPublisher() :
  fuse_core::AsyncPublisher(1),
  device_id_(fuse_core::uuid::NIL),
  latest_stamp_(Synchronizer::ROS_TIME_ZERO),
  latest_covariance_stamp_(Synchronizer::ROS_TIME_ZERO),
  publish_timer_spinner_(1, &publish_timer_callback_queue_)
{
}

void Odometry2DPublisher::onInit()
{
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);

  params_.loadFromROS(private_node_handle_);

  if (!params_.invert_tf && params_.world_frame_id == params_.map_frame_id)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(params_.tf_cache_time);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_handle_);
  }

  odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>(ros::names::resolve(params_.topic), params_.queue_size);
  acceleration_pub_ = node_handle_.advertise<geometry_msgs::AccelWithCovarianceStamped>(
      ros::names::resolve(params_.acceleration_topic), params_.queue_size);

  publish_timer_node_handle_.setCallbackQueue(&publish_timer_callback_queue_);
  publish_timer_spinner_.start();
}

void Odometry2DPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  // Find the most recent common timestamp
  const auto latest_stamp = synchronizer_.findLatestCommonStamp(*transaction, *graph);
  if (latest_stamp == Synchronizer::ROS_TIME_ZERO)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_stamp_ = latest_stamp;
    }

    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 10.0 * 1000,
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

  nav_msgs::Odometry odom_output;
  geometry_msgs::AccelWithCovarianceStamped acceleration_output;
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
  if (odom_pub_.getNumSubscribers() > 0 || acceleration_pub_.getNumSubscribers() > 0)
  {
    // Throttle covariance computation
    if (params_.covariance_throttle_period.isZero() ||
       latest_stamp - latest_covariance_stamp > params_.covariance_throttle_period)
    {
      latest_covariance_stamp = latest_stamp;

      try
      {
        std::vector<std::pair<fuse_core::UUID, fuse_core::UUID>> covariance_requests;
        covariance_requests.emplace_back(position_uuid, position_uuid);
        covariance_requests.emplace_back(position_uuid, orientation_uuid);
        covariance_requests.emplace_back(orientation_uuid, orientation_uuid);
        covariance_requests.emplace_back(velocity_linear_uuid, velocity_linear_uuid);
        covariance_requests.emplace_back(velocity_linear_uuid, velocity_angular_uuid);
        covariance_requests.emplace_back(velocity_angular_uuid, velocity_angular_uuid);
        covariance_requests.emplace_back(acceleration_linear_uuid, acceleration_linear_uuid);

        std::vector<std::vector<double>> covariance_matrices;
        graph->getCovariance(covariance_requests, covariance_matrices, params_.covariance_options);

        odom_output.pose.covariance[0] = covariance_matrices[0][0];
        odom_output.pose.covariance[1] = covariance_matrices[0][1];
        odom_output.pose.covariance[5] = covariance_matrices[1][0];
        odom_output.pose.covariance[6] = covariance_matrices[0][2];
        odom_output.pose.covariance[7] = covariance_matrices[0][3];
        odom_output.pose.covariance[11] = covariance_matrices[1][1];
        odom_output.pose.covariance[30] = covariance_matrices[1][0];
        odom_output.pose.covariance[31] = covariance_matrices[1][1];
        odom_output.pose.covariance[35] = covariance_matrices[2][0];

        odom_output.twist.covariance[0] = covariance_matrices[3][0];
        odom_output.twist.covariance[1] = covariance_matrices[3][1];
        odom_output.twist.covariance[5] = covariance_matrices[4][0];
        odom_output.twist.covariance[6] = covariance_matrices[3][2];
        odom_output.twist.covariance[7] = covariance_matrices[3][3];
        odom_output.twist.covariance[11] = covariance_matrices[4][1];
        odom_output.twist.covariance[30] = covariance_matrices[4][0];
        odom_output.twist.covariance[31] = covariance_matrices[4][1];
        odom_output.twist.covariance[35] = covariance_matrices[5][0];

        acceleration_output.accel.covariance[0] = covariance_matrices[6][0];
        acceleration_output.accel.covariance[1] = covariance_matrices[6][1];
        acceleration_output.accel.covariance[6] = covariance_matrices[6][2];
        acceleration_output.accel.covariance[7] = covariance_matrices[6][3];

        latest_covariance_valid = true;
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "An error occurred computing the covariance information for " << latest_stamp
                           << ". The covariance will be set to zero.\n" << e.what());
        std::fill(odom_output.pose.covariance.begin(), odom_output.pose.covariance.end(), 0.0);
        std::fill(odom_output.twist.covariance.begin(), odom_output.twist.covariance.end(), 0.0);
        std::fill(acceleration_output.accel.covariance.begin(), acceleration_output.accel.covariance.end(), 0.0);

        latest_covariance_valid = false;
      }
    }
    else
    {
      // This covariance computation cycle has been skipped, so simply take the last covariance computed
      //
      // We do not propagate the latest covariance forward because it would grow unbounded being very different from
      // the actual covariance we would have computed if not throttling.
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

void Odometry2DPublisher::onStart()
{
  synchronizer_ = Synchronizer(device_id_);
  latest_stamp_ = latest_covariance_stamp_ = Synchronizer::ROS_TIME_ZERO;
  latest_covariance_valid_ = false;
  odom_output_ = nav_msgs::Odometry();
  acceleration_output_ = geometry_msgs::AccelWithCovarianceStamped();

  // TODO(CH3): Add this to a separate callback group for async behavior
  publish_timer_ = this->node_.create_timer(
    rclcpp::Duration::from_seconds(1.0 / params_.publish_frequency),
    std::bind(&Odometry2DPublisher::publishTimerCallback, this)
  );

  delayed_throttle_filter_.reset();
}

void Odometry2DPublisher::onStop()
{
  publish_timer_.cancel();
}

bool Odometry2DPublisher::getState(
  const fuse_core::Graph& graph,
  const rclcpp::Time& stamp,
  const fuse_core::UUID& device_id,
  fuse_core::UUID& position_uuid,
  fuse_core::UUID& orientation_uuid,
  fuse_core::UUID& velocity_linear_uuid,
  fuse_core::UUID& velocity_angular_uuid,
  fuse_core::UUID& acceleration_linear_uuid,
  nav_msgs::Odometry& odometry,
  geometry_msgs::AccelWithCovarianceStamped& acceleration)
{
  try
  {
    position_uuid = fuse_variables::Position2DStamped(stamp, device_id).uuid();
    auto position_variable = dynamic_cast<const fuse_variables::Position2DStamped&>(
      graph.getVariable(position_uuid));

    orientation_uuid = fuse_variables::Orientation2DStamped(stamp, device_id).uuid();
    auto orientation_variable = dynamic_cast<const fuse_variables::Orientation2DStamped&>(
      graph.getVariable(orientation_uuid));

    velocity_linear_uuid = fuse_variables::VelocityLinear2DStamped(stamp, device_id).uuid();
    auto velocity_linear_variable = dynamic_cast<const fuse_variables::VelocityLinear2DStamped&>(
      graph.getVariable(velocity_linear_uuid));

    velocity_angular_uuid = fuse_variables::VelocityAngular2DStamped(stamp, device_id).uuid();
    auto velocity_angular_variable = dynamic_cast<const fuse_variables::VelocityAngular2DStamped&>(
      graph.getVariable(velocity_angular_uuid));

    acceleration_linear_uuid = fuse_variables::AccelerationLinear2DStamped(stamp, device_id).uuid();
    auto acceleration_linear_variable = dynamic_cast<const fuse_variables::AccelerationLinear2DStamped&>(
      graph.getVariable(acceleration_linear_uuid));

    odometry.pose.pose.position.x = position_variable.x();
    odometry.pose.pose.position.y = position_variable.y();
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = tf2::toMsg(tf2_2d::Rotation(orientation_variable.yaw()));
    odometry.twist.twist.linear.x = velocity_linear_variable.x();
    odometry.twist.twist.linear.y = velocity_linear_variable.y();
    odometry.twist.twist.linear.z = 0.0;
    odometry.twist.twist.angular.x = 0.0;
    odometry.twist.twist.angular.y = 0.0;
    odometry.twist.twist.angular.z = velocity_angular_variable.yaw();

    acceleration.accel.accel.linear.x = acceleration_linear_variable.x();
    acceleration.accel.accel.linear.y = acceleration_linear_variable.y();
    acceleration.accel.accel.linear.z = 0.0;
    acceleration.accel.accel.angular.x = 0.0;
    acceleration.accel.accel.angular.y = 0.0;
    acceleration.accel.accel.angular.z = 0.0;
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 10.0 * 1000,
                                "Failed to find a state at time " << stamp << ". Error: " << e.what());
    return false;
  }
  catch (...)
  {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 10.0 * 1000,
                                "Failed to find a state at time " << stamp << ". Error: unknown");
    return false;
  }

  return true;
}

void Odometry2DPublisher::publishTimerCallback()
{
  rclcpp::Time latest_stamp;
  rclcpp::Time latest_covariance_stamp;
  bool latest_covariance_valid;
  nav_msgs::Odometry odom_output;
  geometry_msgs::AccelWithCovarianceStamped acceleration_output;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    latest_stamp = latest_stamp_;
    latest_covariance_stamp = latest_covariance_stamp_;
    latest_covariance_valid = latest_covariance_valid_;
    odom_output = odom_output_;
    acceleration_output = acceleration_output_;
  }

  if (latest_stamp == Synchronizer::ROS_TIME_ZERO)
  {
    RCLCPP_WARN_STREAM_EXPRESSION(
      node_->get_logger(), delayed_throttle_filter_.isEnabled(),
      "No valid state data yet. Delaying tf broadcast.");
    return;
  }

  tf2_2d::Transform pose;
  tf2::fromMsg(odom_output.pose.pose, pose);

  // If requested, we need to project our state forward in time using the 2D kinematic model
  if (params_.predict_to_current_time)
  {
    tf2_2d::Vector2 velocity_linear;
    tf2::fromMsg(odom_output.twist.twist.linear, velocity_linear);

    const double dt = event.current_real.seconds() - odom_output.header.stamp.seconds();

    fuse_core::Matrix8d jacobian;

    tf2_2d::Vector2 acceleration_linear;
    if (params_.predict_with_acceleration)
    {
      tf2::fromMsg(acceleration_output.accel.accel.linear, acceleration_linear);
    }

    double yaw_vel;

    predict(
      pose,
      velocity_linear,
      odom_output.twist.twist.angular.z,
      acceleration_linear,
      dt,
      pose,
      velocity_linear,
      yaw_vel,
      acceleration_linear,
      jacobian);

    odom_output.pose.pose.position.x = pose.getX();
    odom_output.pose.pose.position.y = pose.getY();
    odom_output.pose.pose.orientation = tf2::toMsg(pose.getRotation());

    odom_output.twist.twist.linear.x = velocity_linear.x();
    odom_output.twist.twist.linear.y = velocity_linear.y();
    odom_output.twist.twist.angular.z = yaw_vel;

    if (params_.predict_with_acceleration)
    {
      acceleration_output.accel.accel.linear.x = acceleration_linear.x();
      acceleration_output.accel.accel.linear.y = acceleration_linear.y();
    }

    odom_output.header.stamp = event.current_real;
    acceleration_output.header.stamp = event.current_real;

    // Either the last covariance computation was skipped because there was no subscriber,
    // or it failed
    if (latest_covariance_valid)
    {
      fuse_core::Matrix8d covariance;
      covariance(0, 0) = odom_output.pose.covariance[0];
      covariance(0, 1) = odom_output.pose.covariance[1];
      covariance(0, 2) = odom_output.pose.covariance[5];
      covariance(1, 0) = odom_output.pose.covariance[6];
      covariance(1, 1) = odom_output.pose.covariance[7];
      covariance(1, 2) = odom_output.pose.covariance[11];
      covariance(2, 0) = odom_output.pose.covariance[30];
      covariance(2, 1) = odom_output.pose.covariance[31];
      covariance(2, 2) = odom_output.pose.covariance[35];

      covariance(3, 3) = odom_output.twist.covariance[0];
      covariance(3, 4) = odom_output.twist.covariance[1];
      covariance(3, 5) = odom_output.twist.covariance[5];
      covariance(4, 3) = odom_output.twist.covariance[6];
      covariance(4, 4) = odom_output.twist.covariance[7];
      covariance(4, 5) = odom_output.twist.covariance[11];
      covariance(5, 3) = odom_output.twist.covariance[30];
      covariance(5, 4) = odom_output.twist.covariance[31];
      covariance(5, 5) = odom_output.twist.covariance[35];

      covariance(6, 6) = acceleration_output.accel.covariance[0];
      covariance(6, 7) = acceleration_output.accel.covariance[1];
      covariance(7, 6) = acceleration_output.accel.covariance[6];
      covariance(7, 7) = acceleration_output.accel.covariance[7];

      // TODO(efernandez) for now we set to zero the out-of-diagonal blocks with the correlations between pose, twist
      // and acceleration, but we could cache them in another attribute when we retrieve the covariance from the ceres
      // problem
      covariance.topRightCorner<3, 5>().setZero();
      covariance.bottomLeftCorner<5, 3>().setZero();
      covariance.block<3, 2>(3, 6).setZero();
      covariance.block<2, 3>(6, 3).setZero();

      covariance = jacobian * covariance * jacobian.transpose();

      auto process_noise_covariance = params_.process_noise_covariance;
      if (params_.scale_process_noise)
      {
        common::scaleProcessNoiseCovariance(process_noise_covariance, velocity_linear,
                                            odom_output.twist.twist.angular.z, params_.velocity_norm_min);
      }

      covariance.noalias() += dt * process_noise_covariance;

      odom_output.pose.covariance[0] = covariance(0, 0);
      odom_output.pose.covariance[1] = covariance(0, 1);
      odom_output.pose.covariance[5] = covariance(0, 2);
      odom_output.pose.covariance[6] = covariance(1, 0);
      odom_output.pose.covariance[7] = covariance(1, 1);
      odom_output.pose.covariance[11] = covariance(1, 2);
      odom_output.pose.covariance[30] = covariance(2, 0);
      odom_output.pose.covariance[31] = covariance(2, 1);
      odom_output.pose.covariance[35] = covariance(2, 2);

      odom_output.twist.covariance[0] = covariance(3, 3);
      odom_output.twist.covariance[1] = covariance(3, 4);
      odom_output.twist.covariance[5] = covariance(3, 5);
      odom_output.twist.covariance[6] = covariance(4, 3);
      odom_output.twist.covariance[7] = covariance(4, 4);
      odom_output.twist.covariance[11] = covariance(4, 5);
      odom_output.twist.covariance[30] = covariance(5, 3);
      odom_output.twist.covariance[31] = covariance(5, 4);
      odom_output.twist.covariance[35] = covariance(5, 5);

      acceleration_output.accel.covariance[0] = covariance(6, 6);
      acceleration_output.accel.covariance[1] = covariance(6, 7);
      acceleration_output.accel.covariance[6] = covariance(7, 6);
      acceleration_output.accel.covariance[7] = covariance(7, 7);
    }
  }

  odom_pub_.publish(odom_output);
  acceleration_pub_.publish(acceleration_output);

  if (params_.publish_tf)
  {
    auto frame_id = odom_output.header.frame_id;
    auto child_frame_id = odom_output.child_frame_id;

    if (params_.invert_tf)
    {
      pose = pose.inverse();
      std::swap(frame_id, child_frame_id);
    }

    geometry_msgs::TransformStamped trans;
    trans.header.stamp = odom_output.header.stamp;
    trans.header.frame_id = frame_id;
    trans.child_frame_id = child_frame_id;
    trans.transform.translation.x = pose.getX();
    trans.transform.translation.y = pose.getY();
    trans.transform.translation.z = odom_output.pose.pose.position.z;
    trans.transform.rotation = tf2::toMsg(pose.getRotation());

    if (!params_.invert_tf && params_.world_frame_id == params_.map_frame_id)
    {
      try
      {
        auto base_to_odom = tf_buffer_->lookupTransform(
          params_.base_link_frame_id,
          params_.odom_frame_id,
          trans.header.stamp,
          params_.tf_timeout);

        geometry_msgs::TransformStamped map_to_odom;
        tf2::doTransform(base_to_odom, map_to_odom, trans);
        map_to_odom.child_frame_id = params_.odom_frame_id;
        trans = map_to_odom;
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5.0 * 1000,
          "Could not lookup the " << params_.base_link_frame_id << "->"
          << params_.odom_frame_id<< " transform. Error: " << e.what());

        return;
      }
    }

    tf_broadcaster_.sendTransform(trans);
  }
}

}  // namespace fuse_models
