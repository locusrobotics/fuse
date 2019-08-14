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
#include <fuse_models/common/sensor_proc.h>
#include <fuse_models/imu_2d.h>

#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <memory>


// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Imu2D, fuse_core::SensorModel)

namespace fuse_models
{

Imu2D::Imu2D() :
  fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  tf_listener_(tf_buffer_)
{
}

void Imu2D::onInit()
{
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(private_node_handle_);

  params_.loadFromROS(private_node_handle_);

  if (params_.orientation_indices.empty() &&
      params_.linear_acceleration_indices.empty() &&
      params_.angular_velocity_indices.empty())
  {
    ROS_WARN_STREAM("No dimensions were specified. Data from topic " << ros::names::resolve(params_.topic) <<
                    " will be ignored.");
  }
}

void Imu2D::onStart()
{
  if (!params_.orientation_indices.empty() ||
      !params_.linear_acceleration_indices.empty() ||
      !params_.angular_velocity_indices.empty())
  {
    previous_pose_.reset();
    subscriber_ = node_handle_.subscribe(ros::names::resolve(params_.topic), params_.queue_size, &Imu2D::process, this);
  }
}

void Imu2D::onStop()
{
  subscriber_.shutdown();
}

void Imu2D::process(const sensor_msgs::Imu::ConstPtr& msg)
{
  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg->header.stamp);

  // Handle the orientation data (treat it as a pose, but with only orientation indices used)
  auto pose = std::make_unique<geometry_msgs::PoseWithCovarianceStamped>();
  pose->header = msg->header;
  pose->pose.pose.orientation = msg->orientation;
  pose->pose.covariance[21] = msg->orientation_covariance[0];
  pose->pose.covariance[22] = msg->orientation_covariance[1];
  pose->pose.covariance[23] = msg->orientation_covariance[2];
  pose->pose.covariance[27] = msg->orientation_covariance[3];
  pose->pose.covariance[28] = msg->orientation_covariance[4];
  pose->pose.covariance[29] = msg->orientation_covariance[5];
  pose->pose.covariance[33] = msg->orientation_covariance[6];
  pose->pose.covariance[34] = msg->orientation_covariance[7];
  pose->pose.covariance[35] = msg->orientation_covariance[8];

  if (params_.differential)
  {
    if (previous_pose_)
    {
      common::processDifferentialPoseWithCovariance(
        device_id_,
        *previous_pose_,
        *pose,
        {},
        params_.orientation_indices,
        *transaction);
    }

    previous_pose_ = std::move(pose);
  }
  else
  {
    common::processAbsolutePoseWithCovariance(
      device_id_,
      *pose,
      params_.orientation_target_frame,
      {},
      params_.orientation_indices,
      tf_buffer_,
      *transaction);
  }

  // Handle the twist data (only include indices for angular velocity)
  geometry_msgs::TwistWithCovarianceStamped twist;
  twist.header = msg->header;
  twist.twist.twist.angular = msg->angular_velocity;
  twist.twist.covariance[21] = msg->angular_velocity_covariance[0];
  twist.twist.covariance[22] = msg->angular_velocity_covariance[1];
  twist.twist.covariance[23] = msg->angular_velocity_covariance[2];
  twist.twist.covariance[27] = msg->angular_velocity_covariance[3];
  twist.twist.covariance[28] = msg->angular_velocity_covariance[4];
  twist.twist.covariance[29] = msg->angular_velocity_covariance[5];
  twist.twist.covariance[33] = msg->angular_velocity_covariance[6];
  twist.twist.covariance[34] = msg->angular_velocity_covariance[7];
  twist.twist.covariance[35] = msg->angular_velocity_covariance[8];

  common::processTwistWithCovariance(
    device_id_,
    twist,
    params_.twist_target_frame,
    {},
    params_.angular_velocity_indices,
    tf_buffer_,
    *transaction);

  // Handle the acceleration data
  geometry_msgs::AccelWithCovarianceStamped accel;
  accel.header = msg->header;
  accel.accel.accel.linear = msg->linear_acceleration;
  accel.accel.covariance[0]  = msg->linear_acceleration_covariance[0];
  accel.accel.covariance[1]  = msg->linear_acceleration_covariance[1];
  accel.accel.covariance[2]  = msg->linear_acceleration_covariance[2];
  accel.accel.covariance[6]  = msg->linear_acceleration_covariance[3];
  accel.accel.covariance[7]  = msg->linear_acceleration_covariance[4];
  accel.accel.covariance[8]  = msg->linear_acceleration_covariance[5];
  accel.accel.covariance[12] = msg->linear_acceleration_covariance[6];
  accel.accel.covariance[13] = msg->linear_acceleration_covariance[7];
  accel.accel.covariance[14] = msg->linear_acceleration_covariance[8];

  // Optionally remove the acceleration due to gravity
  if (params_.remove_gravitational_acceleration)
  {
    geometry_msgs::Vector3 accel_gravity;
    accel_gravity.z = params_.gravitational_acceleration;
    geometry_msgs::TransformStamped orientation_trans;
    tf2::Quaternion imu_orientation;
    tf2::fromMsg(msg->orientation, imu_orientation);
    orientation_trans.transform.rotation = tf2::toMsg(imu_orientation.inverse());
    tf2::doTransform(accel_gravity, accel_gravity, orientation_trans);  // Doesn't use the stamp
    accel.accel.accel.linear.x -= accel_gravity.x;
    accel.accel.accel.linear.y -= accel_gravity.y;
    accel.accel.accel.linear.z -= accel_gravity.z;
  }

  common::processAccelWithCovariance(
    device_id_,
    accel,
    params_.acceleration_target_frame,
    params_.linear_acceleration_indices,
    tf_buffer_,
    *transaction);

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
}

}  // namespace fuse_models
