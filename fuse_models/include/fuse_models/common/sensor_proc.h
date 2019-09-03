/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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
#ifndef FUSE_MODELS_COMMON_SENSOR_PROC_H
#define FUSE_MODELS_COMMON_SENSOR_PROC_H

#include <fuse_constraints/absolute_pose_2d_stamped_constraint.h>
#include <fuse_constraints/relative_pose_2d_stamped_constraint.h>
#include <fuse_constraints/absolute_constraint.h>
#include <fuse_core/eigen.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_variables/acceleration_linear_2d_stamped.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_variables/velocity_linear_2d_stamped.h>
#include <fuse_variables/velocity_angular_2d_stamped.h>
#include <fuse_variables/stamped.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_2d/tf2_2d.h>
#include <tf2_2d/transform.h>

#include <boost/range/join.hpp>

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>


namespace tf2
{

/** \brief Apply a geometry_msgs TransformStamped to a geometry_msgs TwistWithCovarianceStamped type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The twist to transform, as a timestamped TwistWithCovarianceStamped message.
* \param t_out The transformed twist, as a timestamped TwistWithCovarianceStamped message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::TwistWithCovarianceStamped& t_in, geometry_msgs::TwistWithCovarianceStamped& t_out, const geometry_msgs::TransformStamped& transform)  // NOLINT
{
  tf2::Vector3 vl;
  fromMsg(t_in.twist.twist.linear, vl);
  tf2::Vector3 va;
  fromMsg(t_in.twist.twist.angular, va);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  t_out.twist.twist.linear = tf2::toMsg(t.getBasis() * vl);
  t_out.twist.twist.angular = tf2::toMsg(t.getBasis() * va);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;

  t_out.twist.covariance = transformCovariance(t_in.twist.covariance, t);
}

/** \brief Apply a geometry_msgs TransformStamped to a geometry_msgs AccelWithCovarianceStamped type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The acceleration to transform, as a timestamped AccelWithCovarianceStamped message.
* \param t_out The transformed acceleration, as a timestamped AccelWithCovarianceStamped message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::AccelWithCovarianceStamped& t_in, geometry_msgs::AccelWithCovarianceStamped& t_out, const geometry_msgs::TransformStamped& transform)  // NOLINT
{
  tf2::Vector3 al;
  fromMsg(t_in.accel.accel.linear, al);
  tf2::Vector3 aa;
  fromMsg(t_in.accel.accel.angular, aa);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  t_out.accel.accel.linear = tf2::toMsg(t.getBasis() * al);
  t_out.accel.accel.angular = tf2::toMsg(t.getBasis() * aa);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;

  t_out.accel.covariance = transformCovariance(t_in.accel.covariance, t);
}

}  // namespace tf2


namespace fuse_models
{

namespace common
{

/**
 * @brief Method to merge two vectors of indices adding an offset to the RHS one.
 *
 * @param[in] lhs_indices - LHS vector of indices
 * @param[in] rhs_indices - RHS vector of indices
 * @param[in] rhs_offset - RHS offset to be added to the RHS vector indices (defaults to 0)
 */
inline std::vector<size_t> mergeIndices(
  const std::vector<size_t>& lhs_indices,
  const std::vector<size_t>& rhs_indices,
  const size_t rhs_offset = 0u)
{
  auto merged_indices = boost::copy_range<std::vector<size_t>>(boost::join(lhs_indices, rhs_indices));

  const auto rhs_it = merged_indices.begin() + lhs_indices.size();
  std::transform(
    rhs_it,
    merged_indices.end(),
    rhs_it,
    std::bind(std::plus<size_t>(), std::placeholders::_1, rhs_offset));

  return merged_indices;
}

/**
 * @brief Method to create sub-measurements from full measurements and append them to existing partial measurements
 *
 * @param[in] mean_full - The full mean vector from which we will generate the sub-measurement
 * @param[in] covariance_full - The full covariance matrix from which we will generate the sub-measurement
 * @param[in] indices - The indices we want to include in the sub-measurement
 * @param[in,out] mean_partial - The partial measurement mean to which we want to append
 * @param[in,out] covariance_partial - The partial measurement covariance to which we want to append
 */
inline void populatePartialMeasurement(
  const fuse_core::VectorXd& mean_full,
  const fuse_core::MatrixXd& covariance_full,
  const std::vector<size_t>& indices,
  fuse_core::VectorXd& mean_partial,
  fuse_core::MatrixXd& covariance_partial)
{
  for (size_t r = 0; r < indices.size(); ++r)
  {
    mean_partial(r) = mean_full(indices[r]);

    for (size_t c = 0; c < indices.size(); ++c)
    {
      covariance_partial(r, c) = covariance_full(indices[r], indices[c]);
    }
  }
}

/**
 * @brief Transforms a ROS geometry message from its frame to the frame of the output message
 *
 * @param[in] tf_buffer - The transform buffer with which we will lookup the required transform
 * @param[in] input - The message to transform. Source frame and stamp are dictated by its header.
 * @param[in,out] output - The transformed message. Target frame is dictated by its header.
 * @return true if the transform succeeded, false otherwise
 */
template <typename T>
bool transformMessage(const tf2_ros::Buffer& tf_buffer, const T& input, T& output)
{
  geometry_msgs::TransformStamped trans;

  bool have_transform = false;

  if (tf_buffer.canTransform(output.header.frame_id, input.header.frame_id, input.header.stamp))
  {
    try
    {
      trans = tf_buffer.lookupTransform(output.header.frame_id, input.header.frame_id, input.header.stamp);
      have_transform = true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "Could not transform message from " << input.header.frame_id << " to " <<
        output.header.frame_id << ". Error was " << ex.what() << " Will attempt to use latest transform instead.");
    }
  }

  if (!have_transform)
  {
    try
    {
      trans = tf_buffer.lookupTransform(output.header.frame_id, input.header.frame_id, ros::Time(0));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM_THROTTLE(5.0, "Could not transform message from " << input.header.frame_id << " to " <<
        output.header.frame_id << ". Error was " << ex.what());

      return false;
    }
  }

  tf2::doTransform(input, output, trans);
  return true;
}

/**
 * @brief Extracts 2D pose data from a PoseWithCovarianceStamped message and adds that data to a fuse Transaction
 *
 * This method effectively adds two variables (2D position and 2D orientation) and a 2D pose constraint to the given
 * \p transaction. The pose data is extracted from the \p pose message. Only 2D data is used. The data will be
 * automatically transformed into the \p target_frame before it is used.
 *
 * @param[in] source - The name of the sensor or motion model that generated this constraint
 * @param[in] device_id - The UUID of the machine
 * @param[in] pose - The PoseWithCovarianceStamped message from which we will extract the pose data
 * @param[in] target_frame - The frame ID into which the pose data will be transformed before it is used
 * @param[in] tf_buffer - The transform buffer with which we will lookup the required transform
 * @param[out] transaction - The generated variables and constraints are added to this transaction
 * @return true if any constraints were added, false otherwise
 */
inline bool processAbsolutePoseWithCovariance(
  const std::string& source,
  const fuse_core::UUID& device_id,
  const geometry_msgs::PoseWithCovarianceStamped& pose,
  const std::string& target_frame,
  const std::vector<size_t>& position_indices,
  const std::vector<size_t>& orientation_indices,
  const tf2_ros::Buffer& tf_buffer,
  fuse_core::Transaction& transaction)
{
  if (position_indices.empty() && orientation_indices.empty())
  {
    return false;
  }

  geometry_msgs::PoseWithCovarianceStamped transformed_message;
  transformed_message.header.frame_id = target_frame;

  if (!transformMessage(tf_buffer, pose, transformed_message))
  {
    ROS_ERROR_STREAM("Cannot create constraint from pose message with stamp " << pose.header.stamp);
    return false;
  }

  // Convert the pose into tf2_2d transform
  tf2_2d::Transform absolute_pose_2d;
  tf2::fromMsg(transformed_message.pose.pose, absolute_pose_2d);

  // Create the pose variable
  auto position = fuse_variables::Position2DStamped::make_shared(transformed_message.header.stamp, device_id);
  auto orientation = fuse_variables::Orientation2DStamped::make_shared(transformed_message.header.stamp, device_id);
  position->x() = absolute_pose_2d.x();
  position->y() = absolute_pose_2d.y();
  orientation->yaw() = absolute_pose_2d.yaw();

  // Create the pose for the constraint
  fuse_core::Vector3d pose_mean;
  pose_mean << absolute_pose_2d.x(), absolute_pose_2d.y(), absolute_pose_2d.yaw();

  // Create the covariance for the constraint
  fuse_core::Matrix3d pose_covariance;
  pose_covariance <<
      transformed_message.pose.covariance[0],
      transformed_message.pose.covariance[1],
      transformed_message.pose.covariance[5],
      transformed_message.pose.covariance[6],
      transformed_message.pose.covariance[7],
      transformed_message.pose.covariance[11],
      transformed_message.pose.covariance[30],
      transformed_message.pose.covariance[31],
      transformed_message.pose.covariance[35];

  // Build the sub-vector and sub-matrices based on the requested indices
  fuse_core::VectorXd pose_mean_partial(position_indices.size() + orientation_indices.size());
  fuse_core::MatrixXd pose_covariance_partial(pose_mean_partial.rows(), pose_mean_partial.rows());

  const auto indices = mergeIndices(position_indices, orientation_indices, position->size());

  populatePartialMeasurement(pose_mean, pose_covariance, indices, pose_mean_partial, pose_covariance_partial);

  // Create an absolute pose constraint
  auto constraint = fuse_constraints::AbsolutePose2DStampedConstraint::make_shared(
    source,
    *position,
    *orientation,
    pose_mean_partial,
    pose_covariance_partial,
    position_indices,
    orientation_indices);

  transaction.addVariable(position);
  transaction.addVariable(orientation);
  transaction.addConstraint(constraint);
  transaction.addInvolvedStamp(pose.header.stamp);

  return true;
}

/**
 * @brief Extracts relative 2D pose data from a PoseWithCovarianceStamped and adds that data to a fuse Transaction
 *
 * This method computes the delta between two poses and creates the required fuse variables and constraints, and then
 * adds them to the given \p transaction. Only 2D data is used. The pose delta is calculated as
 *
 * pose_relative = pose_absolute1^-1 * pose_absolute2
 *
 * Additionally, the covariance of each pose message is rotated into the robot's base frame at the time of
 * pose_absolute1. They are then added in the constraint. This assumes independence between the pose measurements.
 *
 * @param[in] source - The name of the sensor or motion model that generated this constraint
 * @param[in] device_id - The UUID of the machine
 * @param[in] pose1 - The first (and temporally earlier) PoseWithCovarianceStamped message
 * @param[in] pose2 - The first (and temporally later) PoseWithCovarianceStamped message
 * @param[out] transaction - The generated variables and constraints are added to this transaction
 * @return true if any constraints were added, false otherwise
 */
inline bool processDifferentialPoseWithCovariance(
  const std::string& source,
  const fuse_core::UUID& device_id,
  const geometry_msgs::PoseWithCovarianceStamped& pose1,
  const geometry_msgs::PoseWithCovarianceStamped& pose2,
  const std::vector<size_t>& position_indices,
  const std::vector<size_t>& orientation_indices,
  fuse_core::Transaction& transaction)
{
  if (position_indices.empty() && orientation_indices.empty())
  {
    return false;
  }

  // Convert the poses into tf2_2d transforms
  tf2_2d::Transform pose1_2d;
  tf2::fromMsg(pose1.pose.pose, pose1_2d);

  tf2_2d::Transform pose2_2d;
  tf2::fromMsg(pose2.pose.pose, pose2_2d);

  // Create the pose variables
  auto position1 = fuse_variables::Position2DStamped::make_shared(pose1.header.stamp, device_id);
  auto orientation1 =
    fuse_variables::Orientation2DStamped::make_shared(pose1.header.stamp, device_id);
  position1->x() = pose1_2d.x();
  position1->y() = pose1_2d.y();
  orientation1->yaw() = pose1_2d.yaw();

  auto position2 = fuse_variables::Position2DStamped::make_shared(pose2.header.stamp, device_id);
  auto orientation2 = fuse_variables::Orientation2DStamped::make_shared(pose2.header.stamp, device_id);
  position2->x() = pose2_2d.x();
  position2->y() = pose2_2d.y();
  orientation2->yaw() = pose2_2d.yaw();

  // Create the delta for the constraint
  const double sy = ::sin(-pose1_2d.yaw());
  const double cy = ::cos(-pose1_2d.yaw());
  double x_diff = pose2_2d.x() - pose1_2d.x();
  double y_diff = pose2_2d.y() - pose1_2d.y();
  fuse_core::Vector3d pose_relative_mean;
  pose_relative_mean <<
    cy * x_diff - sy * y_diff,
    sy * x_diff + cy * y_diff,
    (pose2_2d.rotation() - pose1_2d.rotation()).getAngle();

  // Compute Jacobians so we can rotate the covariance
  fuse_core::Matrix3d j_pose1;
  j_pose1 <<
    -cy,  sy,  sy * x_diff + cy * y_diff,
    -sy, -cy, -cy * x_diff + sy * y_diff,
      0,   0,                         -1;

  fuse_core::Matrix3d j_pose2;
  j_pose2 <<
     cy, -sy,  0,
     sy,  cy,  0,
      0,   0,  1;

  // Create the covariance components for the constraint
  fuse_core::Matrix3d cov1;
  cov1 <<
    pose1.pose.covariance[0],
    pose1.pose.covariance[1],
    pose1.pose.covariance[5],
    pose1.pose.covariance[6],
    pose1.pose.covariance[7],
    pose1.pose.covariance[11],
    pose1.pose.covariance[30],
    pose1.pose.covariance[31],
    pose1.pose.covariance[35];

  fuse_core::Matrix3d cov2;
  cov2 <<
    pose2.pose.covariance[0],
    pose2.pose.covariance[1],
    pose2.pose.covariance[5],
    pose2.pose.covariance[6],
    pose2.pose.covariance[7],
    pose2.pose.covariance[11],
    pose2.pose.covariance[30],
    pose2.pose.covariance[31],
    pose2.pose.covariance[35];

  auto pose_relative_covariance = j_pose1 * cov1 * j_pose1.transpose() + j_pose2 * cov2 * j_pose2.transpose();

  // Build the sub-vector and sub-matrices based on the requested indices
  fuse_core::VectorXd pose_relative_mean_partial(position_indices.size() + orientation_indices.size());
  fuse_core::MatrixXd pose_relative_covariance_partial(pose_relative_mean_partial.rows(),
                                                       pose_relative_mean_partial.rows());

  const auto indices = mergeIndices(position_indices, orientation_indices, position1->size());

  populatePartialMeasurement(
    pose_relative_mean,
    pose_relative_covariance,
    indices,
    pose_relative_mean_partial,
    pose_relative_covariance_partial);

  // Create a relative pose constraint. We assume the pose measurements are independent.
  auto constraint = fuse_constraints::RelativePose2DStampedConstraint::make_shared(
    source,
    *position1,
    *orientation1,
    *position2,
    *orientation2,
    pose_relative_mean_partial,
    pose_relative_covariance_partial,
    position_indices,
    orientation_indices);

  transaction.addVariable(position1);
  transaction.addVariable(orientation1);
  transaction.addVariable(position2);
  transaction.addVariable(orientation2);
  transaction.addConstraint(constraint);
  transaction.addInvolvedStamp(pose1.header.stamp);
  transaction.addInvolvedStamp(pose2.header.stamp);

  return true;
}

/**
 * @brief Extracts velocity data from a TwistWithCovarianceStamped and adds that data to a fuse Transaction
 *
 * This method effectively adds two variables (2D linear velocity and 2D angular velocity) and their respective
 * constraints to the given \p transaction. The velocity data is extracted from the \p twist message. Only 2D data is
 * used. The data will be automatically transformed into the \p target_frame before it is used.
 *
 * @param[in] source - The name of the sensor or motion model that generated this constraint
 * @param[in] device_id - The UUID of the machine
 * @param[in] twist - The TwistWithCovarianceStamped message from which we will extract the twist data
 * @param[in] target_frame - The frame ID into which the twist data will be transformed before it is used
 * @param[in] tf_buffer - The transform buffer with which we will lookup the required transform
 * @param[out] transaction - The generated variables and constraints are added to this transaction
 * @return true if any constraints were added, false otherwise
 */
inline bool processTwistWithCovariance(
  const std::string& source,
  const fuse_core::UUID& device_id,
  const geometry_msgs::TwistWithCovarianceStamped& twist,
  const std::string& target_frame,
  const std::vector<size_t>& linear_indices,
  const std::vector<size_t>& angular_indices,
  const tf2_ros::Buffer& tf_buffer,
  fuse_core::Transaction& transaction)
{
  // Make sure we actually have work to do
  if (linear_indices.empty() && angular_indices.empty())
  {
    return false;
  }

  geometry_msgs::TwistWithCovarianceStamped transformed_message;
  transformed_message.header.frame_id = target_frame;

  if (!transformMessage(tf_buffer, twist, transformed_message))
  {
    ROS_ERROR_STREAM("Cannot create constraint from pose message with stamp " << twist.header.stamp);
    return false;
  }

  bool constraints_added = false;

  // Create two absolute constraints
  if (!linear_indices.empty())
  {
    auto velocity_linear =
      fuse_variables::VelocityLinear2DStamped::make_shared(transformed_message.header.stamp, device_id);
    velocity_linear->x() = transformed_message.twist.twist.linear.x;
    velocity_linear->y() = transformed_message.twist.twist.linear.y;

    // Create the mean twist vectors for the constraints
    fuse_core::Vector2d linear_vel_mean;
    linear_vel_mean << transformed_message.twist.twist.linear.x, transformed_message.twist.twist.linear.y;

    // Create the covariances for the constraints
    fuse_core::Matrix2d linear_vel_covariance;
    linear_vel_covariance <<
        transformed_message.twist.covariance[0],
        transformed_message.twist.covariance[1],
        transformed_message.twist.covariance[6],
        transformed_message.twist.covariance[7];

    // Build the sub-vector and sub-matrices based on the requested indices
    fuse_core::VectorXd linear_vel_mean_partial(linear_indices.size());
    fuse_core::MatrixXd linear_vel_covariance_partial(linear_vel_mean_partial.rows(), linear_vel_mean_partial.rows());

    populatePartialMeasurement(
      linear_vel_mean,
      linear_vel_covariance,
      linear_indices,
      linear_vel_mean_partial,
      linear_vel_covariance_partial);

    auto linear_vel_constraint = fuse_constraints::AbsoluteVelocityLinear2DStampedConstraint::make_shared(
      source, *velocity_linear, linear_vel_mean_partial, linear_vel_covariance_partial, linear_indices);

    transaction.addVariable(velocity_linear);
    transaction.addConstraint(linear_vel_constraint);
    constraints_added = true;
  }

  if (!angular_indices.empty())
  {
    // Create the twist variables
    auto velocity_angular =
      fuse_variables::VelocityAngular2DStamped::make_shared(transformed_message.header.stamp, device_id);
    velocity_angular->yaw() = transformed_message.twist.twist.angular.z;

    fuse_core::Vector1d angular_vel_vector;
    angular_vel_vector << transformed_message.twist.twist.angular.z;

    fuse_core::Matrix1d angular_vel_covariance;
    angular_vel_covariance << transformed_message.twist.covariance[35];

    auto angular_vel_constraint = fuse_constraints::AbsoluteVelocityAngular2DStampedConstraint::make_shared(
      source, *velocity_angular, angular_vel_vector, angular_vel_covariance, angular_indices);

    transaction.addVariable(velocity_angular);
    transaction.addConstraint(angular_vel_constraint);
    constraints_added = true;
  }

  if (constraints_added)
  {
    transaction.addInvolvedStamp(twist.header.stamp);
  }

  return constraints_added;
}

/**
 * @brief Extracts linear acceleration data from an AccelWithCovarianceStamped and adds that data to a fuse Transaction
 *
 * This method effectively adds a linear acceleration variable and constraint to the given to the given \p transaction.
 * The acceleration data is extracted from the \p acceleration message. Only 2D data is used. The data will be
 * automatically transformed into the \p target_frame before it is used.
 *
 * @param[in] source - The name of the sensor or motion model that generated this constraint
 * @param[in] device_id - The UUID of the machine
 * @param[in] acceleration - The AccelWithCovarianceStamped message from which we will extract the acceleration data
 * @param[in] target_frame - The frame ID into which the acceleration data will be transformed before it is used
 * @param[in] tf_buffer - The transform buffer with which we will lookup the required transform
 * @param[out] transaction - The generated variables and constraints are added to this transaction
 * @return true if any constraints were added, false otherwise
 */
inline bool processAccelWithCovariance(
  const std::string& source,
  const fuse_core::UUID& device_id,
  const geometry_msgs::AccelWithCovarianceStamped& acceleration,
  const std::string& target_frame,
  const std::vector<size_t>& indices,
  const tf2_ros::Buffer& tf_buffer,
  fuse_core::Transaction& transaction)
{
  // Make sure we actually have work to do
  if (indices.empty())
  {
    return false;
  }

  geometry_msgs::AccelWithCovarianceStamped transformed_message;
  transformed_message.header.frame_id = target_frame;

  if (!transformMessage(tf_buffer, acceleration, transformed_message))
  {
    ROS_ERROR_STREAM("Cannot create constraint from pose message with stamp " << acceleration.header.stamp);
    return false;
  }

  // Create the acceleration variables
  auto acceleration_linear =
    fuse_variables::AccelerationLinear2DStamped::make_shared(transformed_message.header.stamp, device_id);
  acceleration_linear->x() = transformed_message.accel.accel.linear.x;
  acceleration_linear->y() = transformed_message.accel.accel.linear.y;

  // Create the full mean vector and covariance for the constraint
  fuse_core::Vector2d accel_mean;
  accel_mean << transformed_message.accel.accel.linear.x, transformed_message.accel.accel.linear.y;

  fuse_core::Matrix2d accel_covariance;
  accel_covariance <<
      transformed_message.accel.covariance[0],
      transformed_message.accel.covariance[1],
      transformed_message.accel.covariance[6],
      transformed_message.accel.covariance[7];

  // Build the sub-vector and sub-matrices based on the requested indices
  fuse_core::VectorXd accel_mean_partial(indices.size());
  fuse_core::MatrixXd accel_covariance_partial(accel_mean_partial.rows(), accel_mean_partial.rows());

  populatePartialMeasurement(accel_mean, accel_covariance, indices, accel_mean_partial, accel_covariance_partial);

  // Create the constraint
  auto linear_accel_constraint = fuse_constraints::AbsoluteAccelerationLinear2DStampedConstraint::make_shared(
    source,
    *acceleration_linear,
    accel_mean_partial,
    accel_covariance_partial,
    indices);

  transaction.addVariable(acceleration_linear);
  transaction.addConstraint(linear_accel_constraint);
  transaction.addInvolvedStamp(acceleration.header.stamp);

  return true;
}

}  // namespace common

}  // namespace fuse_models

#endif  // FUSE_MODELS_COMMON_SENSOR_PROC_H
