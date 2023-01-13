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
#include <fuse_tutorials/range_sensor_model.h>

#include <fuse_constraints/absolute_constraint.hpp>
#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/sensor_model.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_tutorials/range_constraint.h>
#include <fuse_variables/point_2d_landmark.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_tutorials::RangeSensorModel, fuse_core::SensorModel);

namespace fuse_tutorials
{
void RangeSensorModel::priorBeaconsCallback(const sensor_msgs::msg::PointCloud2& msg)
{
  // Store a copy of the beacon database. We use a map to allow efficient lookups by ID number.
  sensor_msgs::msg::PointCloud2ConstIterator<float> x_it(msg, "x");
  sensor_msgs::msg::PointCloud2ConstIterator<float> y_it(msg, "y");
  sensor_msgs::msg::PointCloud2ConstIterator<float> z_it(msg, "z");
  sensor_msgs::msg::PointCloud2ConstIterator<float> sigma_it(msg, "sigma");
  sensor_msgs::msg::PointCloud2ConstIterator<unsigned int> id_it(msg, "id");
  for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++sigma_it, ++id_it)
  {
    beacon_db_[*id_it] = Beacon { *x_it, *y_it, *sigma_it };
  }
  RCLCPP_INFO_STREAM(logger_, "Updated Beacon Database.");
}

void RangeSensorModel::onInit()
{
  // Read settings from the parameter server, or any other one-time operations. This sensor model doesn't have any
  // user configuration to read. But we do need a copy of the beacon database. We will subscribe to that now, as it
  // is assumed to be constant -- no need to clear it if the optimizer is reset.
  beacon_sub_ = node_handle_.subscribe("prior_beacons", 10, &RangeSensorModel::priorBeaconsCallback, this);
}

void RangeSensorModel::onStart()
{
  // Ensure the "initialized" flag is false at the beginning. This is what triggers the beacon database information
  // to be sent to the optimizer. This needs to happen once and only once for each run of the optimizer. By clearing
  // that flag in onStart(), we ensure the database information will be processed after every optimizer reset.
  initialized_ = false;

  // Subscribe to the ranges topic. Any received messages will be processed within the message callback function,
  // and the created constraints will be sent to the optimizer. By subscribing to the topic in onStart() and
  // unsubscribing in onStop(), we will only send transactions to the optimizer while it is running.
  sub_ = node_handle_.subscribe("ranges", 10, &RangeSensorModel::rangesCallback, this);
}

void RangeSensorModel::onStop()
{
  // Unsubscribe from the ranges topic. Since the sensor constraints are created and sent from the subscriber callback,
  // shutting down the subscriber effectively stops the creation of new constraints from this sensor model. This
  // ensures we only send transactions to the optimizer while it is running.
  sub_.reset();
}

void RangeSensorModel::rangesCallback(const sensor_msgs::msg::PointCloud2& msg)
{
  // We received a new message for our sensor. This is where most of the processing happens for our sensor model. We
  // take the published ROS message and transform it into one or more Constraints, and send them to the optimizer.

  // However, if we have not received the prior beacon positions yet, we cannot process the range messages.
  if (beacon_db_.empty())
  {
    return;
  }

  // Create a transaction object. This is used to package all of the generated constraints from a given timestamp
  // into one update operation for the optimizer.
  auto transaction = fuse_core::Transaction::make_shared();
  // Each transaction has a timestamp. This is used by the optimizer to determine what order the sensor transactions
  // should be added to the graph. Unless you have a very specific reason not to, the transaction timestamp should be
  // the same as the sensor data timestamp. Or rclcpp::Clock(RCL_SYSTEM_TIME).now() if the sensor data is not stamped.
  transaction->stamp(msg.header.stamp);

  // All of the measured range constraints will involve the robot position at the pointcloud message timestamp.
  // Construct a robot position variable at that timestamp now.
  auto robot_position = fuse_variables::Position2DStamped::make_shared(msg.header.stamp);
  // The transaction needs to know about all of the involved variables as well as the constraints, so insert the robot
  // position variable now.
  transaction->addVariable(robot_position);
  // Additionally the transaction needs to know about all of the individual timestamps involved in this transaction.
  // Since not every variable is associated with a timestamp, I could not work out a way to populate this list
  // automatically. The robot pose is the only stamped variable involved, so add that timestamp now as well.
  transaction->addInvolvedStamp(msg.header.stamp);

  // Loop over the pointcloud, extracting the beacon ID, range, and measurement uncertainty for each detected beacon
  sensor_msgs::msg::PointCloud2ConstIterator<unsigned int> id_it(msg, "id");
  sensor_msgs::msg::PointCloud2ConstIterator<double> range_it(msg, "range");
  sensor_msgs::msg::PointCloud2ConstIterator<double> sigma_it(msg, "sigma");
  for (; id_it != id_it.end(); ++id_it, ++range_it, ++sigma_it)
  {
    // Each measure range will involve a different observed beacon. Construct a variable for this
    // measurement's beacon.
    auto beacon = beacon_db_[*id_it];
    auto beacon_position = fuse_variables::Point2DLandmark::make_shared(*id_it);
    beacon_position->x() = beacon.x;
    beacon_position->y() = beacon.y;
    transaction->addVariable(beacon_position);

    // Now that we have the involved variables defined, create a constraint for this sensor measurement
    auto constraint = fuse_tutorials::RangeConstraint::make_shared(
      this->name(),
      *robot_position,
      *beacon_position,
      *range_it,
      *sigma_it);
    transaction->addConstraint(constraint);

    // If this is the very first measurement, add a prior position constraint on all of the beacons as well. This
    // captures the prior information we know from the database. It also fully constrains the beacon estimate within
    // the optimizer. Our beacon measurement is rank-deficient (one range measurement but two degrees of freedom).
    // Without adding this prior, the optimizer would be unable to solve the optimization problem. In the absence of
    // a prior database, the beacons could be tracked over multiple samples. Once the beacon has been observed with
    // enough parallax, then all measurements of that beacon could be added at once. Some type of delayed
    // initialization scheme is common in vision-based odometry and SLAM systems.
    if (!initialized_)
    {
      auto mean = fuse_core::Vector2d(beacon.x, beacon.y);
      auto cov = fuse_core::Matrix2d();
      cov << beacon.sigma * beacon.sigma, 0.0, 0.0, beacon.sigma * beacon.sigma;
      auto prior = fuse_constraints::AbsoluteConstraint<fuse_variables::Point2DLandmark>::make_shared(
        this->name(),
        *beacon_position,
        mean,
        cov);
      transaction->addConstraint(prior);
    }
  }
  initialized_ = true;

  // Send the transaction object to the optimizer
  sendTransaction(transaction);
}

}  // namespace fuse_tutorials
