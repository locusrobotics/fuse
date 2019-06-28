/***************************************************************************
 * Copyright (C) 2019 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#ifndef FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H
#define FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <algorithm>
#include <string>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief Defines the set of parameters required by the fuse_optimizers::FixedLagSmoother class
 */
struct FixedLagSmootherParams
{
public:
  /**
   * @brief The set of sensors whose transactions will trigger the optimizer thread to start running
   *
   * This is designed to keep the system idle until the origin constraint has been received.
   */
  std::vector<std::string> ignition_sensors;

  /**
   * @brief The duration of the smoothing window in seconds
   */
  ros::Duration lag_duration { 5.0 };

  /**
   * @brief The target duration for optimization cycles
   *
   * If an optimization takes longer than expected, an optimization cycle may be skipped. The optimization period
   * may be specified in either the "optimization_period" parameter in seconds, or in the "optimization_frequency"
   * parameter in Hz.
   */
  ros::Duration optimization_period { 0.1 };

  /**
   * @brief The topic name of the advertised reset service
   */
  std::string reset_service { "~reset" };

  /**
   * @brief The maximum time to wait for motion models to be generated for a received transaction.
   *
   * Transactions are processed sequentially, so no new transactions will be added to the graph while waiting for
   * motion models to be generated. Once the timeout expires, that transaction will be deleted from the queue.
   */
  ros::Duration transaction_timeout { 0.1 };

  /**
   * @brief Helper function that loads strictly positive floating point values from the parameter server
   *
   * @param[in] node_handle - The node handle used to load the parameter
   * @param[in] parameter_name - The parameter name to load
   * @param[in] default_value - A default value to use if the provided parameter name does not exist
   * @return The loaded (or default) value
   */
  double getPositiveParam(const ros::NodeHandle& node_handle, const std::string& parameter_name, double default_value)
  {
    double value;
    node_handle.param(parameter_name, value, default_value);
    if (value <= 0)
    {
      ROS_WARN_STREAM("The requested " << parameter_name << " is <= 0. Using the default value (" <<
                      default_value << ") instead.");
      value = default_value;
    }
    return value;
  }

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh)
  {
    // Read settings from the parameter server
    nh.getParam("ignition_sensors", ignition_sensors);
    std::sort(ignition_sensors.begin(), ignition_sensors.end());

    auto lag_duration_sec = getPositiveParam(nh, "lag_duration", lag_duration.toSec());
    lag_duration.fromSec(lag_duration_sec);

    if (nh.hasParam("optimization_frequency"))
    {
      auto optimization_frequency = getPositiveParam(nh, "optimization_frequency", 1.0 / optimization_period.toSec());
      optimization_period.fromSec(1.0 / optimization_frequency);
    }
    else
    {
      auto optimization_period_sec = getPositiveParam(nh, "optimization_period", optimization_period.toSec());
      optimization_period.fromSec(optimization_period_sec);
    }

    nh.getParam("reset_service", reset_service);

    auto transaction_timeout_sec = getPositiveParam(nh, "transaction_timeout", transaction_timeout.toSec());
    transaction_timeout.fromSec(transaction_timeout_sec);
  }
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H
