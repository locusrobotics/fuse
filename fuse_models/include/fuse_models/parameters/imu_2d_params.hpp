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
#ifndef FUSE_MODELS__PARAMETERS__IMU_2D_PARAMS_HPP_
#define FUSE_MODELS__PARAMETERS__IMU_2D_PARAMS_HPP_

#include <string>
#include <vector>

#include <fuse_models/parameters/parameter_base.hpp>

#include <fuse_core/loss.hpp>
#include <fuse_core/parameter.hpp>
#include <fuse_variables/acceleration_linear_2d_stamped.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/velocity_angular_2d_stamped.hpp>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Imu2D class
 */
struct Imu2DParams : public ParameterBase
{
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] interfaces - The node interfaces with which to load parameters
   * @param[in] ns - The parameter namespace to use
   */
  void loadFromROS(
    fuse_core::node_interfaces::NodeInterfaces<
      fuse_core::node_interfaces::Base,
      fuse_core::node_interfaces::Logging,
      fuse_core::node_interfaces::Parameters
    > interfaces,
    const std::string & ns)
  {
    angular_velocity_indices =
      loadSensorConfig<fuse_variables::VelocityAngular2DStamped>(
      interfaces, fuse_core::joinParameterName(
        ns,
        "angular_velocity_dimensions"));
    linear_acceleration_indices =
      loadSensorConfig<fuse_variables::AccelerationLinear2DStamped>(
      interfaces, fuse_core::joinParameterName(
        ns,
        "linear_acceleration_dimensions"));
    orientation_indices = loadSensorConfig<fuse_variables::Orientation2DStamped>(
      interfaces, fuse_core::joinParameterName(
        ns,
        "orientation_dimensions"));

    differential = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "differential"),
      differential);
    disable_checks =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "disable_checks"),
      disable_checks);
    queue_size = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "queue_size"),
      queue_size);
    fuse_core::getPositiveParam(interfaces, "tf_timeout", tf_timeout, false);

    fuse_core::getPositiveParam(interfaces, "throttle_period", throttle_period, false);
    throttle_use_wall_time =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "throttle_use_wall_time"),
      throttle_use_wall_time);

    remove_gravitational_acceleration = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns, "remove_gravitational_acceleration"), remove_gravitational_acceleration);
    gravitational_acceleration =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "gravitational_acceleration"),
      gravitational_acceleration);
    fuse_core::getParamRequired(interfaces, fuse_core::joinParameterName(ns, "topic"), topic);

    if (differential) {
      independent = fuse_core::getParam(
        interfaces, fuse_core::joinParameterName(
          ns,
          "independent"),
        independent);
      use_twist_covariance =
        fuse_core::getParam(
        interfaces, fuse_core::joinParameterName(
          ns,
          "use_twist_covariance"),
        use_twist_covariance);

      minimum_pose_relative_covariance =
        fuse_core::getCovarianceDiagonalParam<3>(
        interfaces,
        fuse_core::joinParameterName(ns, "minimum_pose_relative_covariance_diagonal"), 0.0);
      twist_covariance_offset =
        fuse_core::getCovarianceDiagonalParam<3>(
        interfaces,
        fuse_core::joinParameterName(ns, "twist_covariance_offset_diagonal"), 0.0);
    }

    acceleration_target_frame =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "acceleration_target_frame"),
      acceleration_target_frame);
    orientation_target_frame =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "orientation_target_frame"),
      orientation_target_frame);
    twist_target_frame =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "twist_target_frame"),
      twist_target_frame);

    pose_loss =
      fuse_core::loadLossConfig(interfaces, fuse_core::joinParameterName(ns, "pose_loss"));
    angular_velocity_loss =
      fuse_core::loadLossConfig(
      interfaces, fuse_core::joinParameterName(
        ns,
        "angular_velocity_loss"));
    linear_acceleration_loss =
      fuse_core::loadLossConfig(
      interfaces,
      fuse_core::joinParameterName(ns, "linear_acceleration_loss"));
  }

  bool differential {false};
  bool disable_checks {false};
  bool independent {true};
  bool use_twist_covariance {true};
  fuse_core::Matrix3d minimum_pose_relative_covariance;  //!< Minimum pose relative covariance
                                                         //!< matrix
  fuse_core::Matrix3d twist_covariance_offset;    //!< Offset already added to the twist covariance
                                                  //!< matrix, that will be substracted in order to
                                                  //!< recover the raw values
  bool remove_gravitational_acceleration {false};
  int queue_size {10};
  rclcpp::Duration tf_timeout {0, 0};  //!< The maximum time to wait for a transform to become
                                       //!< available
  rclcpp::Duration throttle_period {0, 0};  //!< The throttle period duration in seconds
  bool throttle_use_wall_time {false};      //!< Whether to throttle using ros::WallTime or not
  double gravitational_acceleration {9.80665};
  std::string acceleration_target_frame {};
  std::string orientation_target_frame {};
  std::string topic {};
  std::string twist_target_frame {};
  std::vector<size_t> angular_velocity_indices;
  std::vector<size_t> linear_acceleration_indices;
  std::vector<size_t> orientation_indices;
  fuse_core::Loss::SharedPtr pose_loss;
  fuse_core::Loss::SharedPtr angular_velocity_loss;
  fuse_core::Loss::SharedPtr linear_acceleration_loss;
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS__PARAMETERS__IMU_2D_PARAMS_HPP_
