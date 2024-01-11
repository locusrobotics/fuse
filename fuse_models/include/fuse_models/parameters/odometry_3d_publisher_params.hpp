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
#ifndef FUSE_MODELS__PARAMETERS__ODOMETRY_3D_PUBLISHER_PARAMS_HPP_
#define FUSE_MODELS__PARAMETERS__ODOMETRY_3D_PUBLISHER_PARAMS_HPP_

#include <ceres/covariance.h>

#include <algorithm>
#include <cassert>
#include <string>
#include <vector>

#include <fuse_models/parameters/parameter_base.hpp>

#include <fuse_core/ceres_options.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/parameter.hpp>

#include <rclcpp/logging.hpp>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Odometry3DPublisher class
 */
struct Odometry3DPublisherParams : public ParameterBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
    publish_tf = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "publish_tf"),
      publish_tf);
    invert_tf = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "invert_tf"),
      invert_tf);
    predict_to_current_time =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "predict_to_current_time"),
      predict_to_current_time);
    predict_with_acceleration =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "predict_with_acceleration"),
      predict_with_acceleration);
    publish_frequency =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "publish_frequency"),
      publish_frequency);

    process_noise_covariance = fuse_core::getCovarianceDiagonalParam<15>(
      interfaces, fuse_core::joinParameterName(
        ns,
        "process_noise_diagonal"), 0.0);
    scale_process_noise =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "scale_process_noise"),
      scale_process_noise);
    velocity_linear_norm_min_ =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "velocity_linear_norm_min"),
      velocity_linear_norm_min_);
    velocity_angular_norm_min_ =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "velocity_angular_norm_min"),
      velocity_angular_norm_min_);
    fuse_core::getPositiveParam(
      interfaces,
      fuse_core::joinParameterName(
        ns,
        "covariance_throttle_period"), covariance_throttle_period,
      false);

    fuse_core::getPositiveParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "tf_cache_time"), tf_cache_time,
      false);
    fuse_core::getPositiveParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "tf_timeout"), tf_timeout,
      false);

    queue_size = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "queue_size"),
      queue_size);

    map_frame_id = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "map_frame_id"),
      map_frame_id);
    odom_frame_id = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "odom_frame_id"),
      odom_frame_id);
    base_link_frame_id =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "base_link_frame_id"),
      base_link_frame_id);
    base_link_output_frame_id =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "base_link_output_frame_id"),
      base_link_output_frame_id);
    world_frame_id =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "world_frame_id"),
      world_frame_id);

    const bool frames_valid =
      map_frame_id != odom_frame_id &&
      map_frame_id != base_link_frame_id &&
      map_frame_id != base_link_output_frame_id &&
      odom_frame_id != base_link_frame_id &&
      odom_frame_id != base_link_output_frame_id &&
      (world_frame_id == map_frame_id || world_frame_id == odom_frame_id);

    if (!frames_valid) {
      RCLCPP_FATAL_STREAM(
        interfaces.get_node_logging_interface()->get_logger(),
        "Invalid frame configuration! Please note:\n"
          << " - The values for map_frame_id, odom_frame_id, and base_link_frame_id must be "
          << "unique\n"
          << " - The values for map_frame_id, odom_frame_id, and base_link_output_frame_id must be "
          << "unique\n"
          << " - The world_frame_id must be the same as the map_frame_id or odom_frame_id\n");

      assert(frames_valid);
    }

    topic = fuse_core::getParam(interfaces, fuse_core::joinParameterName(ns, "topic"), topic);
    acceleration_topic =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "acceleration_topic"),
      acceleration_topic);

    fuse_core::loadCovarianceOptionsFromROS(interfaces, covariance_options, "covariance_options");
  }

  bool publish_tf {true};    //!< Whether to publish/broadcast the TF transform or not
  bool invert_tf{false};     //!< Whether to broadcast the inverse of the TF transform or not. When
                             //!< the inverse is broadcasted, the transform is inverted and the
                             //!< header.frame_id and child_frame_id are swapped, i.e. the odometry
                             //!< output header.frame_id is set to the base_link_output_frame_id and
                             //!< the child_frame_id to the world_frame_id
  bool predict_to_current_time {false};
  bool predict_with_acceleration {false};
  double publish_frequency {10.0};
  fuse_core::Matrix15d process_noise_covariance;   //!< Process noise covariance matrix
  bool scale_process_noise{false};
  double velocity_linear_norm_min_{1e-3};
  double velocity_angular_norm_min_{1e-3};
  rclcpp::Duration covariance_throttle_period {0, 0};  //!< The throttle period duration in seconds
                                                       //!< to compute the covariance
  rclcpp::Duration tf_cache_time {10, 0};
  rclcpp::Duration tf_timeout {0, static_cast<uint32_t>(RCUTILS_S_TO_NS(0.1))};
  int queue_size {1};
  std::string map_frame_id {"map"};
  std::string odom_frame_id {"odom"};
  std::string base_link_frame_id {"base_link"};
  std::string base_link_output_frame_id {base_link_frame_id};
  std::string world_frame_id {odom_frame_id};
  std::string topic {"odometry/filtered"};
  std::string acceleration_topic {"acceleration/filtered"};
  ceres::Covariance::Options covariance_options;
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS__PARAMETERS__ODOMETRY_3D_PUBLISHER_PARAMS_HPP_
