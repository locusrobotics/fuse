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
#ifndef FUSE_MODELS_PARAMETERS_ODOMETRY_2D_PUBLISHER_PARAMS_H
#define FUSE_MODELS_PARAMETERS_ODOMETRY_2D_PUBLISHER_PARAMS_H

#include <fuse_models/parameters/parameter_base.h>

#include <fuse_core/ceres_options.h>
#include <fuse_core/eigen.h>
#include <fuse_core/parameter.h>

#include <ros/console.h>
#include <ros/node_handle.h>

#include <ceres/covariance.h>

#include <algorithm>
#include <cassert>
#include <string>
#include <vector>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Odometry2DPublisher class
 */
struct Odometry2DPublisherParams : public ParameterBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final
  {
    nh.getParam("publish_tf", publish_tf);
    nh.getParam("invert_tf", invert_tf);
    nh.getParam("predict_to_current_time", predict_to_current_time);
    nh.getParam("predict_with_acceleration", predict_with_acceleration);
    nh.getParam("publish_frequency", publish_frequency);

    process_noise_covariance = fuse_core::getCovarianceDiagonalParam<8>(nh, "process_noise_diagonal", 0.0);
    nh.param("scale_process_noise", scale_process_noise, scale_process_noise);
    nh.param("velocity_norm_min", velocity_norm_min, velocity_norm_min);

    double tf_cache_time_double = tf_cache_time.toSec();
    nh.getParam("tf_cache_time", tf_cache_time_double);
    tf_cache_time.fromSec(tf_cache_time_double);

    double tf_timeout_double = tf_timeout.toSec();
    nh.getParam("tf_timeout", tf_timeout_double);
    tf_timeout.fromSec(tf_timeout_double);

    nh.getParam("queue_size", queue_size);

    nh.getParam("map_frame_id", map_frame_id);
    nh.getParam("odom_frame_id", odom_frame_id);
    nh.getParam("base_link_frame_id", base_link_frame_id);
    nh.param("base_link_output_frame_id", base_link_output_frame_id, base_link_frame_id);
    nh.param("world_frame_id", world_frame_id, odom_frame_id);

    const bool frames_valid =
      map_frame_id != odom_frame_id &&
      map_frame_id != base_link_frame_id &&
      map_frame_id != base_link_output_frame_id &&
      odom_frame_id != base_link_frame_id &&
      odom_frame_id != base_link_output_frame_id &&
      (world_frame_id == map_frame_id || world_frame_id == odom_frame_id);

    if (!frames_valid)
    {
      ROS_FATAL_STREAM("Invalid frame configuration! Please note:\n" <<
        " - The values for map_frame_id, odom_frame_id, and base_link_frame_id must be unique\n" <<
        " - The values for map_frame_id, odom_frame_id, and base_link_output_frame_id must be unique\n" <<
        " - The world_frame_id must be the same as the map_frame_id or odom_frame_id\n");

      assert(frames_valid);
    }

    nh.getParam("topic", topic);
    nh.getParam("acceleration_topic", acceleration_topic);

    fuse_core::loadCovarianceOptionsFromROS(ros::NodeHandle(nh, "covariance_options"), covariance_options);
  }

  bool publish_tf { true };  //!< Whether to publish/broadcast the TF transform or not
  bool invert_tf{ false };   //!< Whether to broadcast the inverse of the TF transform or not. When the inverse is
                             //!< broadcasted, the transform is inverted and the header.frame_id and child_frame_id are
                             //!< swapped, i.e. the odometry output header.frame_id is set to the
                             //!< base_link_output_frame_id and the child_frame_id to the world_frame_id
  bool predict_to_current_time { false };
  bool predict_with_acceleration { false };
  double publish_frequency { 10.0 };
  fuse_core::Matrix8d process_noise_covariance;   //!< Process noise covariance matrix
  bool scale_process_noise{ false };
  double velocity_norm_min{ 1e-3 };
  ros::Duration tf_cache_time { 10.0 };
  ros::Duration tf_timeout { 0.1 };
  int queue_size { 1 };
  std::string map_frame_id { "map" };
  std::string odom_frame_id { "odom" };
  std::string base_link_frame_id { "base_link" };
  std::string base_link_output_frame_id { base_link_frame_id };
  std::string world_frame_id { odom_frame_id };
  std::string topic { "odometry/filtered" };
  std::string acceleration_topic { "acceleration/filtered" };
  ceres::Covariance::Options covariance_options;
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS_PARAMETERS_ODOMETRY_2D_PUBLISHER_PARAMS_H
