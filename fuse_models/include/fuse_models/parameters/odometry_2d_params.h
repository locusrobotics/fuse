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
#ifndef FUSE_MODELS_PARAMETERS_ODOMETRY_2D_PARAMS_H
#define FUSE_MODELS_PARAMETERS_ODOMETRY_2D_PARAMS_H

#include <fuse_models/parameters/parameter_base.h>

#include <fuse_core/loss.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <fuse_variables/velocity_angular_2d_stamped.h>
#include <fuse_variables/velocity_linear_2d_stamped.h>
#include <ros/node_handle.h>

#include <string>
#include <vector>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Odometry2D class
 */
struct Odometry2DParams : public ParameterBase
{
  public:
    /**
     * @brief Method for loading parameter values from ROS.
     *
     * @param[in] nh - The ROS node handle with which to load parameters
     */
    void loadFromROS(const ros::NodeHandle& nh) final
    {
      position_indices = loadSensorConfig<fuse_variables::Position2DStamped>(nh, "position_dimensions");
      orientation_indices = loadSensorConfig<fuse_variables::Orientation2DStamped>(nh, "orientation_dimensions");
      linear_velocity_indices =
        loadSensorConfig<fuse_variables::VelocityLinear2DStamped>(nh, "linear_velocity_dimensions");
      angular_velocity_indices =
        loadSensorConfig<fuse_variables::VelocityAngular2DStamped>(nh, "angular_velocity_dimensions");

      nh.getParam("differential", differential);
      nh.getParam("disable_checks", disable_checks);
      nh.getParam("queue_size", queue_size);
      getParamRequired(nh, "topic", topic);
      getParamRequired(nh, "twist_target_frame", twist_target_frame);

      if (!differential)
      {
        getParamRequired(nh, "pose_target_frame", pose_target_frame);
      }
      else
      {
        nh.getParam("independent", independent);

        if (!independent)
        {
          nh.getParam("use_twist_covariance", use_twist_covariance);

          std::vector<double> minimum_pose_relative_covariance_diagonal(3, 0.0);
          nh.param("minimum_pose_relative_covariance_diagonal", minimum_pose_relative_covariance_diagonal,
                   minimum_pose_relative_covariance_diagonal);

          if (minimum_pose_relative_covariance_diagonal.size() != 3)
          {
            throw std::runtime_error("Minimum pose relative covariance diagonal must be of length 3!");
          }

          if (std::any_of(minimum_pose_relative_covariance_diagonal.begin(),
                          minimum_pose_relative_covariance_diagonal.end(),
                          [](const auto& v) { return v < 0.0; }))  // NOLINT(whitespace/braces)
          {
            throw std::runtime_error("All minimum pose relative covariance diagonal entries must be positive!");
          }

          minimum_pose_relative_covariance =
              fuse_core::Vector3d(minimum_pose_relative_covariance_diagonal.data()).asDiagonal();
        }
      }

      pose_loss = loadLossConfig(nh, "pose_loss");
      linear_velocity_loss = loadLossConfig(nh, "linear_velocity_loss");
      angular_velocity_loss = loadLossConfig(nh, "angular_velocity_loss");
    }

    bool differential { false };
    bool disable_checks { false };
    bool independent { true };
    bool use_twist_covariance { true };
    fuse_core::Matrix3d minimum_pose_relative_covariance;  //!< Minimum pose relative covariance matrix
    int queue_size { 10 };
    std::string topic {};
    std::string pose_target_frame {};
    std::string twist_target_frame {};
    std::vector<size_t> position_indices;
    std::vector<size_t> orientation_indices;
    std::vector<size_t> linear_velocity_indices;
    std::vector<size_t> angular_velocity_indices;
    fuse_core::Loss::SharedPtr pose_loss;
    fuse_core::Loss::SharedPtr linear_velocity_loss;
    fuse_core::Loss::SharedPtr angular_velocity_loss;
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS_PARAMETERS_ODOMETRY_2D_PARAMS_H
