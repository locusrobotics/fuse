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
#ifndef FUSE_MODELS_PARAMETERS_IMU_2D_PARAMS_H
#define FUSE_MODELS_PARAMETERS_IMU_2D_PARAMS_H

#include <fuse_models/parameters/parameter_base.h>

#include <fuse_variables/acceleration_linear_2d_stamped.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/velocity_angular_2d_stamped.h>
#include <ros/node_handle.h>

#include <string>
#include <vector>


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
     * @param[in] nh - The ROS node handle with which to load parameters
     */
    void loadFromROS(const ros::NodeHandle& nh) final
    {
      angular_velocity_indices =
        loadSensorConfig<fuse_variables::VelocityAngular2DStamped>(nh, "angular_velocity_dimensions");
      linear_acceleration_indices =
        loadSensorConfig<fuse_variables::AccelerationLinear2DStamped>(nh, "linear_acceleration_dimensions");
      orientation_indices = loadSensorConfig<fuse_variables::Orientation2DStamped>(nh, "orientation_dimensions");

      nh.getParam("differential", differential);
      nh.getParam("queue_size", queue_size);
      nh.getParam("remove_gravitational_acceleration", remove_gravitational_acceleration);
      nh.getParam("gravitational_acceleration", gravitational_acceleration);
      getParamRequired(nh, "topic", topic);

      if (!linear_acceleration_indices.empty())
      {
        getParamRequired(nh, "acceleration_target_frame", acceleration_target_frame);
      }

      if (!orientation_indices.empty())
      {
        getParamRequired(nh, "orientation_target_frame", orientation_target_frame);
      }

      if (!angular_velocity_indices.empty())
      {
        getParamRequired(nh, "twist_target_frame", twist_target_frame);
      }
    }

    bool differential { false };
    bool remove_gravitational_acceleration { false };
    int queue_size { 10 };
    double gravitational_acceleration { 9.80665 };
    std::string acceleration_target_frame {};
    std::string orientation_target_frame {};
    std::string topic {};
    std::string twist_target_frame {};
    std::vector<size_t> angular_velocity_indices;
    std::vector<size_t> linear_acceleration_indices;
    std::vector<size_t> orientation_indices;
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS_PARAMETERS_IMU_2D_PARAMS_H
