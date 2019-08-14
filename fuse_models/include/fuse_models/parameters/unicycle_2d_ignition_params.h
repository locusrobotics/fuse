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
#ifndef FUSE_MODELS_PARAMETERS_UNICYCLE_2D_IGNITION_PARAMS_H
#define FUSE_MODELS_PARAMETERS_UNICYCLE_2D_IGNITION_PARAMS_H

#include <fuse_models/parameters/parameter_base.h>

#include <ros/node_handle.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Unicycle2DIgnition class
 */
struct Unicycle2DIgnitionParams : public ParameterBase
{
  public:
    /**
     * @brief Method for loading parameter values from ROS.
     *
     * @param[in] nh - The ROS node handle with which to load parameters
     */
    void loadFromROS(const ros::NodeHandle& nh) final
    {
      nh.getParam("publish_on_startup", publish_on_startup);
      nh.getParam("queue_size", queue_size);
      nh.getParam("reset_service", reset_service);
      nh.getParam("set_pose_service", set_pose_service);
      nh.getParam("set_pose_deprecated_service", set_pose_deprecated_service);
      nh.getParam("topic", topic);

      std::vector<double> sigma_vector;
      if (nh.getParam("initial_sigma", sigma_vector))
      {
        if (sigma_vector.size() != 8)
        {
          throw std::invalid_argument("The supplied initial_sigma parameter must be length 8, but is actually length " +
                                      std::to_string(sigma_vector.size()));
        }
        auto is_sigma_valid = [](const double sigma)
        {
          return std::isfinite(sigma) && (sigma > 0);
        };
        if (!std::all_of(sigma_vector.begin(), sigma_vector.end(), is_sigma_valid))
        {
          throw std::invalid_argument("The supplied initial_sigma parameter must contain valid floating point values. "
                                      "NaN, Inf, and values <= 0 are not acceptable.");
        }
        initial_sigma.swap(sigma_vector);
      }

      std::vector<double> state_vector;
      if (nh.getParam("initial_state", state_vector))
      {
        if (state_vector.size() != 8)
        {
          throw std::invalid_argument("The supplied initial_state parameter must be length 8, but is actually length " +
                                      std::to_string(state_vector.size()));
        }
        auto is_state_valid = [](const double state)
        {
          return std::isfinite(state);
        };
        if (!std::all_of(state_vector.begin(), state_vector.end(), is_state_valid))
        {
          throw std::invalid_argument("The supplied initial_state parameter must contain valid floating point values. "
                                      "NaN, Inf, etc are not acceptable.");
        }
        initial_state.swap(state_vector);
      }
    }


    /**
     * @brief Flag indicating if an initial state transaction should be sent on startup, or only in response to a
     *        set_pose service call or topic message.
     */
    bool publish_on_startup { true };

    /**
     * @brief The size of the subscriber queue for the set_pose topic
     */
    int queue_size { 10 };

    /**
     * @brief The name of the reset service to call before sending transactions to the optimizer
     */
    std::string reset_service { "~reset" };

    /**
     * @brief The name of the set_pose service to advertise
     */
    std::string set_pose_service { "~set_pose" };

    /**
     * @brief The name of the deprecated set_pose service without return codes
     */
    std::string set_pose_deprecated_service { "~set_pose_deprecated" };

    /**
     * @brief The topic name for received PoseWithCovarianceStamped messages
     */
    std::string topic { "~set_pose" };

    /**
     * @brief The uncertainty of the initial state value
     *
     * Standard deviations are provided as an 8-dimensional vector in the order:
     *   (x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
     * The covariance matrix is created placing the squared standard deviations along the diagonal of an 8x8 matrix.
     */
    std::vector<double> initial_sigma {1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9};

    /**
     * @brief The initial value of the 8-dimension state vector (x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
     */
    std::vector<double> initial_state {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS_PARAMETERS_UNICYCLE_2D_IGNITION_PARAMS_H
