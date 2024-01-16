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
#ifndef FUSE_MODELS__PARAMETERS__UNICYCLE_3D_IGNITION_PARAMS_HPP_
#define FUSE_MODELS__PARAMETERS__UNICYCLE_3D_IGNITION_PARAMS_HPP_

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <fuse_models/parameters/parameter_base.hpp>

#include <fuse_core/loss.hpp>
#include <fuse_core/parameter.hpp>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Omnidirectional3DIgnition class
 */
struct Omnidirectional3DIgnitionParams : public ParameterBase
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
    publish_on_startup =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "publish_on_startup"),
      publish_on_startup);
    queue_size = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "queue_size"),
      queue_size);
    reset_service = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "reset_service"),
      reset_service);
    set_pose_service =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "set_pose_service"),
      set_pose_service);
    set_pose_deprecated_service =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "set_pose_deprecated_service"),
      set_pose_deprecated_service);
    topic = fuse_core::getParam(interfaces, fuse_core::joinParameterName(ns, "topic"), topic);

    std::vector<double> sigma_vector;
    sigma_vector = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "initial_sigma"),
      sigma_vector);
    if (!sigma_vector.empty()) {
      if (sigma_vector.size() != 15) {
        throw std::invalid_argument(
                "The supplied initial_sigma parameter must be length 15, but "
                "is actually length " +
                std::to_string(sigma_vector.size()));
      }
      auto is_sigma_valid = [](const double sigma)
        {
          return std::isfinite(sigma) && (sigma > 0);
        };
      if (!std::all_of(sigma_vector.begin(), sigma_vector.end(), is_sigma_valid)) {
        throw std::invalid_argument(
                "The supplied initial_sigma parameter must contain valid floating point values. "
                "NaN, Inf, and values <= 0 are not acceptable.");
      }
      initial_sigma.swap(sigma_vector);
    }

    std::vector<double> state_vector;
    state_vector = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "initial_state"),
      state_vector);
    if (!state_vector.empty()) {
      if (state_vector.size() != 15) {
        throw std::invalid_argument(
                "The supplied initial_state parameter must be length 15, but is actually length " +
                std::to_string(state_vector.size()));
      }
      auto is_state_valid = [](const double state)
        {
          return std::isfinite(state);
        };
      if (!std::all_of(state_vector.begin(), state_vector.end(), is_state_valid)) {
        throw std::invalid_argument(
                "The supplied initial_state parameter must contain valid floating point values. "
                "NaN, Inf, etc are not acceptable.");
      }
      initial_state.swap(state_vector);
    }

    loss = fuse_core::loadLossConfig(interfaces, fuse_core::joinParameterName(ns, "loss"));
  }


  /**
   * @brief Flag indicating if an initial state transaction should be sent on startup, or only in
   *        response to a set_pose service call or topic message.
   */
  bool publish_on_startup {true};

  /**
   * @brief The size of the subscriber queue for the set_pose topic
   */
  int queue_size {10};

  /**
   * @brief The name of the reset service to call before sending transactions to the optimizer
   */
  std::string reset_service {"~/reset"};

  /**
   * @brief The name of the set_pose service to advertise
   */
  std::string set_pose_service {"set_pose"};

  /**
   * @brief The name of the deprecated set_pose service without return codes
   */
  std::string set_pose_deprecated_service {"set_pose_deprecated"};

  /**
   * @brief The topic name for received PoseWithCovarianceStamped messages
   */
  std::string topic {"set_pose"};

  /**
   * @brief The uncertainty of the initial state value
   *
   * Standard deviations are provided as an 15-dimensional vector in the order:
   *   (x, y, z, roll, pitch, yaw, x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel, x_acc, y_acc, z_acc)
   * The covariance matrix is created placing the squared standard deviations along the diagonal of
   * an 15x15 matrix.
   */
  std::vector<double> initial_sigma {1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 
                                    1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9,
                                    1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9
                                    };

  /**
   * @brief The initial value of the 15-dimension state vector (x, y, z, roll, pitch, yaw, 
   * x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel, x_acc, y_acc, z_acc)
   */
  std::vector<double> initial_state {0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0};
  /**
   * @brief Loss function
   */
  fuse_core::Loss::SharedPtr loss;
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS__PARAMETERS__UNICYCLE_3D_IGNITION_PARAMS_HPP_
