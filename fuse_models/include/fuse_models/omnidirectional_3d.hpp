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
#ifndef FUSE_MODELS__UNICYCLE_3D_HPP_
#define FUSE_MODELS__UNICYCLE_3D_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <fuse_core/async_motion_model.hpp>
#include <fuse_core/constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/timestamp_manager.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/variable.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fuse_models
{

/**
 * @brief A fuse_models 3D kinematic model that generates kinematic constraints between provided
 *        time stamps, and adds those constraints to the fuse graph.
 *
 * This class uses a unicycle kinematic model for the robot. It is equivalent to the motion model
 * in the robot_localization state estimation nodes.
 *
 * Parameters:
 *  - ~device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to
 *                                                                            publish
 *  - ~device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - ~buffer_length (double) The length of the graph state buffer and state history, in seconds
 *  - ~process_noise_diagonal (vector of doubles) An 15-dimensional vector containing the diagonal
 *                                                 values for the process noise covariance matrix.
 *                                                 Variable order is (x, y, z, roll, pitch, yaw, 
 *                                                 x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel, 
 *                                                 x_acc, y_acc, z_acc).
 */
class Omnidirectional3D : public fuse_core::AsyncMotionModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS_WITH_EIGEN(Omnidirectional3D)

  /**
   * @brief Default constructor
   *
   * All plugins are required to have a constructor that accepts no arguments
   */
  Omnidirectional3D();

  /**
   * @brief Destructor
   */
  ~Omnidirectional3D() = default;

  /**
   * @brief Shadowing extension to the AsyncMotionModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name) override;

  void print(std::ostream & stream = std::cout) const;

protected:
  /**
   * @brief Structure used to maintain a history of "good" pose estimates
   */
  struct StateHistoryElement
  {
    fuse_core::UUID position_uuid;        //!< The uuid of the associated position variable
    fuse_core::UUID orientation_uuid;     //!< The uuid of the associated orientation variable
    fuse_core::UUID vel_linear_uuid;      //!< The uuid of the associated linear velocity variable
    fuse_core::UUID vel_angular_uuid;     //!< The uuid of the associated angular velocity variable
    fuse_core::UUID acc_linear_uuid;      //!< The uuid of the associated linear acceleration
                                          //!< variable
    fuse_core::Vector3d position    = fuse_core::Vector3d::Zero(); //!< Map-frame position
    Eigen::Quaterniond orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); //!< Map-frame orientation (quaternion)
    fuse_core::Vector3d vel_linear  = fuse_core::Vector3d::Zero(); //!< Body-frame linear velocities
    fuse_core::Vector3d vel_angular = fuse_core::Vector3d::Zero(); //!< Body-frame angular velocities
    fuse_core::Vector3d acc_linear  = fuse_core::Vector3d::Zero(); //!< Body-frame linear (angular not needed) accelerations

    void print(std::ostream & stream = std::cout) const;

    /**
     * @brief Validate the state components: pose, linear velocities, angular velocities and linear
     *        accelerations.
     *
     * This validates the state components are finite. It throws an exception if any validation
     * check fails.
     */
    void validate() const;
  };
  using StateHistory = std::map<rclcpp::Time, StateHistoryElement>;

  /**
   * @brief Augment a transaction structure such that the provided timestamps are connected by
   *        motion model constraints.
   * @param[in]  stamps      The set of timestamps that should be connected by motion model
   *                         constraints
   * @param[out] transaction The transaction object that should be augmented with motion model
   *                         constraints
   * @return                 True if the motion models were generated successfully, false otherwise
   */
  bool applyCallback(fuse_core::Transaction & transaction) override;

  /**
   * @brief Generate a single motion model segment between the specified timestamps.
   *
   * This function is used by the timestamp manager to generate just the new motion model segments
   * required to fulfill a query.
   *
   * @param[in]  beginning_stamp The beginning timestamp of the motion model constraints to be
   *                             generated. \p beginning_stamp is guaranteed to be less than \p
   *                             ending_stamp.
   * @param[in]  ending_stamp    The ending timestamp of the motion model constraints to be
   *                             generated. \p ending_stamp is guaranteed to be greater than \p
   *                             beginning_stamp.
   * @param[out] constraints     One or more motion model constraints between the requested
   *                             timestamps.
   * @param[out] variables       One or more variables at both the \p beginning_stamp and \p
   *                             ending_stamp. The variables should include initial values for the
   *                             optimizer.
   */
  void generateMotionModel(
    const rclcpp::Time & beginning_stamp,
    const rclcpp::Time & ending_stamp,
    std::vector<fuse_core::Constraint::SharedPtr> & constraints,
    std::vector<fuse_core::Variable::SharedPtr> & variables);

  /**
   * @brief Callback fired in the local callback queue thread(s) whenever a new Graph is received
   *        from the optimizer
   * @param[in] graph A read-only pointer to the graph object, allowing queries to be performed
   *                  whenever needed.
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) override;

  /**
   * @brief Perform any required initialization for the kinematic model
   */
  void onInit() override;

  /**
   * @brief Reset the internal state history before starting
   */
  void onStart() override;

  /**
   * @brief Update all of the estimated states in the state history container using the optimized
   *        values from the graph
   * @param[in] graph         The graph object containing updated variable values
   * @param[in] state_history The state history object to be updated
   * @param[in] buffer_length States older than this in the history will be pruned
   */
  static void updateStateHistoryEstimates(
    const fuse_core::Graph & graph,
    StateHistory & state_history,
    const rclcpp::Duration & buffer_length);

  /**
   * @brief Validate the motion model state #1, state #2 and process noise covariance
   *
   * This validates the motion model states and process noise covariance are valid. It throws an
   * exception if any validation check fails.
   *
   * @param[in] state1                   The first/oldest state
   * @param[in] state2                   The second/newest state
   * @param[in] process_noise_covariance The process noise covariance, after it is scaled and
   *                                     multiplied by dt
   */
  static void validateMotionModel(
    const StateHistoryElement & state1, const StateHistoryElement & state2,
    const fuse_core::Matrix15d & process_noise_covariance);

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncSensorModel interfaces_

  rclcpp::Clock::SharedPtr clock_;  //!< The sensor model's clock, for timestamping and logging
  rclcpp::Logger logger_;  //!< The sensor model's logger

  rclcpp::Duration buffer_length_;                 //!< The length of the state history
  fuse_core::UUID device_id_;                      //!< The UUID of the device to be published
  fuse_core::TimestampManager timestamp_manager_;  //!< Tracks timestamps and previously created
                                                   //!< motion model segments
  fuse_core::Matrix15d process_noise_covariance_;  //!< Process noise covariance matrix
  bool scale_process_noise_{false};                //!< Whether to scale the process noise
                                                   //!< covariance pose by the norm of the current
                                                   //!< state twist
  double velocity_linear_norm_min_{1e-3};          //!< The minimum linear velocity norm allowed when
                                                   //!< scaling the process noise covariance
  double velocity_angular_norm_min_{1e-3};         //!< The minimum twist norm allowed when
                                                   //!< scaling the process noise covariance
  bool disable_checks_{false};    //!< Whether to disable the validation checks for the current and
                                  //!< predicted state, including the process noise covariance after
                                  //!< it is scaled and multiplied by dt
  StateHistory state_history_;    //!< History of optimized graph pose estimates
};

std::ostream & operator<<(std::ostream & stream, const Omnidirectional3D & unicycle_2d);

}  // namespace fuse_models

#endif  // FUSE_MODELS__UNICYCLE_3D_HPP_
