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
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;\
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FUSE_MODELS__UNICYCLE_3D_IGNITION_HPP_
#define FUSE_MODELS__UNICYCLE_3D_IGNITION_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_models/parameters/omnidirectional_3d_ignition_params.hpp>
#include <fuse_msgs/srv/set_pose.hpp>
#include <fuse_msgs/srv/set_pose_deprecated.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>


namespace fuse_models
{

/**
 * @brief A fuse_models ignition sensor designed to be used in conjunction with the Omnidirectional 3D
 *        motion model.
 *
 * This class publishes a transaction that contains a prior on each state subvariable used in the
 * Omnidirectional 3D motion model (x, y, z, qx, qy, qz, x_vel, y_vel, z_vel, roll_vel, pitch_vel, 
 * yaw_vel, x_acc, y_acc, z_acc). When the sensor is first loaded, it publishes a single transaction 
 * with the configured initial state and covariance.
 * Additionally, whenever a pose is received, either on the set_pose service or the topic, this
 * ignition sensor resets the optimizer then publishes a new transaction with a prior at the
 * specified pose. Priors on velocities and accelerations continue to use the values
 * configured on the parameter server.
 *
 * Parameters:
 *  - ~device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to
 *                                                                            publish
 *  - ~device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - ~initial_sigma (vector of doubles) A 15-dimensional vector containing the standard deviations
 *                                       for the initial state values. The covariance matrix is
 *                                       created placing the squared standard deviations along the
 *                                       diagonal of an 15x15 matrix. Variable order is (x, y, z, 
 *                                       roll, pitch, yaw, x_vel, y_vel, z_vel, roll_vel, pitch_vel, 
 *                                       yaw_vel, x_acc, y_acc, z_acc).
 *  - ~initial_state (vector of doubles) A 15-dimensional vector containing the initial values for
 *                                       the state. Variable order is (x, y, z, 
 *                                       qx, qy, qz, x_vel, y_vel, z_vel, roll_vel, pitch_vel, 
 *                                       yaw_vel, x_acc, y_acc, z_acc).
 *  - ~queue_size (int, default: 10) The subscriber queue size for the pose messages
 *  - ~reset_service (string, default: "~/reset") The name of the reset service to call before
 *                                                sending a transaction
 *  - ~set_pose_deprecated_service (string, default: "set_pose_deprecated") The name of the
 *                                                                          set_pose_deprecated
 *                                                                          service
 *  - ~set_pose_service (string, default: "set_pose") The name of the set_pose service to advertise
 *  - ~topic (string, default: "set_pose") The topic name for received PoseWithCovarianceStamped
 *                                         messages
 */
class Omnidirectional3DIgnition : public fuse_core::AsyncSensorModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Omnidirectional3DIgnition)
  using ParameterType = parameters::Omnidirectional3DIgnitionParams;

  /**
   * @brief Default constructor
   *
   * All plugins are required to have a constructor that accepts no arguments
   */
  Omnidirectional3DIgnition();

  /**
   * @brief Destructor
   */
  ~Omnidirectional3DIgnition() = default;

  /**
   * @brief Shadowing extension to the AsyncSensorModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the optimizer
   *
   * As a very special case, we are overriding the start() method instead of providing an onStart()
   * implementation. This is because the Omnidirectional3DIgnition sensor calls reset() on the optimizer,
   * which in turn calls stop() and start(). If we used the AsyncSensorModel implementations of
   * start() and stop(), the system would hang inside of one callback function while waiting for
   * another callback to complete.
   */
  void start() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the optimizer
   *
   * As a very special case, we are overriding the stop() method instead of providing an onStop()
   * implementation. This is because the Omnidirectional3DIgnition sensor calls reset() on the optimizer,
   * which in turn calls stop() and start(). If we used the AsyncSensorModel implementations of
   * start() and stop(), the system would hang inside of one callback function while waiting for
   * another callback to complete.
   */
  void stop() override;

  /**
   * @brief Triggers the publication of a new prior transaction at the supplied pose
   */
  void subscriberCallback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  /**
   * @brief Triggers the publication of a new prior transaction at the supplied pose
   */
  bool setPoseServiceCallback(
    rclcpp::Service<fuse_msgs::srv::SetPose>::SharedPtr service,
    std::shared_ptr<rmw_request_id_t>,
    const fuse_msgs::srv::SetPose::Request::SharedPtr req);

  /**
   * @brief Triggers the publication of a new prior transaction at the supplied pose
   */
  bool setPoseDeprecatedServiceCallback(
    rclcpp::Service<fuse_msgs::srv::SetPoseDeprecated>::SharedPtr service,
    std::shared_ptr<rmw_request_id_t> request_id,
    const fuse_msgs::srv::SetPoseDeprecated::Request::SharedPtr req);

protected:
  /**
   * @brief Perform any required initialization for the kinematic ignition sensor
   */
  void onInit() override;

  /**
   * @brief Process a received pose from one of the ROS comm channels
   *
   * This method validates the input pose, resets the optimizer, then constructs and sends the
   * initial state constraints (by calling sendPrior()).
   *
   * @param[in] pose - The pose and covariance to use for the prior constraints on (x, y, z, roll, pitch, yaw)
   */
  void process(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    std::function<void()> post_process = nullptr);

  /**
   * @brief Create and send a prior transaction based on the supplied pose
   *
   * The omnidirectional 3d state members not included in the pose message (x_vel, y_vel, z_vel, roll_vel, pitch_vel, 
   * yaw_vel, x_acc, y_acc, z_acc) will use the initial state values and standard deviations configured on the 
   * parameter server.
   *
   * @param[in] pose - The pose and covariance to use for the prior constraints on (x, y, z, roll, pitch, yaw)
   */
  void sendPrior(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Graph,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Services,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncSensorModel interfaces_

  std::atomic_bool started_;  //!< Flag indicating the sensor has been started
  bool initial_transaction_sent_;  //!< Flag indicating an initial transaction has been sent already
  fuse_core::UUID device_id_;  //!< The UUID of this device
  rclcpp::Clock::SharedPtr clock_;  //!< The sensor model's clock, for timestamping
  rclcpp::Logger logger_;  //!< The sensor model's logger

  ParameterType params_;  //!< Object containing all of the configuration parameters

  //!< Service client used to call the "reset" service on the optimizer
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::Service<fuse_msgs::srv::SetPose>::SharedPtr set_pose_service_;
  rclcpp::Service<fuse_msgs::srv::SetPoseDeprecated>::SharedPtr set_pose_deprecated_service_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
};

}  // namespace fuse_models

#endif  // FUSE_MODELS__UNICYCLE_3D_IGNITION_HPP_
