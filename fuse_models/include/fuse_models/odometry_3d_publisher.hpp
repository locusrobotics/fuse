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
#ifndef FUSE_MODELS__ODOMETRY_3D_PUBLISHER_HPP_
#define FUSE_MODELS__ODOMETRY_3D_PUBLISHER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

#include <fuse_models/parameters/odometry_3d_publisher_params.hpp>

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/console.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_core/variable.hpp>
#include <fuse_publishers/stamped_variable_synchronizer.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>


namespace fuse_models
{

/**
 * @class Odometry3DPublisher plugin that publishes a nav_msgs::msg::Odometry message and broadcasts
 * a tf transform for optimized 3D state data (combination of Position3DStamped,
 * Orientation3DStamped, VelocityLinear3DStamped, VelocityAngular3DStamped, and
 * AccelerationLinear3DStamped).
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to
 *                                                                           publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - publish_tf (bool, default: true)  Whether to publish the generated pose data as a transform to
 *                                      the tf tree
 *  - predict_to_current_time (bool, default: false) The tf publication happens at a fixed rate.
 *                                                   This parameter specifies whether we should
 *                                                   predict, using the 3D omnidirectional model, the state
 *                                                   at the time of the tf publication, rather than
 *                                                   the last posterior (optimized) state.
 *  - publish_frequency (double, default: 10.0)  How often, in Hz, we publish the filtered state
 *                                               data and broadcast the transform
 *  - tf_cache_time (double, default: 10.0)  The length of our tf cache (only used if the
 *                                           world_frame_id and the map_frame_id are the same)
 *  - tf_timeout (double, default: 0.1)  Our tf lookup timeout period (only used if the
 *                                       world_frame_id and the map_frame_id are the same)
 *  - queue_size (int, default: 1)  The size of our ROS publication queue
 *  - map_frame_id (string, default: "map")  Our map frame_id
 *  - odom_frame_id (string, default: "odom")  Our odom frame_id
 *  - base_link_frame_id (string, default: "base_link")  Our base_link (body) frame_id
 *  - world_frame_id (string, default: "odom")  The frame_id that will be published as the parent
 *                                              frame for the output. Must be either the
 *                                              map_frame_id or the odom_frame_id.
 *  - topic (string, default: "odometry/filtered")  The ROS topic to which we will publish the
 *                                                  filtered state data
 *
 * Publishes:
 *  - odometry/filtered (nav_msgs::msg::Odometry)  The most recent optimized state, gives as an
 *                                                 odometry message
 *  - tf (via a tf2_ros::TransformBroadcaster)  The most recent optimized state, as a tf transform
 *
 * Subscribes:
 *  - tf, tf_static (tf2_msgs::msg::TFMessage)  Subscribes to tf data to obtain the requisite
 *                                              odom->base_link transform, but only if the
 *                                              world_frame_id is set to the value of the
 *                                              map_frame_id.
 */
class Odometry3DPublisher : public fuse_core::AsyncPublisher
{
public:
  FUSE_SMART_PTR_DEFINITIONS_WITH_EIGEN(Odometry3DPublisher)
  using ParameterType = parameters::Odometry3DPublisherParams;

  /**
   * @brief Constructor
   */
  Odometry3DPublisher();

  /**
   * @brief Destructor
   */
  virtual ~Odometry3DPublisher() = default;

  /**
   * @brief Shadowing extension to the AsyncPublisher::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name) override;

protected:
  /**
   * @brief Perform any required post-construction initialization, such as advertising publishers or
   *        reading from the parameter server.
   */
  void onInit() override;

  /**
   * @brief Fires whenever an optimized graph has been computed
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been
   *                        added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be
   *                        performed whenever needed
   */
  void notifyCallback(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph) override;

  /**
   * @brief Perform any required operations before the first call to notify() occurs
   */
  void onStart() override;

  /**
   * @brief Perform any required operations to stop publications
   */
  void onStop() override;

  /**
   * @brief Retrieves the given variable values at the requested time from the graph
   * @param[in] graph The graph from which we will retrieve the state
   * @param[in] stamp The time stamp at which we want the state
   * @param[in] device_id The device ID for which we want the given variables
   * @param[out] position_uuid The UUID of the position variable that gets extracted from the graph
   * @param[out] orientation_uuid The UUID of the orientation variable that gets extracted from the
   *                              graph
   * @param[out] velocity_linear_uuid The UUID of the linear velocity variable that gets extracted
   *                                  from the graph
   * @param[out] velocity_angular_uuid The UUID of the angular velocity variable that gets extracted
   *                                   from the graph
   * @param[out] acceleration_linear_uuid The UUID of the linear acceleration variable that gets
   *                                      extracted from the graph
   * @param[out] odometry All of the fuse pose and velocity variable values get packed into this
   *                      structure
   * @param[out] acceleration All of the fuse acceleration variable values get packed into this
   *                          structure
   * @return true if the checks pass, false otherwise
   */
  bool getState(
    const fuse_core::Graph & graph,
    const rclcpp::Time & stamp,
    const fuse_core::UUID & device_id,
    fuse_core::UUID & position_uuid,
    fuse_core::UUID & orientation_uuid,
    fuse_core::UUID & velocity_linear_uuid,
    fuse_core::UUID & velocity_angular_uuid,
    fuse_core::UUID & acceleration_linear_uuid,
    nav_msgs::msg::Odometry & odometry,
    geometry_msgs::msg::AccelWithCovarianceStamped & acceleration);

  /**
   * @brief Timer callback method for the filtered state publication and tf broadcasting
   * @param[in] event The timer event parameters that are associated with the given invocation
   */
  void publishTimerCallback();

  /**
   * @brief Object that searches for the most recent common timestamp for a set of variables
   */
  using Synchronizer = fuse_publishers::StampedVariableSynchronizer<
    fuse_variables::Orientation3DStamped,
    fuse_variables::Position3DStamped,
    fuse_variables::VelocityLinear3DStamped,
    fuse_variables::VelocityAngular3DStamped,
    fuse_variables::AccelerationLinear3DStamped>;

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Timers,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncPublisher interfaces_

  fuse_core::UUID device_id_;  //!< The UUID of this device
  rclcpp::Clock::SharedPtr clock_;  //!< The publisher's clock, for timestamping and logging
  rclcpp::Logger logger_;  //!< The publisher's logger

  ParameterType params_;

  rclcpp::Time latest_stamp_;
  rclcpp::Time latest_covariance_stamp_;
  bool latest_covariance_valid_{false};  //!< Whether the latest covariance computed is valid or not
  nav_msgs::msg::Odometry odom_output_;
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration_output_;

  //!< Object that tracks the latest common timestamp of multiple variables
  Synchronizer synchronizer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  fuse_core::DelayedThrottleFilter delayed_throttle_filter_{10.0};  //!< A ros::console filter to
                                                                    //!< print delayed throttle
                                                                    //!< messages, that can be reset
                                                                    //!< on start

  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex mutex_;  //!< A mutex to protect the access to the attributes used concurrently by the
                      //!< notifyCallback and publishTimerCallback methods:
                      //!<   latest_stamp_, latest_covariance_stamp_, odom_output_ and
                      //!<   acceleration_output_
};

}  // namespace fuse_models

#endif  // FUSE_MODELS__ODOMETRY_3D_PUBLISHER_HPP_
