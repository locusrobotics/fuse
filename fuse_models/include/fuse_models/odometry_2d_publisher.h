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
#ifndef FUSE_MODELS_ODOMETRY_2D_PUBLISHER_H
#define FUSE_MODELS_ODOMETRY_2D_PUBLISHER_H

#include <fuse_models/parameters/odometry_2d_publisher_params.h>

#include <fuse_core/async_publisher.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_publishers/stamped_variable_synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace fuse_models
{

/**
 * @class Odometry2DPublisher plugin that publishes a nav_msgs::Odometry message and broadcasts a tf transform for optimized 2D
 * state data (combination of Position2DStamped, Orientation2DStamped, VelocityLinear2DStamped, and
 * VelocityAngular2DStamped).
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - publish_tf (bool, default: true)  Whether to publish the generated pose data as a transform to the tf tree
 *  - predict_to_current_time (bool, default: false) The tf publication happens at a fixed rate. This parameter
 *                                                   specifies whether we should predict, using the 2D unicycle model,
 *                                                   the state at the time of the tf publication, rather than the last
 *                                                   posterior (optimized) state.
 *  - tf_publish_frequency (double, default: 10.0)  How often, in Hz, we publish the transform
 *  - tf_cache_time (double, default: 10.0)  The length of our tf cache (only used if the world_frame_id and the
 *                                           map_frame_id are the same)
 *  - tf_timeout (double, default: 0.1)  Our tf lookup timeout period (only used if the world_frame_id and the
 *                                       map_frame_id are the same)
 *  - queue_size (int, default: 1)  The size of our ROS publication queue
 *  - map_frame_id (string, default: "map")  Our map frame_id
 *  - odom_frame_id (string, default: "odom")  Our odom frame_id
 *  - base_link_frame_id (string, default: "base_link")  Our base_link (body) frame_id
 *  - world_frame_id (string, default: "odom")  The frame_id that will be published as the parent frame for the output.
 *                                              Must be either the map_frame_id or the odom_frame_id.
 *  - topic (string, default: "~odometry/filtered")  The ROS topic to which we will publish the filtered state data
 *
 * Publishes:
 *  - odometry/filtered (nav_msgs::Odometry)  The most recent optimized state, gives as an odometry message
 *  - tf (via a tf2_ros::TransformBroadcaster)  The most recent optimized state, as a tf transform
 *
 * Subscribes:
 *  - tf, tf_static (tf2_msgs::TFMessage)  Subscribes to tf data to obtain the requisite odom->base_link transform,
 *                                         but only if the world_frame_id is set to the value of the map_frame_id.
 */
class Odometry2DPublisher : public fuse_core::AsyncPublisher
{
public:
  SMART_PTR_DEFINITIONS(Odometry2DPublisher);
  using ParameterType = parameters::Odometry2DPublisherParams;

  /**
   * @brief Constructor
   */
  Odometry2DPublisher();

  /**
   * @brief Destructor
   */
  virtual ~Odometry2DPublisher() = default;

protected:
  /**
   * @brief Perform any required post-construction initialization, such as advertising publishers or reading from the
   * parameter server.
   */
  void onInit() override;

  /**
   * @brief Fires whenever an optimized graph has been computed
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be performed whenever needed
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
   * @param[out] orientation_uuid The UUID of the orientation variable that gets extracted from the graph
   * @param[out] velocity_linear_uuid The UUID of the linear velocity variable that gets extracted from the graph
   * @param[out] velocity_angular_uuid The UUID of the angular velocity variable that gets extracted from the graph
   * @param[out] state All of the fuse variable values get packed into this structure
   * @return true if the checks pass, false otherwise
   */
  bool getState(
    const fuse_core::Graph& graph,
    const ros::Time& stamp,
    const fuse_core::UUID& device_id,
    fuse_core::UUID& position_uuid,
    fuse_core::UUID& orientation_uuid,
    fuse_core::UUID& velocity_linear_uuid,
    fuse_core::UUID& velocity_angular_uuid,
    nav_msgs::Odometry& state);

  /**
   * @brief Timer callback method for the tf publication
   * @param[in] event The timer event parameters that are associated with the given invocation
   */
  void tfPublishTimerCallback(const ros::TimerEvent& event);

  /**
   * @brief Object that searches for the most recent common timestamp for a set of variables
   */
  using Synchronizer = fuse_publishers::StampedVariableSynchronizer<fuse_variables::Orientation2DStamped,
                                                                    fuse_variables::Position2DStamped,
                                                                    fuse_variables::VelocityLinear2DStamped,
                                                                    fuse_variables::VelocityAngular2DStamped>;

  fuse_core::UUID device_id_;  //!< The UUID of this device

  ParameterType params_;

  ros::Time latest_stamp_;

  nav_msgs::Odometry odom_output_;

  Synchronizer synchronizer_;  //!< Object that tracks the latest common timestamp of multiple variables

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  ros::Publisher odom_pub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Timer tf_publish_timer_;
};

}  // namespace fuse_models

#endif  // FUSE_MODELS_ODOMETRY_2D_PUBLISHER_H
