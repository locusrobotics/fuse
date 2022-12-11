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
#ifndef FUSE_PUBLISHERS_PATH_2D_PUBLISHER_H
#define FUSE_PUBLISHERS_PATH_2D_PUBLISHER_H

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/transaction.hpp>
#include <ros/ros.h>

#include <string>


namespace fuse_publishers
{

/**
 * @brief Publisher plugin that publishes all of the stamped 2D poses as a nav_msgs::msg::Path message.
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - frame_id (string, default: map)  Name for the robot's map frame
 */
class Path2DPublisher : public fuse_core::AsyncPublisher
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Path2DPublisher)

  /**
   * @brief Constructor
   */
  Path2DPublisher();

  /**
   * @brief Destructor
   */
  virtual ~Path2DPublisher() = default;

  /**
   * @brief Perform any required post-construction initialization, such as advertising publishers or reading from the
   * parameter server.
   */
  void onInit() override;

  /**
   * @brief Notify the publisher about variables that have been added or removed
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be performed whenever needed
   */
  void notifyCallback(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph) override;

protected:
  fuse_core::UUID device_id_;  //!< The UUID of the device to be published
  std::string frame_id_;  //!< The name of the frame for this path
  ros::Publisher path_publisher_;  //!< The publisher that sends the entire robot trajectory as a path
  ros::Publisher pose_array_publisher_;  //!< The publisher that sends the entire robot trajectory as a pose array
};

}  // namespace fuse_publishers

#endif  // FUSE_PUBLISHERS_PATH_2D_PUBLISHER_H
