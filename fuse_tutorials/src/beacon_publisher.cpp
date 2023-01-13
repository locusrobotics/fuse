/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Locus Robotics
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
#include <fuse_tutorials/beacon_publisher.h>

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_variables/point_2d_landmark.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <string>
#include <vector>

// Register this publisher with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_tutorials::BeaconPublisher, fuse_core::Publisher);

namespace fuse_tutorials
{
void BeaconPublisher::onInit()
{
  // Read configuration from the parameter server
  private_node_handle_.param("map_frame_id", map_frame_id_, std::string("map"));

  // Advertise the output topics
<<<<<<< HEAD
  beacon_publisher_ = node_->create_publisher<sensor_msgs::PointCloud2>("beacons", 1);
=======
  beacon_publisher_ = private_node_handle_.advertise<sensor_msgs::msg::PointCloud2>("beacons", 1);
>>>>>>> eb51957... msg changes
}

void BeaconPublisher::notifyCallback(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  // This is where all of the processing happens in this publisher implementation. All of the beacons are represented
  // as fuse_variables::Point2DLandmark objects. We loop through the variables in the graph and keep a pointer to the
  // variables that are the correct type.
  auto beacons = std::vector<const fuse_variables::Point2DLandmark*>();
  for (const auto& variable : graph->getVariables())
  {
    const auto beacon = dynamic_cast<const fuse_variables::Point2DLandmark*>(&variable);
    if (beacon)
    {
      beacons.push_back(beacon);
    }
  }

  // We then transform those variables into a sensor_msgs::msg::PointCloud2 representation. To support visualization in
  // rviz, the PointCloud2 needs to have (x, y, z) fields of type Float32. Additionally we are adding a channel for
  // the beacon ID. Rviz cannot really display that information, but it is potentially useful.
  auto msg = sensor_msgs::msg::PointCloud2();
  msg.header.stamp = this->get_node_clock_interface()->now();  // TODO(CH3): Implement getter in AsyncPublisher
  msg.header.frame_id = map_frame_id_;
  sensor_msgs::msg::PointCloud2Modifier modifier(msg);
  // clang-format off
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "id", 1, sensor_msgs::msg::PointField::UINT32);
  // clang-format on
  modifier.resize(beacons.size());
  sensor_msgs::msg::PointCloud2Iterator<float> x_it(msg, "x");
  sensor_msgs::msg::PointCloud2Iterator<float> y_it(msg, "y");
  sensor_msgs::msg::PointCloud2Iterator<float> z_it(msg, "z");
  sensor_msgs::msg::PointCloud2Iterator<unsigned int> id_it(msg, "id");
  for (auto id = 0u; id < beacons.size(); ++id)
  {
    const auto& beacon = beacons.at(id);
    *x_it = static_cast<float>(beacon->x());
    *y_it = static_cast<float>(beacon->y());
    *z_it = 0.0f;
    *id_it = static_cast<unsigned int>(beacon->id());
    ++x_it;
    ++y_it;
    ++z_it;
    ++id_it;
  }

  // Publish the pointcloud
  beacon_publisher_->publish(msg);
}

}  // namespace fuse_tutorials
