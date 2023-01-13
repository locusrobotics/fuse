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
#ifndef FUSE_TUTORIALS_BEACON_PUBLISHER_H
#define FUSE_TUTORIALS_BEACON_PUBLISHER_H

#include <fuse_core/async_publisher.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/transaction.hpp>
#include <ros/ros.h>

#include <string>

namespace fuse_tutorials
{
/**
 * @brief Implements a fuse Publisher plugin to visualize the current position of each beacon
 *
 * The main purpose for this publisher is to demonstrate how to write your own publisher classes. In the fuse
 * framework, a "Publisher" class is provided with a copy of the graph after each optimization cycle. This allows
 * the publisher to extract optimized variable values and covariances from the graph, and then publish the extracted
 * information to ROS in a convenient format. This could be as simple as publishing the latest robot pose, or as
 * complicated as constructing a map from all optimized poses.
 *
 * For the purposes of this tutorial, let's imagine that you have developed a new robotic sensor that is capable
 * of measuring the distance to some sort of beacon. The position of each beacon is known roughly ahead of time, but
 * the beacon positions will be refined as the robot collects data. We would like to extract the current estimate for
 * each beacon position and publish it for visualization in rviz. The easiest thing to display in rviz that supports
 * a collection of 2D points is a PointCloud2 message, so that is what this Publisher plugin will output.
 *
 * Each fuse Publisher is implemented as a plugin, which is loaded by the optimizer at runtime. This allows new
 * publisher classes to be implemented outside of the main fuse package, such as in this fuse_tutorials package. The
 * fuse Publisher base class defines a few basic methods for communicating between the derived Publisher and the
 * optimizer.
 *  - initialize()
 *    This is called by the optimizer after construction. This is a common pattern used by plugins. This is often when
 *    the Parameter Server is queried for configuration data.
 *  - start()
 *    Tells the publisher to prepare for publishing messages. This will be called once, before the optimizer starts
 *    processing its first graph. Additionally, if the optimizer is ever stopped, the start() method will be called
 *    again before the optimizer resumes graph processing. Initializing any state that tracks changes to the graph
 *    can be done here.
 *  - stop()
 *    Tells the fuse publisher that publishing should stop. Releasing memory and other cleanup can be done here.
 *  - notify()
 *    The optimizer provides the publisher with the latest set of optimized states in the form of a Graph object, as
 *    well as the set of changes since the last notify() call in the form of a Transaction. This is where most of the
 *    Publisher's work will happen, extracting information from the graph and publishing messages to ROS.
 *
 * An issue with the Publisher base class is a rather complex threading model, where notify() can be executed at any
 * time by optimizer's thread, but any other ROS callbacks (timers, subscriptions, etc.) will be executed in the
 * callback queue thread. To make derived publishers easier to implement, the AsyncPublisher base class is provided,
 * which hides most of the thread synchronization details. The AsyncPublisher class provides its own callback queue
 * and spinner, making it act much like a typical, single-threaded ROS node. All of the original base Publisher methods
 * are wrapped. Instead, slightly modified versions are provided, which are executed within the AsyncPublisher's
 * spinner thread.
 *  - onInit() can be overridden instead of initialize()
 *  - onStart() can be overridden instead of start()
 *  - onStop() can be overridden instead of stop()
 *  - notifyCallback() can be overridden instead of notify()
 * Additionally, the AsyncPublisher base class provides a public and private node handle that are pre-configured
 * for use with the spinner's thread and callback queue. These should be used instead of creating your own, in much the
 * same way that nodelets provide methods for accessing properly configured node handles.
 *
 * All of the publishers provided by the fuse_models package use the AsyncPublisher base class, and it is the
 * recommended way to start developing new publishers.
 */
class BeaconPublisher : public fuse_core::AsyncPublisher
{
public:
  // It is convenient to have some typedefs for various smart pointer types (shared, unique, etc.). A macro is provided
  // to make it easy to define these typedefs and ensures that the naming is consistent throughout all fuse packages.
  FUSE_SMART_PTR_DEFINITIONS(BeaconPublisher)

  /**
   * @brief Default constructor
   *
   * A default constructor is required by pluginlib. The real initialization of the publisher will occur in the
   * onInit() method. This will be called immediately after construction by the optimizer node. We do, however, specify
   * the number of threads to use to spin the callback queue. Generally this will be 1, unless you have a good reason
   * to use a multi-threaded spinner.
   */
  BeaconPublisher() : fuse_core::AsyncPublisher(1) {}  // TODO(methylDragon): Refactor this in the same way it was done in fuse_publishers

  /**
   * @brief Perform any required initialization for the publisher
   *
   * This could include things like reading from the parameter server or advertising ROS publishers. The class's node
   * handles will be properly initialized before AsyncPublisher::onInit() is called. Spinning of the callback queue
   * will not begin until after the call to AsyncPublisher::onInit() completes.
   */
  void onInit() override;

  /**
   * @brief Perform any required operations before the first call to notifyCallback() occurs
   *
   * Special handling in the onStart() and onStop() is less common for Publisher classes, but it is provided if needed.
   * For this publisher, we do not need to track state between notify() calls, so there is nothing to do here. If this
   * was not a tutorial, I would have omitted the override entirely.
   */
  void onStart() override {};

  /**
   * @brief Perform any required operations to stop publications
   *
   * Special handling in the onStart() and onStop() is less common for Publisher classes, but it is provided if needed.
   * For this publisher, we do not have any internal state that needs special cleanup, so there is nothing to do here.
   * If this was not a tutorial, I would have omitted the override entirely.
   */
  void onStop() override {};

  /**
   * @brief Called when a newly optimized graph has been generated by the optimizer
   *
   * This is typically where most of the processing happens in publisher implementations. The variables of interest
   * are extracted from the graph and converted into ROS messages. In this publisher, we are interested in all
   * "beacon" variables, represented as fuse_variables::Point2DLandmark objects. We loop through the variables in the
   * graph, determine if it is the correct type, and insert the current (X, Y) coordinates into a
   * sensor_msgs::msg::PointCloud2 object.
   *
   * @param[in] transaction - A read-only pointer to a Transaction object, describing all changes to the graph
   * @param[in] graph - A read-only pointer to the graph object, allowing queries to be performed whenever needed
   */
  void notifyCallback(fuse_core::Transaction::ConstSharedPtr transaction, fuse_core::Graph::ConstSharedPtr graph)
    override;

protected:
  std::string map_frame_id_;  //!< The name of the robot's map frame
  ros::Publisher beacon_publisher_;  //!< Publish the set of beacon positions as a sensor_msgs::msg::PointCloud2 message
};

}  // namespace fuse_tutorials

#endif  // FUSE_TUTORIALS_BEACON_PUBLISHER_H
