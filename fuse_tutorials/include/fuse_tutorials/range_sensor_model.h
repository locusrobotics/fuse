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
#ifndef FUSE_TUTORIALS_RANGE_SENSOR_MODEL_H
#define FUSE_TUTORIALS_RANGE_SENSOR_MODEL_H

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/uuid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <unordered_map>

namespace fuse_tutorials
{
/**
 * @brief Implements a range-only sensor model that generates constraints between the robot and a beacon.
 *
 * The main purpose for this sensor model is to demonstrate how to write your own sensor model classes.
 *
 * For the purposes of this tutorial, let's imagine that you have developed a new robotic sensor that is capable
 * of measuring the distance to some sort of beacon, but does not provide any information about the bearing/heading
 * to that beacon. None of the fuse packages provide such a sensor model, so you need to develop one yourself.
 * Because I don't want to create a brand new message type for this tutorial, the driver for this new sensor will be
 * publishing sensor_msgs::msg::PointCloud2 messages with the following fields defined:
 *  - "id", uint32, count 1, offset 0, The unique ID associated with that beacon
 *  - "range", float64, count 1, offset 4, The range, in meters, between the robot and the beacon
 *  - "sigma", float64, count 1, offset 12, The standard deviation of the range measurement, in meters
 *
 * The "sensor model" class provides an interface to ROS, allowing sensor messages to be received. The sensor model
 * class also acts as a "factory" (in a programming sense) that creates new sensor constraints for each received
 * sensor measurement, and forward those constraints the fuse optimizer. The optimizer is where the constraints from
 * all configured sensors are combined, and the best possible value for each state variable is determined.
 *
 * Each fuse SensorModel is implemented as a plugin, which is loaded by the optimizer at runtime. This allows new
 * sensor models to be implemented outside of the main fuse package, such as in this fuse_tutorials package. The
 * fuse SensorModel base class defines a few basic methods for communicating between the derived SensorModel and the
 * optimizer.
 *  - initialize()
 *    This is called by the optimizer after construction. This is a common pattern used by plugins. This is often when
 *    the Parameter Server is queried for configuration data.
 *  - start()
 *    Tells the sensor model to start producing constraints. This is commonly where fuse sensor models first subscribe
 *    to their sensor data topics.
 *  - stop()
 *    Tells the sensor model to stop producing constraints. fuse sensor models typically unsubscribe from topics here.
 *  - graphCallback()
 *    The optimizer provides the sensor model with the latest set of optimized states. For simple sensor models, this
 *    is likely not needed. But something maintaining a database of visual landmarks or similar may need access to the
 *    current set of landmarks.
 *  - TransactionCallback
 *    This is a little different than the other interfaces. This is a callback provided *to* the SensorModel plugin.
 *    That callback is executed by the plugin whenever new constraints are ready to be sent to the optimizer.
 *
 * An issue with the SensorModel base class is a rather complex threading model, where TransactionCallbacks can be
 * executed at any time by any of the sensor model plugins. To make derived sensor models easier to implement, the
 * AsyncSensorModel base class is provided, which hides most of the thread synchronization details. The
 * AsyncSensorModel class provides its own callback queue and spinner, making it act much like a typical,
 * single-threaded ROS node. All of the original base SensorModel methods are wrapped. Instead, slightly modified
 * versions are provided, which are executed within the AsyncSensorModel's spinner thread.
 *  - onInit() can be overridden instead of initialize()
 *  - onStart() can be overridden instead of start()
 *  - onStop() can be overridden instead of stop()
 *  - onGraphUpdate() can be overridden instead of graphCallback()
 *  - sendTransaction() can be called instead of TransactionCallback()
 * Additionally, the AsyncSensorModel base class provides a public and private node handle that are pre-configured
 * for use with the spinner's thread and callback queue. These should be used instead of creating your own, in much the
 * same way that nodelets provide methods for accessing properly configured node handles.
 *
 * All of the sensor models provided by the fuse_models package use the AsyncSensorModel base class, and it is the
 * recommended way to start developing new sensor models.
 */
class RangeSensorModel : public fuse_core::AsyncSensorModel
{
public:
  // It is convenient to have some typedefs for various smart pointer types (shared, unique, etc.). A macro is provided
  // to make it easy to define these typedefs and ensures that the naming is consistent throughout all fuse packages.
  FUSE_SMART_PTR_DEFINITIONS(RangeSensorModel)

  /**
   * @brief Default constructor
   *
   * A default constructor is required by pluginlib. The real initialization of the sensor model will occur in the
   * onInit() method. This will be called immediately after construction by the optimizer node. We do, however, specify
   * the number of threads to use to spin the callback queue. Generally this will be 1, unless you have a good reason
   * to use a multi-threaded spinner.
   */
  RangeSensorModel() :
    fuse_core::AsyncSensorModel(1), logger_(rclcpp::get_logger("uninitialized")) {}

  /**
   * @brief Shadowing extension to the AsyncSensorModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  /**
   * @brief Receives the set of known beacon positions
   *
   * We need a method to initialize the position of each beacon variable. In this tutorial we are assuming a database
   * exists of previously measured but noisy beacon positions. That database will be published on a latched topic. This
   * sensor model will subscribe to that topic, and use that prior information to initialize and properly constrain the
   * beacon positions within the optimizer. No range measurements will be processed until this database has been
   * received.
   *
   * @param[in] msg - Message containing the database of known but noisy beacon positions.
   */
  void priorBeaconsCallback(const sensor_msgs::msg::PointCloud2& msg);

  /**
   * @brief Callback for range measurement messages
   *
   * We will process all of the detected beacons in the input message, generate one or more RangeConstraint
   * objects, and send all of the constraints to the optimizer at once packaged in a Transaction object.
   *
   * @param[in] msg - The range message to process
   */
  void rangesCallback(const sensor_msgs::msg::PointCloud2& msg);

protected:
  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or subscribing to topics. The class's node
   * handles will be properly initialized before AsyncSensorModel::onInit() is called. Spinning of the callback queue
   * will not begin until after the call to AsyncSensorModel::onInit() completes.
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the optimizer
   */
  void onStop() override;

  struct Beacon
  {
    double x;
    double y;
    double sigma;
  };

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncSensorModel interfaces_

  rclcpp::Logger logger_;  //!< The sensor model's logger

  std::unordered_map<unsigned int, Beacon> beacon_db_;  //!< The estimated position of each beacon
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr beacon_sub_;  //!< ROS subscription for the database of prior beacon positions
  bool initialized_ { false };  //!< Flag indicating the initial beacon positions have been processed
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;  //!< ROS subscription for the range sensor measurements
};

}  // namespace fuse_tutorials

#endif  // FUSE_TUTORIALS_RANGE_SENSOR_MODEL_H
