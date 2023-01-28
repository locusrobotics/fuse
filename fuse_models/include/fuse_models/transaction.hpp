/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
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

#ifndef FUSE_MODELS__TRANSACTION_HPP_
#define FUSE_MODELS__TRANSACTION_HPP_

#include <string>

#include <fuse_models/parameters/transaction_params.hpp>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/transaction_deserializer.hpp>

#include <fuse_msgs/msg/serialized_transaction.hpp>
#include <rclcpp/rclcpp.hpp>


namespace fuse_models
{

/**
 * @brief An adapter-type sensor that produces transactions with the same added and removed
 *        constraints from an input transaction. This is useful for debugging purposes because it
 *        allows to play back the recorded transactions.
 *
 * This sensor subscribes to a fuse_msgs::msg::SerializedTransaction topic and deserializes each
 * received message into a transaction.
 *
 * Parameters:
 *  - ~queue_size (int, default: 10) The subscriber queue size for the transaction messages
 *  - ~topic (string) The topic to which to subscribe for the transaction messages
 *
 * Subscribes:
 *  - topic (fuse_msgs::msg::SerializedTransaction) Transaction
 */
class Transaction : public fuse_core::AsyncSensorModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Transaction)
  using ParameterType = parameters::TransactionParams;

  /**
   * @brief Default constructor
   */
  Transaction();

  /**
   * @brief Destructor
   */
  virtual ~Transaction() = default;

  /**
   * @brief Shadowing extension to the AsyncSensorModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

protected:
  /**
   * @brief Loads ROS parameters and subscribes to the parameterized topic
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

  /**
   * @brief Callback for transaction messages
   * @param[in] msg - The transaction message to process
   */
  void process(const fuse_msgs::msg::SerializedTransaction & msg);

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncSensorModel interfaces_

  ParameterType params_;  //!< Object containing all of the configuration parameters

  rclcpp::Subscription<fuse_msgs::msg::SerializedTransaction>::SharedPtr sub_;

  //!< Deserializer for SerializedTransaction messages
  fuse_core::TransactionDeserializer transaction_deserializer_;
};

}  // namespace fuse_models

#endif  // FUSE_MODELS__TRANSACTION_HPP_
