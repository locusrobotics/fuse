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
#ifndef FUSE_CORE__TRANSACTION_DESERIALIZER_HPP_
#define FUSE_CORE__TRANSACTION_DESERIALIZER_HPP_

#include <fuse_core/constraint.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/variable.hpp>
#include <fuse_msgs/msg/serialized_transaction.hpp>
#include <pluginlib/class_loader.hpp>

namespace fuse_core
{

/**
 * @brief Serialize a transaction into a message
 */
void serializeTransaction(
  const fuse_core::Transaction & transaction,
  fuse_msgs::msg::SerializedTransaction & msg);

/**
 * @brief Deserialize a Transaction
 *
 * The deserializer object loads all of the known Variable and Constraint libraries, allowing
 * derived types contained within the transaction to be properly deserialized. The libraries will
 * be unloaded on destruction. As a consequence, the deserializer object must outlive any created
 * transaction instances.
 */
class TransactionDeserializer
{
public:
  /**
   * @brief Constructor
   */
  TransactionDeserializer();

  /**
   * @brief Deserialize a SerializedTransaction message into a fuse Transaction object.
   *
   * If no plugin is available for a contained Variable or Constraint, or an error occurs
   * during deserialization, an exception is thrown.
   *
   * @param[IN]  msg  The SerializedTransaction message to be deserialized
   * @return          A fuse Transaction object
   */
  inline fuse_core::Transaction::UniquePtr deserialize(
    const fuse_msgs::msg::SerializedTransaction::ConstSharedPtr msg) const
  {
    return deserialize(*msg);
  }

  /**
   * @brief Deserialize a SerializedTransaction message into a fuse Transaction object.
   *
   * If no plugin is available for a contained Variable or Constraint, or an error occurs
   * during deserialization, an exception is thrown.
   *
   * @param[IN]  msg  The SerializedTransaction message to be deserialized
   * @return          A fuse Transaction object
   */
  fuse_core::Transaction::UniquePtr deserialize(
    const fuse_msgs::msg::SerializedTransaction & msg) const;

private:
  //! Pluginlib class loader for Variable types
  pluginlib::ClassLoader<fuse_core::Variable> variable_loader_;

  //! Pluginlib class loader for Constraint types
  pluginlib::ClassLoader<fuse_core::Constraint> constraint_loader_;

  //! Pluginlib class loader for Loss types
  pluginlib::ClassLoader<fuse_core::Loss> loss_loader_;
};

}  // namespace fuse_core

#endif  // FUSE_CORE__TRANSACTION_DESERIALIZER_HPP_
