/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Clearpath Robotics
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
#ifndef FUSE_PUBLISHERS_GRAPH_PUBLISHER_H
#define FUSE_PUBLISHERS_GRAPH_PUBLISHER_H

#include <fuse_core/async_publisher.h>
#include <fuse_core/graph.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>

#include <fuse_msgs/Graph.h>

#include <ros/ros.h>

#include <string>
#include <unordered_map>


namespace fuse_publishers
{

/**
 * @brief Publisher plugin that publishes the latest graph
 *
 * There are two options:
 *  - The graph is only published when variables or constraints are added or removed.
 *  - The graph is also published on every optimization update, even if the graph topology didn't change.
 *    Remember the variables' solution might have changed on every graph optimization cycle.
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - publish_optimized_graph (bool, default: true) Flag indicating that the graph should be always published, i.e. not
 *                                                  only when variables or constraints are added or removed, but also
 *                                                  on every optimization cycle.
 *  - queue_size (int, default: 100) Queue size for the graph publisher.
 *  - frame_id (string, default: "map") Graph frame ID.
 *
 * Publishes:
 *  - graph (fuse_msgs::Graph)  The most recent changed and/or optimized grap
 */
class GraphPublisher : public fuse_core::AsyncPublisher
{
public:
  SMART_PTR_DEFINITIONS(GraphPublisher);

  /**
   * @brief Constructor
   */
  GraphPublisher();

  /**
   * @brief Destructor
   */
  virtual ~GraphPublisher() = default;

  /**
   * @brief Perform any required post-construction initialization, such as advertising publishers or reading from the
   * parameter server.
   */
  void onInit() override;

  /**
   * @brief Perform any required operations before the first call to notify() occurs
   */
  void onStart() override;

  /**
   * @brief Notify the publisher about variables that have been added or removed
   *
   * This publisher publishes the most recent graph. By analyzing the added and removed variables, the most
   * recent graph can be maintained without re-building the whole message from scratch on every Publisher::publish()
   * call.
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be performed whenever needed
   */
  void notifyCallback(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph) override;

protected:
  // UUID to index map, used to efficiently find the index of the variable or constraint in the graph message arrays
  using UUIDIndices = std::unordered_map<fuse_core::UUID, std::size_t, fuse_core::uuid::hash>;

  /**
   * @brief Update the last graph message with a transaction, as in Graph::update.
   *
   * @param[in] transaction A set of variable and constraints additions and deletions
   */
  void updateGraphMessage(fuse_core::Transaction::ConstSharedPtr transaction);

  /**
   * @brief Check if the variable already exists in the graph
   *
   * @param[in] variable_uuid The UUID of the variable being searched for
   * @return                  True if the variable exists, false otherwise
   */
  bool variableExists(const fuse_core::UUID& variable_uuid) const;

  /**
   * @brief Check if a constraint already exists in the graph
   *
   * @param[in] constraint_uuid The UUID of the constraint being searched for
   * @return                    True if the constraint already exists, false otherwise
   */
  bool constraintExists(const fuse_core::UUID& constraint_uuid) const;

  /**
   * @brief Add a new variable to the graph message.
   *
   * A fuse_msgs::Variable message will be created and added to the graph message. If this variable already exists in
   * the graph, the function will return false.
   *
   * @param[in] variable The new variable to be added
   * @return             True if the variable was added, false otherwise
   */
  bool addVariable(const fuse_core::Variable& variable);

  /**
   * @brief Add a new constraint to the graph message.
   *
   * A fuse_msgs::Constraint message will be created and added to the graph message. The referenced variables are
   * assumed to already exist in the graph.
   *
   * @param[in] constraint The new constraint to be added
   * @return               True if the constraint was added, False otherwise
   */
  bool addConstraint(const fuse_core::Constraint& constraint);

  /**
   * @brief Remove a variable from the graph message.
   *
   * @param[in] variable_uuid The UUID of the variable to be removed
   * @return                  True if the variable was removed, false otherwise
   */
  bool removeVariable(const fuse_core::UUID& variable_uuid);

  /**
   * @brief Remove a constraint from the graph message.
   *
   * @param[in] constraint_uuid The UUID of the constraint to be removed
   * @return                    True if the constraint was removed, false otherwise
   */
  bool removeConstraint(const fuse_core::UUID& constraint_uuid);

  fuse_core::UUID device_id_{ fuse_core::uuid::NIL };  //!< The UUID of the device to be published
  ros::Publisher graph_publisher_;  //!< Publish the graph as a fuse_msgs::Graph
  bool publish_optimized_graph_{ true };  //!< Internal flag indicating that the graph should be always published
  int queue_size_ { 100 };  //!< Queue size for the graph publisher
  std::string frame_id_ { "map" };  //!< Graph frame ID

  fuse_msgs::Graph graph_msg_;  //!< Last graph message. This is cached so we can efficiently update it using new
                                //!< transactions and graphs updates in notifyCallback
  UUIDIndices index_by_variable_uuid_;  //!< Variable UUID to index map
  UUIDIndices index_by_constraint_uuid_;  //!< Constraint UUID to index map
};

}  // namespace fuse_publishers

#endif  // FUSE_PUBLISHERS_GRAPH_PUBLISHER_H
