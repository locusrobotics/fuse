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
#ifndef FUSE_OPTIMIZERS_OPTIMIZER_H
#define FUSE_OPTIMIZERS_OPTIMIZER_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <fuse_core/graph.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/motion_model.h>
#include <fuse_core/publisher.h>
#include <fuse_core/sensor_model.h>
#include <fuse_core/transaction.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief A base class that can be used to build fuse optimizer nodes
 *
 * An optimizer implements the basic fuse information flow contract:
 *  - Sensors push information into the optimizer using the transaction callback
 *  - The optimizer requests motion models be created between each sensor timestamp
 *  - The optimizer computes the optimal variable values
 *  - The optimizer provides access to the optimal variable values to the publishers
 *
 * Optimizer implementations are not required to use this base class; it is simply provided as a convenience
 * class that implements the mechanics of the information flow contract. Derived classes can then concentrate
 * on the details of when and what to optimize.
 *
 * This base class provides functions for:
 *  - Loading the set of motion model plugins as configured on the parameter server
 *  - Loading the set of publisher plugins as configured on the parameter server
 *  - Loading the set of sensor plugins as configured on the parameter server
 *  - Generating the correct motion model constraints for each received sensor transaction
 *  - Sending updated variable information to the sensors, motion models, and publishers
 *
 * Parameter Server format:
 * @code{.yaml}
 * motion_models:
 *  - name: string
 *    type: string
 *  - ...
 * sensor_models:
 *  - name: string
 *    type: string
 *    motion_models: [name1, name2, ...]
 *  - ...
 * publishers:
 *  - name: string
 *    type: string
 *  - ...
 * @endcode
 */
class Optimizer : public rclcpp::Node
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(Optimizer)

  /**
   * @brief Constructor
   *
   * @param[in] graph               The derived graph object. This allows different graph implementations to be used
   *                                with the same optimizer code.
   * @param[in] node_handle         A node handle in the global namespace
   * @param[in] private_node_handle A node handle in the node's private namespace
   */
  Optimizer(
    rclcpp::NodeOptions options,
    fuse_core::Graph::UniquePtr graph,
    );

  /**
   * @brief Destructor
   */
  virtual ~Optimizer();

protected:
  // The unique ptrs returned by pluginlib have a custom deleter. This makes specifying the type rather annoying
  // as it is not equivalent to Class::UniquePtr
  using MotionModelUniquePtr = class_loader::ClassLoader::UniquePtr<fuse_core::MotionModel>;
  using MotionModels = std::unordered_map<std::string, MotionModelUniquePtr>;
  using PublisherUniquePtr = class_loader::ClassLoader::UniquePtr<fuse_core::Publisher>;
  using Publishers = std::unordered_map<std::string, PublisherUniquePtr>;
  using SensorModelUniquePtr = class_loader::ClassLoader::UniquePtr<fuse_core::SensorModel>;

  /**
   * @brief A struct to hold the sensor model and whether it is an ignition one or not
   */
  struct SensorModelInfo
  {
    /**
     * @brief Constructor
     *
     * @param[in] model The sensor model
     * @param[in] ignition Whether this sensor model is an ignition one or not
     */
    SensorModelInfo(SensorModelUniquePtr model, const bool ignition) : model(std::move(model)), ignition(ignition)
    {
    }

    SensorModelUniquePtr model;  //!< The sensor model
    bool ignition;               //!< Whether this sensor model is an ignition one or not
  };

  using SensorModels = std::unordered_map<std::string, SensorModelInfo>;

  // Some internal book-keeping data structures
  using MotionModelGroup = std::vector<std::string>;  //!< A set of motion model names
  using AssociatedMotionModels = std::unordered_map<std::string, MotionModelGroup>;  //!< sensor -> motion models group

  AssociatedMotionModels associated_motion_models_;  //!< Tracks what motion models should be used for each sensor
  fuse_core::Graph::UniquePtr graph_;  //!< The graph object that holds all variables and constraints

  pluginlib::ClassLoader<fuse_core::MotionModel> motion_model_loader_;  //!< Pluginlib class loader for MotionModels
  MotionModels motion_models_;  //!< The set of motion models, addressable by name
  pluginlib::ClassLoader<fuse_core::Publisher> publisher_loader_;  //!< Pluginlib class loader for Publishers
  Publishers publishers_;  //!< The set of publishers to execute after every graph optimization
  pluginlib::ClassLoader<fuse_core::SensorModel> sensor_model_loader_;  //!< Pluginlib class loader for SensorModels
  SensorModels sensor_models_;  //!< The set of sensor models, addressable by name

  diagnostic_updater::Updater diagnostic_updater_;  //!< Diagnostic updater
  rclcpp::TimerBase::SharedPtr diagnostic_updater_timer_; //!< Diagnostic updater timer
  double diagnostic_updater_timer_period_{ 1.0 };  //!< Diagnostic updater timer period in seconds

  std::shared_ptr<fuse_core::CallbackAdapter> callback_queue_;


  /**
   * @brief Callback fired every time a SensorModel plugin creates a new transaction
   *
   * @param[in] sensor_name The name of the sensor that produced the Transaction
   * @param[in] stamps      Any timestamps associated with the added variables. These are sent to the motion models
   *                        to generate connected constraints.
   * @param[in] transaction The populated Transaction object created by the loaded SensorModel plugin
   */
  virtual void transactionCallback(
    const std::string& sensor_name,
    fuse_core::Transaction::SharedPtr transaction) = 0;

  /**
   * @brief Configure the motion model plugins specified on the parameter server
   *
   * Will throw if the parameter server configuration is invalid.
   */
  void loadMotionModels();

  /**
   * @brief Configure the publisher plugins specified on the parameter server
   *
   * Will throw if the parameter server configuration is invalid.
   */
  void loadPublishers();

  /**
   * @brief Configure the sensor model plugins specified on the parameter server
   *
   * Will throw if the parameter server configuration is invalid.
   */
  void loadSensorModels();

  /**
   * @brief Given a transaction and some timestamps, augment the transaction with constraints from all associated
   * motion models.
   *
   * If no timestamps are provided, or no motion models are associated with this sensor, the transaction is left
   * unmodified. If an associated motion model is unavailable, this will throw an exception.
   *
   * @param[in]  name        The name of the sensor that produced the Transaction
   * @param[in]  timestamps  Any timestamps associated with the added variables. These are sent to the motion models
   *                         to generate connected constraints.
   * @param[out] transaction The Transaction object will be augmented with constraints and variables from the motion
   *                         models
   * @return                 Flag indicating if all motion model constraints were successfully generated
   */
  bool applyMotionModels(
    const std::string& sensor_name,
    fuse_core::Transaction& transaction) const;

  /**
   * @brief Send the sensors, motion models, and publishers updated graph information
   *
   * @param[in] transaction A read-only pointer to a transaction containing all recent additions and removals
   * @param[in] graph       A read-only pointer to the graph object
   */
  void notify(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph);

  /**
   * @brief Inject a transaction callback function into the global callback queue
   *
   * @param[in] sensor_name The name of the sensor that produced the Transaction
   * @param[in] transaction The populated Transaction object created by the loaded SensorModel plugin
   */
  void injectCallback(
    const std::string& sensor_name,
    fuse_core::Transaction::SharedPtr transaction);

  /**
   * @brief Clear all of the callbacks inserted into the callback queue by the injectCallback() method
   */
  void clearCallbacks();

  /**
   * @brief Start all configured plugins (motion models, publishers, and sensor models)
   */
  void startPlugins();

  /**
   * @brief Stop all configured plugins (motion models, publishers, and sensor models)
   */
  void stopPlugins();

  /**
   * @brief Update and publish diagnotics
   * @param[in] status The diagnostic status
   */
  virtual void setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_OPTIMIZER_H
