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
#include <fuse_core/callback_wrapper.h>
#include <fuse_core/graph.h>
#include <fuse_core/time.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_optimizers/optimizer.h>
#include <ros/callback_queue.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <XmlRpcValue.h>

#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace fuse_optimizers
{

/**
 * @brief Plugin name and type configuration, as received from the parameter server
 *
 * The entire configuration itself is also stored, so additional optional parameters can be retrieved, e.g. the
 * 'motion_models' parameter for the SensorModel plugins
 */
struct PluginConfig
{
  /**
   * @brief Constructor. This allows to use emplace_back(name, type, config) instead of push_back({name, type, config}),
   * that generates roslint whitespace/braces errors
   *
   * @param[in] name   The plugin name
   * @param[in] type   The plugin type
   * @param[in] config The entire configuration, that might have additiona optional parameters
   */
  PluginConfig(const std::string& name, const std::string& type, const XmlRpc::XmlRpcValue& config)
    : name(name), type(type), config(config)
  {
  }

  std::string name;            //!< Plugin name
  std::string type;            //!< Plugin type
  XmlRpc::XmlRpcValue config;  //!< The entire configuration, that might have additional optional parameters
};

Optimizer::Optimizer(
  fuse_core::Graph::UniquePtr graph,
  const ros::NodeHandle& node_handle,
  const ros::NodeHandle& private_node_handle) :
    graph_(std::move(graph)),
    node_handle_(node_handle),
    private_node_handle_(private_node_handle),
    motion_model_loader_("fuse_core", "fuse_core::MotionModel"),
    publisher_loader_("fuse_core", "fuse_core::Publisher"),
    sensor_model_loader_("fuse_core", "fuse_core::SensorModel"),
    diagnostic_updater_(node_handle_)
{
  // Setup diagnostics updater
  private_node_handle_.param("diagnostic_updater_timer_period", diagnostic_updater_timer_period_,
                             diagnostic_updater_timer_period_);

  diagnostic_updater_timer_ = this->create_timer(
    rclcpp::Duration::from_seconds(diagnostic_updater_timer_period_),
    std::bind(&diagnostic_updater::Updater::update, &diagnostic_updater_)
  );

  diagnostic_updater_.add(private_node_handle_.getNamespace(), this, &Optimizer::setDiagnostics);
  diagnostic_updater_.setHardwareID("fuse");

  // Wait for a valid time before loading any of the plugins
  fuse_core::wait_for_valid(this->get_node_clock_interface()->get_clock());

  // Load all configured plugins
  loadMotionModels();
  loadSensorModels();
  loadPublishers();

  // Start all the plugins
  startPlugins();
}

Optimizer::~Optimizer()
{
  // Stop all the plugins
  stopPlugins();
}

void Optimizer::loadMotionModels()
{
  // Read and configure all of sensor model plugins
  if (!private_node_handle_.hasParam("motion_models"))
  {
    return;
  }
  // Validate the parameter server values
  XmlRpc::XmlRpcValue motion_models;
  private_node_handle_.getParam("motion_models", motion_models);
  std::vector<PluginConfig> motion_model_configs;
  if (motion_models.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    // Validate all of the parameters before we attempt to create any plugin instances
    for (int32_t motion_model_index = 0; motion_model_index < motion_models.size(); ++motion_model_index)
    {
      // Validate the parameter server values
      const auto& motion_model = motion_models[motion_model_index];
      if ( (motion_model.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        || (!motion_model.hasMember("name"))
        || (!motion_model.hasMember("type")))
      {
        throw std::invalid_argument("The 'motion_models' parameter should be a list of the form: "
                                    "-{name: string, type: string}");
      }

      motion_model_configs.emplace_back(static_cast<std::string>(motion_model["name"]),
                                        static_cast<std::string>(motion_model["type"]), motion_model);
    }
  }
  else if (motion_models.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    // Validate all of the parameters before we attempt to create any plugin instances
    for (const auto& motion_model : motion_models)
    {
      const auto& motion_model_config = motion_model.second;
      if ( (motion_model_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        || (!motion_model_config.hasMember("type")))
      {
        throw std::invalid_argument("The 'motion_models' parameter should be a struct of the form: "
                                    "{string: {type: string}}");
      }

      motion_model_configs.emplace_back(static_cast<std::string>(motion_model.first),
                                        static_cast<std::string>(motion_model_config["type"]), motion_model_config);
    }
  }
  else
  {
    throw std::invalid_argument("The 'motion_models' parameter should be a list of the form: "
                                "-{name: string, type: string} or a struct of the form: {string: {type: string}}");
  }

  for (const auto& config : motion_model_configs)
  {
    // Create a motion model object using pluginlib. This will throw if the plugin name is not found.
    auto motion_model = motion_model_loader_.createUniqueInstance(config.type);
    // Initialize the publisher
    motion_model->initialize(config.name);
    // Store the publisher in a member variable for use later
    motion_models_.emplace(config.name, std::move(motion_model));
  }

  diagnostic_updater_.force_update();
}

void Optimizer::loadSensorModels()
{
  // Read and configure all of sensor model plugins
  if (!private_node_handle_.hasParam("sensor_models"))
  {
    return;
  }
  // Validate the parameter server values
  XmlRpc::XmlRpcValue sensor_models;
  private_node_handle_.getParam("sensor_models", sensor_models);
  std::vector<PluginConfig> sensor_model_configs;
  if (sensor_models.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    // Validate all of the parameters before we attempt to create any plugin instances
    for (int32_t sensor_model_index = 0; sensor_model_index < sensor_models.size(); ++sensor_model_index)
    {
      // Validate the parameter server values
      const auto& sensor_model = sensor_models[sensor_model_index];
      if ( (sensor_model.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        || (!sensor_model.hasMember("name"))
        || (!sensor_model.hasMember("type")))
      {
        throw std::invalid_argument("The 'sensor_models' parameter should be a list of the form: "
                                    "-{name: string, type: string, motion_models: [name1, name2, ...]}");
      }

      sensor_model_configs.emplace_back(static_cast<std::string>(sensor_model["name"]),
                                        static_cast<std::string>(sensor_model["type"]), sensor_model);
    }
  }
  else if (sensor_models.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    // Validate all of the parameters before we attempt to create any plugin instances
    for (const auto& sensor_model : sensor_models)
    {
      // Validate the parameter server values
      const auto& sensor_model_config = sensor_model.second;
      if ( (sensor_model_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        || (!sensor_model_config.hasMember("type")))
      {
        throw std::invalid_argument("The 'sensor_models' parameter should be a struct of the form: "
                                    "{string: {type: string, motion_models: [name1, name2, ...]}}");
      }

      sensor_model_configs.emplace_back(static_cast<std::string>(sensor_model.first),
                                        static_cast<std::string>(sensor_model_config["type"]), sensor_model_config);
    }
  }
  else
  {
    throw std::invalid_argument("The 'sensor_models' parameter should be a list of the form: "
                                "-{name: string, type: string, motion_models: [name1, name2, ...]} "
                                "or a struct of the form: "
                                "{string: {type: string, motion_models: [name1, name2, ...]}}");
  }

  for (const auto& config : sensor_model_configs)
  {
    // Check whether this is an ignition sensor model or not
    const bool ignition = config.config.hasMember("ignition") ? static_cast<bool>(config.config["ignition"]) : false;

    // Create a sensor object using pluginlib. This will throw if the plugin name is not found.
    auto sensor_model = sensor_model_loader_.createUniqueInstance(config.type);
    // Initialize the sensor
    sensor_model->initialize(
      config.name,
      std::bind(&Optimizer::injectCallback, this, config.name, std::placeholders::_1));
    // Store the sensor in a member variable for use later
    sensor_models_.emplace(config.name,
                           SensorModelInfo{ std::move(sensor_model), ignition });  // NOLINT(whitespace/braces)

    // Parse out the list of associated motion models, if any
    if ( (config.config.hasMember("motion_models"))
      && (config.config["motion_models"].getType() == XmlRpc::XmlRpcValue::TypeArray))
    {
      XmlRpc::XmlRpcValue motion_model_list = config.config["motion_models"];
      for (int32_t motion_model_index = 0; motion_model_index < motion_model_list.size(); ++motion_model_index)
      {
        const auto motion_model_name = static_cast<std::string>(motion_model_list[motion_model_index]);
        associated_motion_models_[config.name].push_back(motion_model_name);
        if (motion_models_.find(motion_model_name) == motion_models_.end())
        {
          RCLCPP_WARN_STREAM(this->get_logger(),
                             "Sensor model '" << config.name << "' is configured to use motion model '"
                             << motion_model_name << "', but no motion model with that name currently exists. "
                             << "This is likely a configuration error.");
        }
      }
    }
  }

  diagnostic_updater_.force_update();
}

void Optimizer::loadPublishers()
{
  // Read and configure all of the publisher plugins
  if (!private_node_handle_.hasParam("publishers"))
  {
    return;
  }
  // Validate the parameter server values
  XmlRpc::XmlRpcValue publishers;
  private_node_handle_.getParam("publishers", publishers);
  std::vector<PluginConfig> publisher_configs;
  if (publishers.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    // Validate all of the parameters before we attempt to create any plugin instances
    for (int32_t publisher_index = 0; publisher_index < publishers.size(); ++publisher_index)
    {
      // Validate the parameter server values
      const auto& publisher = publishers[publisher_index];
      if ( (publisher.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        || (!publisher.hasMember("name"))
        || (!publisher.hasMember("type")))
      {
        throw std::invalid_argument("The 'publishers' parameter should be a list of the form: "
                                    "-{name: string, type: string}");
      }

      publisher_configs.emplace_back(static_cast<std::string>(publisher["name"]),
                                     static_cast<std::string>(publisher["type"]), publisher);
    }
  }
  else if (publishers.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    // Validate all of the parameters before we attempt to create any plugin instances
    for (const auto& publisher : publishers)
    {
      // Validate the parameter server values
      const auto& publisher_config = publisher.second;
      if ( (publisher_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        || (!publisher_config.hasMember("type")))
      {
        throw std::invalid_argument("The 'publishers' parameter should be a struct of the form: "
                                    "{string: {type: string}}");
      }

      publisher_configs.emplace_back(static_cast<std::string>(publisher.first),
                                     static_cast<std::string>(publisher_config["type"]), publisher_config);
    }
  }
  else
  {
    throw std::invalid_argument("The 'publishers' parameter should be a list of the form: "
                                "-{name: string, type: string} or a struct of the form: {string: {type: string}}");
  }

  for (const auto& config : publisher_configs)
  {
    // Create a Publisher object using pluginlib. This will throw if the plugin name is not found.
    auto publisher = publisher_loader_.createUniqueInstance(config.type);
    // Initialize the publisher
    publisher->initialize(config.name);
    // Store the publisher in a member variable for use later
    publishers_.emplace(config.name, std::move(publisher));
  }

  diagnostic_updater_.force_update();
}

bool Optimizer::applyMotionModels(
  const std::string& sensor_name,
  fuse_core::Transaction& transaction) const
{
  // Check for trivial cases where we don't have to do anything
  auto iter = associated_motion_models_.find(sensor_name);
  if (iter == associated_motion_models_.end())
  {
    return true;
  }
  // Generate constraints for each configured motion model
  const auto& motion_model_names = iter->second;
  bool success = true;
  for (const auto& motion_model_name : motion_model_names)
  {
    try
    {
      success &= motion_models_.at(motion_model_name)->apply(transaction);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Error generating constraints for sensor '" << sensor_name << "' "
                          << "from motion model '" << motion_model_name << "'. Error: " << e.what());
      success = false;
    }
  }
  return success;
}

void Optimizer::notify(
  fuse_core::Transaction::ConstSharedPtr transaction,
  fuse_core::Graph::ConstSharedPtr graph)
{
  for (const auto& name__sensor_model : sensor_models_)
  {
    try
    {
      name__sensor_model.second.model->graphCallback(graph);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Failed calling graphCallback() on sensor '" << name__sensor_model.first
                          << "'. Error: " << e.what());
      continue;
    }
  }
  for (const auto& name__motion_model : motion_models_)
  {
    try
    {
      name__motion_model.second->graphCallback(graph);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Failed calling graphCallback() on motion model '" << name__motion_model.first
                          << ". Error: " << e.what());
      continue;
    }
  }
  for (const auto& name__publisher : publishers_)
  {
    try
    {
      name__publisher.second->notify(transaction, graph);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Failed calling notify() on publisher '" << name__publisher.first
                          << ". Error: " << e.what());
      continue;
    }
  }
}

void Optimizer::injectCallback(
  const std::string& sensor_name,
  fuse_core::Transaction::SharedPtr transaction)
{
  // We are going to insert a call to the derived class's transactionCallback() method into the global callback queue.
  // This returns execution to the sensor's thread quickly by moving the transaction processing to the optimizer's
  // thread. And by using the existing ROS callback queue, we simplify the threading model of the optimizer.
  ros::getGlobalCallbackQueue()->addCallback(
    boost::make_shared<fuse_core::CallbackWrapper<void>>(
      std::bind(&Optimizer::transactionCallback, this, sensor_name, std::move(transaction))),
    reinterpret_cast<uint64_t>(this));
}

void Optimizer::clearCallbacks()
{
  ros::getGlobalCallbackQueue()->removeByID(reinterpret_cast<uint64_t>(this));
}

void Optimizer::startPlugins()
{
  for (const auto& name_plugin : motion_models_)
  {
    name_plugin.second->start();
  }
  for (const auto& name_plugin : sensor_models_)
  {
    name_plugin.second.model->start();
  }
  for (const auto& name_plugin : publishers_)
  {
    name_plugin.second->start();
  }

  diagnostic_updater_.force_update();
}

void Optimizer::stopPlugins()
{
  for (const auto& name_plugin : publishers_)
  {
    name_plugin.second->stop();
  }
  for (const auto& name_plugin : sensor_models_)
  {
    name_plugin.second.model->stop();
  }
  for (const auto& name_plugin : motion_models_)
  {
    name_plugin.second->stop();
  }

  diagnostic_updater_.force_update();
}

void Optimizer::setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
{
  if (!fuse_core::is_valid(this->get_node_clock_interface()->get_clock()))
  {
    status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Waiting for valid ROS time");
    return;
  }

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Optimization converged");

  auto print_key = [](const std::string& result, const auto& entry) { return result + entry.first + ' '; };

  status.add("Sensor Models", std::accumulate(sensor_models_.begin(), sensor_models_.end(), std::string(), print_key));
  status.add("Motion Models", std::accumulate(motion_models_.begin(), motion_models_.end(), std::string(), print_key));
  status.add("Publishers", std::accumulate(publishers_.begin(), publishers_.end(), std::string(), print_key));
}

}  // namespace fuse_optimizers
