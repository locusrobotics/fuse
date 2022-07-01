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
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_optimizers/optimizer.h>
#include <fuse_graphs/hash_graph.h>

//#include <XmlRpcValue.h>

#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace fuse_optimizers
{

// XXX pass a rclcpp::Context, and a CallbackGroup
Optimizer::Optimizer(
  rclcpp::NodeOptions options,
  std::string node_name = "optimizer_node",
  fuse_core::Graph::UniquePtr graph = fuse_graphs::HashGraph::make_unique()
  ) :
    Node(node_name, options),
    graph_(std::move(graph)),
    motion_model_loader_("fuse_core", "fuse_core::MotionModel"),
    publisher_loader_("fuse_core", "fuse_core::Publisher"),
    sensor_model_loader_("fuse_core", "fuse_core::SensorModel"),
    diagnostic_updater_(shared_from_this()),
    callback_queue_(std::make_shared<fuse_core::CallbackAdapter>(rclcpp::contexts::get_global_default_context()))
{
  // Setup diagnostics updater
  // XXX private_node_handle_.param("diagnostic_updater_timer_period", diagnostic_updater_timer_period_,
  // XXX                            diagnostic_updater_timer_period_);

  diagnostic_updater_timer_ = this->create_wall_timer(
    diagnostic_updater_timer_period_,
    std::bind(&diagnostic_updater::Updater::update, &diagnostic_updater_)
  );

  //add a ros1 style callback queue so that transactions can be processed in the optimiser's executor
  this->get_node_waitables_interface()->add_waitable(
    callback_queue_, (rclcpp::CallbackGroup::SharedPtr) nullptr);

  diagnostic_updater_.add(this->get_namespace(), this, &Optimizer::setDiagnostics);
  diagnostic_updater_.setHardwareID("fuse");

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
  const std::string param_prefix = "motion_models/";
  const std::string list_param_full_name = param_prefix + "motion_model_list";

  // struct for readability
  typedef struct {
    std::string name;
    std::string type;
    std::string param_name;
  } ModelConfig;

  // the configurations used to load models
  std::vector<ModelConfig> motion_model_config;

  // declare the parameter
  if(! this->has_parameter(list_param_full_name)){
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = "the list of motion models to load";
    this->declare_parameter(
      list_param_full_name,
      rclcpp::ParameterValue (std::vector< std::string >()),
      descr
    );
    // XXX catch rclcpp::exceptions::InvalidParameterValueException when launch assigns wrong type
  }

  // get the list of motion models
  rclcpp::Parameter motion_model_list_param = this->get_parameter(list_param_full_name);

  //extract the list from the parameter
  if(motion_model_list_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY){
    std::vector< std::string > names = motion_model_list_param.as_string_array();
    for(std::string name : names){
      ModelConfig config;
      config.name = name;
      motion_model_config.push_back(std::move(config));
    }
  }

  // declare config parameters for each model
  for(ModelConfig & config : motion_model_config){
    config.param_name = param_prefix + config.name + "/type";

    if(! this->has_parameter(config.param_name)){
      rcl_interfaces::msg::ParameterDescriptor descr;
      descr.description = "the PLUGINLIB type string to load for this motion_model (eg: 'fuse_models::Unicycle2D')";
      this->declare_parameter(
        config.param_name,
        rclcpp::ParameterValue (std::string ()),
        descr
      );
    }

    // get the type parameter for the motion model
    rclcpp::Parameter motion_model_type_param = this->get_parameter(config.param_name);
    //extract the type string from the parameter
    if(motion_model_type_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
      config.type = motion_model_type_param.as_string();
    }

    // quickly check for common errors
    if(config.type == ""){
      RCLCPP_WARN_STREAM(this->get_logger(),
        "parameter '" << config.param_name <<
        "' should be the string of a motion_model type " <<
        "for the motion_model named '" << config.name <<
        "'.");
    }
  }

  // now load the models defined above

  for(const ModelConfig & config : motion_model_config){
    // Create a motion_model object using pluginlib. This will throw if the plugin name is not found.
    auto motion_model = motion_model_loader_.createUniqueInstance(config.type);
    // Initialize the motion_model
    motion_model->initialize(config.name);
    // Store the motion_model in a member variable for use later
    motion_models_.emplace(config.name, std::move(motion_model));
  }

  diagnostic_updater_.force_update();
}

void Optimizer::loadSensorModels()
{
  const std::string param_prefix = "sensor_models/";
  const std::string list_param_full_name = param_prefix + "sensor_model_list";

  // struct for readability
  typedef struct {
    std::string name;
    std::string type;
    bool ignition;
    std::vector<std::string> associated_motion_models;
    std::string type_param_name;
    std::string models_param_name;
    std::string ignition_param_name;
  } ModelConfig;

  // the configurations used to load models
  std::vector<ModelConfig> sensor_model_config;

  // declare the parameter
  if(! this->has_parameter(list_param_full_name)){
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = "the list of sensor models to load";
    this->declare_parameter(
      list_param_full_name,
      rclcpp::ParameterValue (std::vector< std::string >()),
      descr
    );
    // XXX catch rclcpp::exceptions::InvalidParameterValueException when launch assigns wrong type
  }

  // get the list of sensor models
  rclcpp::Parameter sensor_model_list_param = this->get_parameter(list_param_full_name);

  //extract the list from the parameter
  if(sensor_model_list_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY){
    std::vector< std::string > names = sensor_model_list_param.as_string_array();
    for(std::string name : names){
      ModelConfig config;
      config.name = name;
      sensor_model_config.push_back(std::move(config));
    }
  }

  // declare config parameters for each model
  for(ModelConfig & config : sensor_model_config){
    config.type_param_name = param_prefix + config.name + "/type";
    config.models_param_name = param_prefix + config.name + "/motion_models";
    config.ignition_param_name = param_prefix + config.name + "/ignition";


    // get the type parameter for the sensor model
    if(! this->has_parameter(config.type_param_name)){
      rcl_interfaces::msg::ParameterDescriptor descr;
      descr.description = "the PLUGINLIB type string to load for this sensor_model (eg: 'fuse_models::Acceleration2D')";
      this->declare_parameter(
        config.type_param_name,
        rclcpp::ParameterValue (std::string ()),
        descr
      );
    }

    // get the type parameter for the sensor model
    rclcpp::Parameter sensor_model_type_param = this->get_parameter(config.type_param_name);
    //extract the type string from the parameter
    if(sensor_model_type_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
      config.type = sensor_model_type_param.as_string();
    }


    // get the type parameter for the sensor model
    if(! this->has_parameter(config.models_param_name)){
      rcl_interfaces::msg::ParameterDescriptor descr;
      descr.description = "the list of motion models this sensor is associated with";
      this->declare_parameter(
        config.models_param_name,
        rclcpp::ParameterValue (std::vector< std::string >()),
        descr
      );
    }

    // get the model_list parameter for the sensor model
    rclcpp::Parameter sensor_model_model_list_param = this->get_parameter(config.models_param_name);
    //extract the model_list string from the parameter
    if(sensor_model_model_list_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY){
      config.associated_motion_models = sensor_model_model_list_param.as_string_array();
    }


    // get the ignition parameter for the sensor model
    if(! this->has_parameter(config.ignition_param_name)){
      rcl_interfaces::msg::ParameterDescriptor descr;
      descr.description = "does the first message for this sensor start the optimizer";
      this->declare_parameter(
        config.ignition_param_name,
        rclcpp::ParameterValue (false),
        descr
      );
    }

    // get the model list parameter for the sensor model
    rclcpp::Parameter sensor_model_ignition_param = this->get_parameter(config.ignition_param_name);
    //extract the ignition bool from the parameter
    if(sensor_model_ignition_param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
      config.ignition = sensor_model_ignition_param.as_bool();
    }


    // quickly check for common errors
    if(config.type == ""){
      RCLCPP_WARN_STREAM(this->get_logger(),
        "parameter '" << config.type_param_name <<
        "' should be the string of a sensor_model type " <<
        "for the sensor_model named '" << config.name <<
        "'.");
    }

  }

  // now load the models defined above
  for(const ModelConfig & config : sensor_model_config){
    // Create a sensor object using pluginlib. This will throw if the plugin name is not found.
    auto sensor_model = sensor_model_loader_.createUniqueInstance(config.type);
    // Initialize the sensor
    sensor_model->initialize(
      config.name,
      std::bind(&Optimizer::injectCallback, this, config.name, std::placeholders::_1));
    // Store the sensor in a member variable for use later
    sensor_models_.emplace(config.name,
                           SensorModelInfo{ std::move(sensor_model), config.ignition });  // NOLINT(whitespace/braces)

    // Parse out the list of associated motion models, if any
    associated_motion_models_[config.name] = config.associated_motion_models;

    for (const auto & motion_model_name : config.associated_motion_models)
    {
      if (motion_models_.find(motion_model_name) == motion_models_.end())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Sensor model '" << config.name << "' is configured to use motion model '" <<
                        motion_model_name << "', but no motion model with that name currently exists. This is " <<
                        "likely a configuration error.");
      }
    }
  }
  diagnostic_updater_.force_update();
}








void Optimizer::loadPublishers()
{

  const std::string param_prefix = "publishers/";
  const std::string list_param_full_name = param_prefix + "publisher_list";

  // struct for readability
  typedef struct {
    std::string name;
    std::string type;
    std::string param_name;
  } PublisherConfig;

  // the configurations used to load models
  std::vector<PublisherConfig> publisher_config;

  // declare the parameter
  if(! this->has_parameter(list_param_full_name)){
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = "the list of publishers to load";
    this->declare_parameter(
      list_param_full_name,
      rclcpp::ParameterValue (std::vector< std::string >()),
      descr
    );
    // XXX catch rclcpp::exceptions::InvalidParameterValueException when launch assigns wrong type
  }

  // get the list of publishers
  rclcpp::Parameter publisher_list_param = this->get_parameter(list_param_full_name);

  //extract the list from the parameter
  if(publisher_list_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY){
    std::vector< std::string > names = publisher_list_param.as_string_array();
    for(std::string name : names){
      PublisherConfig config;
      config.name = name;
      publisher_config.push_back(std::move(config));
    }
  }

  // declare config parameters for each model
  for(PublisherConfig & config : publisher_config){
    config.param_name = param_prefix + config.name + "/type";

    if(! this->has_parameter(config.param_name)){
      rcl_interfaces::msg::ParameterDescriptor descr;
      descr.description = "the PLUGINLIB type string to load for this publisher (eg: 'fuse_publishers::Path2DPublisher')";
      this->declare_parameter(
        config.param_name,
        rclcpp::ParameterValue (std::string ()),
        descr
      );
    }

    // get the type parameter for the publisher
    rclcpp::Parameter publisher_type_param = this->get_parameter(config.param_name);
    //extract the type string from the parameter
    if(publisher_type_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
      config.type = publisher_type_param.as_string();
    }

    // quickly check for common errors
    if(config.type == ""){
      RCLCPP_WARN_STREAM(this->get_logger(),
        "parameter '" << config.param_name <<
        "' should be the string of a publisher type " <<
        "for the publisher named '" << config.name <<
        "'.");
    }
  }

  // now load the models defined above

  for(const PublisherConfig & config : publisher_config){
    // Create a publisher object using pluginlib. This will throw if the plugin name is not found.
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error generating constraints for sensor '" << sensor_name << "' "
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed calling graphCallback() on sensor '" << name__sensor_model.first << "'. " <<
                       "Error: " << e.what());
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed calling graphCallback() on motion model '" << name__motion_model.first << "." <<
                       " Error: " << e.what());
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed calling notify() on publisher '" << name__publisher.first << "." <<
                       " Error: " << e.what());
      continue;
    }
  }
}

void Optimizer::injectCallback(
  const std::string& sensor_name,
  fuse_core::Transaction::SharedPtr transaction)
{
  // motion models and sensor models are handed this function to run as their transactionCallback() method
  // This function inserts the transaction into the optimiser's callback queue.
  // This returns execution to the sensor's thread quickly by moving the transaction processing to the optimizer's
  // thread. And by using the existing ROS callback queue, we simplify the threading model of the optimizer.

  auto callback = std::make_shared<fuse_core::CallbackWrapper<void>>(
      std::bind(&Optimizer::transactionCallback, this, sensor_name, std::move(transaction)));
  callback_queue_->addCallback(callback);
}

void Optimizer::clearCallbacks()
{
  callback_queue_->removeAllCallbacks();
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

  // TODO test for previous convergence success or failure

  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Optimizer exists");

  auto print_key = [](const std::string& result, const auto& entry) { return result + entry.first + ' '; };

  status.add("Sensor Models", std::accumulate(sensor_models_.begin(), sensor_models_.end(), std::string(), print_key));
  status.add("Motion Models", std::accumulate(motion_models_.begin(), motion_models_.end(), std::string(), print_key));
  status.add("Publishers", std::accumulate(publishers_.begin(), publishers_.end(), std::string(), print_key));
}

}  // namespace fuse_optimizers
