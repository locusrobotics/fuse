// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FUSE_CORE__NODE_INTERFACES__NODE_INTERFACES_HPP_
#define FUSE_CORE__NODE_INTERFACES__NODE_INTERFACES_HPP_

#include <memory>

#include "fuse_core/node_interfaces/node_interfaces_helpers.hpp"

namespace fuse_core
{
namespace node_interfaces
{

/// TODO(CH3): Remove this once https://github.com/ros2/rclcpp/pull/2041 is merged and released

/// A helper class for aggregating node interfaces
template<typename ... InterfaceTs>
class NodeInterfaces
  : public std::enable_shared_from_this<NodeInterfaces<InterfaceTs...>>, public InterfaceTs ...
{
  static_assert(0 != sizeof ...(InterfaceTs), "Template parameters must be populated!");

public:
  RCLCPP_SMART_PTR_DEFINITIONS(NodeInterfaces)

  /// Create a new NodeInterfaces object with no bound node interfaces.
  NodeInterfaces()
  : InterfaceTs()... {}

  /// Create a new NodeInterfaces object bound with the passed in node-like object's interfaces.
  /**
   * Specify which interfaces you want to bind using the template parameters by specifying
   * interface support classes to use. Any unmentioned interfaces will be unavailable to bind.
   *
   * You may use any of the available support classes in
   * node_interfaces/node_interfaces_helpers.hpp:
   *   - Base:       Supports NodeBaseInterface
   *   - Clock:      Supports NodeClockInterface
   *   - Graph:      Supports NodeGraphInterface
   *   - Logging:    Supports NodeLoggingInterface
   *   - Parameters: Supports NodeParametersInterface
   *   - Services:   Supports NodeServicesInterface
   *   - TimeSource: Supports NodeTimeSourceInterface
   *   - Timers:     Supports NodeTimersInterface
   *   - Topics:     Supports NodeTopicsInterface
   *   - Waitables:  Supports NodeWaitablesInterface
   *
   * Or you can define your own interface support classes!
   *
   * Each of the support classes should define:
   *   - Default constructor
   *   - Templated constructor taking NodeT
   *   - get_node_<interface_name>_interface()
   *   - set_node_<interface_name>_interface()
   *
   * Usage example:
   *   - ```NodeInterfaces<rclcpp::node_interfaces::Base>(node)``` will bind just the
   *        NodeBaseInterface.
   *   - ```NodeInterfaces< rclcpp::node_interfaces::Base,
   *        rclcpp::node_interfaces::Clock>(node)``` will bind both the NodeBaseInterface and
   *        NodeClockInterface.
   *
   * \param[in] node Node-like object to bind the interfaces of.
   */
  template<typename NodeT>
  NodeInterfaces(NodeT & node)  // NOLINT(runtime/explicit)
  : InterfaceTs(node)... {}           // Implicit constructor for node-like passing to functions

  /// SharedPtr Constructor
  template<typename NodeT>
  NodeInterfaces(std::shared_ptr<NodeT> node)  // NOLINT(runtime/explicit)
  : InterfaceTs(node ? *node : throw std::runtime_error("Passed in NodeT is nullptr!"))... {}
};


/// Create a new NodeInterfaces object bound with no node interfaces.
/**
 * Specify which interfaces you want to bind using the template parameters by specifying
 * interface support classes to use. Any unmentioned interfaces will be unavailable to bind.
 *
 * This method will return a NodeInterfaces with no bound interfaces. You must set them using
 * ```NodeInterfaces->set_<interface_name>_interface(InterfaceT::SharedPtr interface)```
 *
 * See the rclcpp::node_interfaces::NodeInterfaces class for usage examples and support classes.
 *
 * \sa rclcpp::node_interfaces::NodeInterfaces
 * \param[in] node Node-like object to bind the interfaces of.
 * \returns a NodeInterfaces::SharedPtr supporting the stated interfaces, but bound with none of
 *          them
 */
template<typename ... InterfaceTs>
typename NodeInterfaces<InterfaceTs...>::SharedPtr
get_node_interfaces()
{
  static_assert(0 != sizeof ...(InterfaceTs), "Template parameters must be populated!");
  return std::make_shared<NodeInterfaces<InterfaceTs...>>();
}

/// Create a new NodeInterfaces object bound with the passed in node-like object's interfaces.
/**
 * Specify which interfaces you want to bind using the template parameters by specifying
 * interface support classes to use. Any unmentioned interfaces will be unavailable to bind.
 *
 * See the rclcpp::node_interfaces::NodeInterfaces class for usage examples and support classes.
 *
 * \sa rclcpp::node_interfaces::NodeInterfaces
 * \param[in] node Node-like object to bind the interfaces of.
 * \returns a NodeInterfaces::SharedPtr bound with the node-like objects's interfaces
 */
template<typename ... InterfaceTs, typename NodeT>
typename NodeInterfaces<InterfaceTs...>::SharedPtr
get_node_interfaces(NodeT & node)
{
  static_assert(0 != sizeof ...(InterfaceTs), "Template parameters must be populated!");
  return std::make_shared<NodeInterfaces<InterfaceTs...>>(node);
}

/// Create a new NodeInterfaces object bound with the passed in node-like shared_ptr's interfaces.
/**
 * Specify which interfaces you want to bind using the template parameters by specifying
 * interface support classes to use. Any unmentioned interfaces will be unavailable to bind.
 *
 * See the rclcpp::node_interfaces::NodeInterfaces class for usage examples and support classes.
 *
 * \sa rclcpp::node_interfaces::NodeInterfaces
 * \param[in] node Node-like object to bind the interfaces of.
 * \returns a NodeInterfaces::SharedPtr bound with the node-like objects's interfaces
 */
template<typename ... InterfaceTs, typename NodeT>
typename NodeInterfaces<InterfaceTs...>::SharedPtr
get_node_interfaces(std::shared_ptr<NodeT> node)
{
  static_assert(0 != sizeof ...(InterfaceTs), "Template parameters must be populated!");
  return std::make_shared<NodeInterfaces<InterfaceTs...>>(node);
}

}  // namespace node_interfaces
}  // namespace fuse_core

#endif  // FUSE_CORE__NODE_INTERFACES__NODE_INTERFACES_HPP_
