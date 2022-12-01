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

#ifndef FUSE_CORE__NODE_INTERFACES__NODE_INTERFACES_HELPERS_HPP_
#define FUSE_CORE__NODE_INTERFACES__NODE_INTERFACES_HELPERS_HPP_

#include <memory>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_time_source_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/visibility_control.hpp"

namespace fuse_core
{
namespace node_interfaces
{

/// TODO(CH3): Remove this once https://github.com/ros2/rclcpp/pull/2041 is merged and released

// Helper classes to be inherited by NodeInterfaces to support node interface aggregation
// via multiple inheritance.

// These also provide a more terse way to configure the supported interfaces!


/// NodeInterfaces support for NodeBaseInterface
class Base
{
public:
  /// Default constructor with no bound NodeBaseInterface
  RCLCPP_PUBLIC
  Base() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeBaseInterface
  template<class NodeT>
  explicit Base(NodeT & node) {impl_ = node.get_node_base_interface();}

  /// Return the bound NodeBaseInterface
  inline
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return impl_;
  }

  /// Set the bound NodeBaseInterface
  inline
  void set_node_base_interface(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeClockInterface
class Clock
{
public:
  /// Default constructor with no bound NodeClockInterface
  RCLCPP_PUBLIC
  Clock() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeClockInterface
  template<class NodeT>
  explicit Clock(NodeT & node) {impl_ = node.get_node_clock_interface();}

  /// Return the bound NodeClockInterface
  inline
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr get_node_clock_interface()
  {
    return impl_;
  }

  /// Set the bound NodeClockInterface
  inline
  void set_node_clock_interface(rclcpp::node_interfaces::NodeClockInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeGraphInterface
class Graph
{
public:
  /// Default constructor with no bound NodeGraphInterface
  RCLCPP_PUBLIC
  Graph() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeGraphInterface
  template<class NodeT>
  explicit Graph(NodeT & node) {impl_ = node.get_node_graph_interface();}

  /// Return the bound NodeGraphInterface
  inline
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr get_node_graph_interface()
  {
    return impl_;
  }

  /// Set the bound NodeGraphInterface
  inline
  void set_node_graph_interface(rclcpp::node_interfaces::NodeGraphInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeLoggingInterface
class Logging
{
public:
  /// Default constructor with no bound NodeLoggingInterface
  RCLCPP_PUBLIC
  Logging() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeLoggingInterface
  template<class NodeT>
  explicit Logging(NodeT & node) {impl_ = node.get_node_logging_interface();}

  /// Return the bound NodeLoggingInterface
  inline
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr get_node_logging_interface()
  {
    return impl_;
  }

  /// Set the bound NodeLoggingInterface
  inline
  void
  set_node_logging_interface(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeParametersInterface
class Parameters
{
public:
  /// Default constructor with no bound NodeParametersInterface
  RCLCPP_PUBLIC
  Parameters() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeParametersInterface
  template<class NodeT>
  explicit Parameters(NodeT & node) {impl_ = node.get_node_parameters_interface();}

  /// Return the bound NodeParametersInterface
  inline
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface()
  {
    return impl_;
  }

  /// Set the bound NodeParametersInterface
  inline
  void
  set_node_parameters_interface(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeServicesInterface
class Services
{
public:
  /// Default constructor with no bound NodeServicesInterface
  RCLCPP_PUBLIC
  Services() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeServicesInterface
  template<class NodeT>
  explicit Services(NodeT & node) {impl_ = node.get_node_services_interface();}

  /// Return the bound NodeServicesInterface
  inline
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr get_node_services_interface()
  {
    return impl_;
  }

  /// Set the bound NodeServicesInterface
  inline
  void
  set_node_services_interface(rclcpp::node_interfaces::NodeServicesInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeTimeSourceInterface
class TimeSource
{
public:
  /// Default constructor with no bound NodeTimeSourceInterface
  RCLCPP_PUBLIC
  TimeSource() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTimeSourceInterface
  template<class NodeT>
  explicit TimeSource(NodeT & node) {impl_ = node.get_node_time_source_interface();}

  /// Return the bound NodeTimeSourceInterface
  inline
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr get_node_time_source_interface()
  {
    return impl_;
  }

  /// Set the bound NodeTimeSourceInterface
  inline
  void
  set_node_time_source_interface(
    rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeTimersInterface
class Timers
{
public:
  /// Default constructor with no bound NodeTimersInterface
  RCLCPP_PUBLIC
  Timers() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTimersInterface
  template<class NodeT>
  explicit Timers(NodeT & node) {impl_ = node.get_node_timers_interface();}

  /// Return the bound NodeTimersInterface
  inline
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr get_node_timers_interface()
  {
    return impl_;
  }

  /// Set the bound NodeTimersInterface
  inline
  void
  set_node_timers_interface(rclcpp::node_interfaces::NodeTimersInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeTopicsInterface
class Topics
{
public:
  /// Default constructor with no bound NodeTopicsInterface
  RCLCPP_PUBLIC
  Topics() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTopicsInterface
  template<class NodeT>
  explicit Topics(NodeT & node) {impl_ = node.get_node_topics_interface();}

  /// Return the bound NodeTopicsInterface
  inline
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface()
  {
    return impl_;
  }

  /// Set the bound NodeTopicsInterface
  inline
  void
  set_node_topics_interface(rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeWaitablesInterface
class Waitables
{
public:
  /// Default constructor with no bound NodeWaitablesInterface
  RCLCPP_PUBLIC
  Waitables() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeWaitablesInterface
  template<class NodeT>
  explicit Waitables(NodeT & node) {impl_ = node.get_node_waitables_interface();}

  /// Return the bound NodeWaitablesInterface
  inline
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr get_node_waitables_interface()
  {
    return impl_;
  }

  /// Set the bound NodeWaitablesInterface
  inline
  void
  set_node_waitables_interface(rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr interface)
  {
    impl_ = interface;
  }

private:
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr impl_;
};


}  // namespace node_interfaces
}  // namespace fuse_core

#endif  // FUSE_CORE__NODE_INTERFACES__NODE_INTERFACES_HELPERS_HPP_
