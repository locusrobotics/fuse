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

#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/node_interfaces/node_time_source_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_waitables_interface.hpp>


#define ALL_FUSE_CORE_NODE_INTERFACES \
  fuse_core::node_interfaces::Base, \
  fuse_core::node_interfaces::Clock, \
  fuse_core::node_interfaces::Graph, \
  fuse_core::node_interfaces::Logging, \
  fuse_core::node_interfaces::Parameters, \
  fuse_core::node_interfaces::Services, \
  fuse_core::node_interfaces::TimeSource, \
  fuse_core::node_interfaces::Timers, \
  fuse_core::node_interfaces::Topics, \
  fuse_core::node_interfaces::Waitables

namespace fuse_core
{
namespace node_interfaces
{
using Base = rclcpp::node_interfaces::NodeBaseInterface;
using Clock = rclcpp::node_interfaces::NodeClockInterface;
using Graph = rclcpp::node_interfaces::NodeGraphInterface;
using Logging = rclcpp::node_interfaces::NodeLoggingInterface;
using Parameters = rclcpp::node_interfaces::NodeParametersInterface;
using Services = rclcpp::node_interfaces::NodeServicesInterface;
using TimeSource = rclcpp::node_interfaces::NodeTimeSourceInterface;
using Timers = rclcpp::node_interfaces::NodeTimersInterface;
using Topics = rclcpp::node_interfaces::NodeTopicsInterface;
using Waitables = rclcpp::node_interfaces::NodeWaitablesInterface;

template<typename ... InterfacesTs>
using NodeInterfaces = ::rclcpp::node_interfaces::NodeInterfaces<InterfacesTs...>;
}  // namespace node_interfaces
}  // namespace fuse_core

#endif  // FUSE_CORE__NODE_INTERFACES__NODE_INTERFACES_HPP_
