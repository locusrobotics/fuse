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
#ifndef FUSE_CORE_PUBLISHER_H
#define FUSE_CORE_PUBLISHER_H

#include <fuse_core/graph.h>
#include <fuse_core/macros.h>
#include <fuse_core/transaction.h>
#include <fuse_core/variable.h>
#include <ros/ros.h>

#include <string>
#include <vector>


namespace fuse_core
{

/**
 * @brief The interface class for publisher plugins in the fuse ecosystem.
 *
 * A publisher plugin is responsible for querying the latest variable value from the optimized Graph, converting the
 * variable values into ROS messages, and publishing the messages on their desired topics. Publisher plugins will be
 * loaded by the optimizer at run/configure time. The optimizer will notify the plugins after the graph has been
 * optimized. This class defines the supported interface between the optimizer and publisher. If you are deriving a
 * new publisher class, consider using fuse_core::AsyncPublisher as the base class instead. It offers additional
 * features making it act similar to a standard node or nodelet.
 */
class Publisher
{
public:
  SMART_PTR_ALIASES_ONLY(Publisher);

  /**
   * @brief Constructor
   *
   * All ROS plugins must provide a default constructor for use with the plugin loader.
   */
  Publisher() = default;

  /**
   * @brief Destructor
   */
  virtual ~Publisher() = default;

  /**
   * @brief Get the unique name of this publisher
   */
  virtual const std::string& name() const = 0;

  /**
   * @brief Perform any required post-construction initialization, such as advertising publishers or reading from the
   * parameter server.
   *
   * This will be called on each plugin after construction, and after the ros node has been initialized. Plugins are
   * encouraged to subnamespace any of their parameters to prevent conflicts and allow the same plugin to be used
   * multiple times with different settings and topics.
   *
   * @param[in] name A unique name to give this plugin instance
   */
  virtual void initialize(const std::string& name) = 0;

  /**
   * @brief Notify the publisher that an optimization cycle is complete, and about changes to the Graph.
   *
   * Most publishers will only publish new data whenever the Graph values have been updated. The Publisher::notify()
   * method will be called by the optimizer (and in the optimizer's thread) after every optimization cycle is
   * complete. Additionally, the set of added and removed variables and constraints since the last optimization cycle
   * will be provided to the publisher. In many cases this should prevent the plugins from searching through the
   * entire graph, looking for the variables of interest.
   *
   * @param[in] transaction A Transaction object, describing the set of variables that have been added and/or removed
   * @param[in] graph       A read-only pointer to the graph object, allowing queries to be performed whenever needed
   */
  virtual void notify(Transaction::ConstSharedPtr transaction, Graph::ConstSharedPtr graph) = 0;
};

}  // namespace fuse_core

#endif  // FUSE_CORE_PUBLISHER_H
