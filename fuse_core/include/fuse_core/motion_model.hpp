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
#ifndef FUSE_CORE__MOTION_MODEL_HPP_
#define FUSE_CORE__MOTION_MODEL_HPP_

#include <string>

#include <fuse_core/fuse_macros.hpp>
#include <fuse_core/graph.hpp>
#include <fuse_core/transaction.hpp>

namespace fuse_core
{

/**
 * @brief The interface definition for motion model plugins in the fuse ecosystem.
 *
 * A model model plugin is responsible for generating constraints that link together timestamps
 * introduced by other sensors in the system.
 */
class MotionModel
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(MotionModel)

  /**
   * @brief Destructor
   */
  virtual ~MotionModel() = default;

  /**
   * @brief Augment a transaction object such that all involved timestamps are connected by motion
   *        model constraints.
   *
   * This function will be called by the optimizer (in the Optimizer's thread) for each
   * received transaction.
   *
   * @param[in,out] transaction The transaction object that should be augmented with motion model
   *                            constraints
   * @return                    True if the motion models were generated successfully, false
   *                            otherwise
   */
  virtual bool apply(Transaction & transaction) = 0;

  /**
   * @brief Function to be executed whenever the optimizer has completed a Graph update
   *
   * This method will be called by the optimizer, in the optimizer's thread, after each Graph
   * update is complete. This generally means that new variables have been inserted into the
   * Graph, and new optimized values are available. To simplify synchronization between the
   * motion models and other consumers of Graph data, the provided Graph object will never be
   * updated by anyone. Thus, only read access to the Graph is provided. Information may be
   * accessed or computed, but it cannot be changed. The optimizer provides the motion models
   * with Graph updates by sending a new Graph object, not by modifying this Graph object.
   *
   * @param[in] graph A read-only pointer to the graph object, allowing queries to be performed
   *                  whenever needed.
   */
  virtual void graphCallback(Graph::ConstSharedPtr /*graph*/) {}

  /**
   * @brief Perform any required post-construction initialization, such as subscribing to topics or
   *        reading from the parameter server.
   *
   * This will be called on each plugin after construction, and after the ros node has been
   * initialized. Plugins are encouraged to subnamespace any of their parameters to prevent
   * conflicts and allow the same plugin to be used multiple times with different settings and
   * topics.
   *
   * @param[in] name A unique name to give this plugin instance
   */
  virtual void initialize(
    node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name) = 0;

  /**
   * @brief Get the unique name of this motion model
   */
  virtual const std::string & name() const = 0;

  /**
   * @brief Function to be executed whenever the optimizer is ready to receive transactions
   *
   * This method will be called by the optimizer, in the optimizer's thread, once the optimizer
   * has been initialized and is ready to receive transactions. It may also be called as part
   * of a stop-start cycle when the optimizer has been requested to reset itself. This allows
   * the motion model to reset any internal state before the optimizer begins processing after
   * a reset. No calls to apply() will happen before the optimizer calls start().
   */
  virtual void start() {}

  /**
   * @brief Function to be executed whenever the optimizer is no longer ready to receive
   *        transactions
   *
   * This method will be called by the optimizer, in the optimizer's thread, before the
   * optimizer shutdowns. It may also be called as part of a stop-start cycle when the
   * optimizer has been requested to reset itself. This allows the motion model to reset any
   * internal state before the optimizer begins processing after a reset. No calls to apply()
   * will happen until start() has been called again.
   */
  virtual void stop() {}

protected:
  /**
   * @brief Default Constructor
   */
  MotionModel() = default;
};

}  // namespace fuse_core

#endif  // FUSE_CORE__MOTION_MODEL_HPP_
