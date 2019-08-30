/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Clearpath Robotics
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
#ifndef FUSE_GRAPHS_HASH_GRAPH_PARAMS_H
#define FUSE_GRAPHS_HASH_GRAPH_PARAMS_H

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <ceres/problem.h>

#include <algorithm>
#include <string>
#include <vector>

namespace fuse_graphs
{

/**
 * @brief Defines the set of parameters required by the fuse_graphs::HashGraph class
 */
struct HashGraphParams
{
public:
  /**
   * @brief Ceres Problem::Options object that controls various aspects of the optimization problem.
   *
   * See https://ceres-solver.googlesource.com/ceres-solver/+/master/include/ceres/problem.h#123
   */
  ceres::Problem::Options problem_options;

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh)
  {
    loadProblemOptionsFromROS(ros::NodeHandle(nh, "problem_options"));
  }

private:
  /**
   * @brief Method for loading Ceres Problem::Options parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load Ceres Problem::Options parameters
   */
  void loadProblemOptionsFromROS(const ros::NodeHandle& nh)
  {
    nh.param("enable_fast_removal", problem_options.enable_fast_removal, problem_options.enable_fast_removal);
    nh.param("disable_all_safety_checks", problem_options.disable_all_safety_checks,
             problem_options.disable_all_safety_checks);
  }
};

}  // namespace fuse_graphs

#endif  // FUSE_GRAPHS_HASH_GRAPH_PARAMS_H
