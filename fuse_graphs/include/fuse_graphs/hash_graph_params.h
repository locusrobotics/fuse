/***************************************************************************
 * Copyright (C) 2019 Clearpath Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#ifndef FUSE_GRAPHS_HASH_GRAPH_PARAMS_H
#define FUSE_GRAPHS_HASH_GRAPH_PARAMS_H

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

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
