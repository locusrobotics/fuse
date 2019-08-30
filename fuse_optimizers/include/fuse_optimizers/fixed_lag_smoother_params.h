/***************************************************************************
 * Copyright (C) 2019 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#ifndef FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H
#define FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <fuse_core/ceres_options.h>

#include <algorithm>
#include <string>
#include <vector>

namespace fuse_optimizers
{

/**
 * @brief Defines the set of parameters required by the fuse_optimizers::FixedLagSmoother class
 */
struct FixedLagSmootherParams
{
public:
  /**
   * @brief The set of sensors whose transactions will trigger the optimizer thread to start running
   *
   * This is designed to keep the system idle until the origin constraint has been received.
   */
  std::vector<std::string> ignition_sensors;

  /**
   * @brief The duration of the smoothing window in seconds
   */
  ros::Duration lag_duration { 5.0 };

  /**
   * @brief The target duration for optimization cycles
   *
   * If an optimization takes longer than expected, an optimization cycle may be skipped. The optimization period
   * may be specified in either the "optimization_period" parameter in seconds, or in the "optimization_frequency"
   * parameter in Hz.
   */
  ros::Duration optimization_period { 0.1 };

  /**
   * @brief The topic name of the advertised reset service
   */
  std::string reset_service { "~reset" };

  /**
   * @brief The maximum time to wait for motion models to be generated for a received transaction.
   *
   * Transactions are processed sequentially, so no new transactions will be added to the graph while waiting for
   * motion models to be generated. Once the timeout expires, that transaction will be deleted from the queue.
   */
  ros::Duration transaction_timeout { 0.1 };

  /**
   * @brief Ceres Solver::Options object that controls various aspects of the optimizer.
   */
  ceres::Solver::Options solver_options;

  /**
   * @brief Helper function that loads strictly positive integral or floating point values from the parameter server
   *
   * @param[in] node_handle - The node handle used to load the parameter
   * @param[in] parameter_name - The parameter name to load
   * @param[in] default_value - A default value to use if the provided parameter name does not exist
   * @return The loaded (or default) value
   */
  template <typename T,
            typename std::enable_if<std::is_integral<T>::value || std::is_floating_point<T>::value>::type* = nullptr>
  T getPositiveParam(const ros::NodeHandle& node_handle, const std::string& parameter_name, T default_value)
  {
    T value;
    node_handle.param(parameter_name, value, default_value);
    if (value <= 0)
    {
      ROS_WARN_STREAM("The requested " << parameter_name << " is <= 0. Using the default value (" <<
                      default_value << ") instead.");
      value = default_value;
    }
    return value;
  }

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh)
  {
    // Read settings from the parameter server
    nh.getParam("ignition_sensors", ignition_sensors);
    std::sort(ignition_sensors.begin(), ignition_sensors.end());

    auto lag_duration_sec = getPositiveParam(nh, "lag_duration", lag_duration.toSec());
    lag_duration.fromSec(lag_duration_sec);

    if (nh.hasParam("optimization_frequency"))
    {
      auto optimization_frequency = getPositiveParam(nh, "optimization_frequency", 1.0 / optimization_period.toSec());
      optimization_period.fromSec(1.0 / optimization_frequency);
    }
    else
    {
      auto optimization_period_sec = getPositiveParam(nh, "optimization_period", optimization_period.toSec());
      optimization_period.fromSec(optimization_period_sec);
    }

    nh.getParam("reset_service", reset_service);

    auto transaction_timeout_sec = getPositiveParam(nh, "transaction_timeout", transaction_timeout.toSec());
    transaction_timeout.fromSec(transaction_timeout_sec);

    loadSolverOptionsFromROS(ros::NodeHandle(nh, "solver_options"));
  }

private:
  /**
   * @brief Method for loading Ceres Solver::Options parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load Ceres Solver::Options parameters
   */
  void loadSolverOptionsFromROS(const ros::NodeHandle& nh)
  {
    // Minimizer options
    solver_options.minimizer_type = fuse_core::getParam(nh, "minimizer_type", solver_options.minimizer_type);
    solver_options.line_search_direction_type =
        fuse_core::getParam(nh, "line_search_direction_type", solver_options.line_search_direction_type);
    solver_options.line_search_type = fuse_core::getParam(nh, "line_search_type", solver_options.line_search_type);
    solver_options.nonlinear_conjugate_gradient_type =
        fuse_core::getParam(nh, "nonlinear_conjugate_gradient_type", solver_options.nonlinear_conjugate_gradient_type);

    nh.param("max_lbfgs_rank", solver_options.max_lbfgs_rank, solver_options.max_lbfgs_rank);
    nh.param("use_approximate_eigenvalue_bfgs_scaling", solver_options.use_approximate_eigenvalue_bfgs_scaling,
             solver_options.use_approximate_eigenvalue_bfgs_scaling);

    solver_options.line_search_interpolation_type =
        fuse_core::getParam(nh, "line_search_interpolation_type", solver_options.line_search_interpolation_type);
    nh.param("min_line_search_step_size", solver_options.min_line_search_step_size,
             solver_options.min_line_search_step_size);

    // Line search parameters
    nh.param("line_search_sufficient_function_decrease", solver_options.line_search_sufficient_function_decrease,
             solver_options.line_search_sufficient_function_decrease);
    nh.param("max_line_search_step_contraction", solver_options.max_line_search_step_contraction,
             solver_options.max_line_search_step_contraction);
    nh.param("min_line_search_step_contraction", solver_options.min_line_search_step_contraction,
             solver_options.min_line_search_step_contraction);
    nh.param("max_num_line_search_step_size_iterations", solver_options.max_num_line_search_step_size_iterations,
             solver_options.max_num_line_search_step_size_iterations);
    nh.param("max_num_line_search_direction_restarts", solver_options.max_num_line_search_direction_restarts,
             solver_options.max_num_line_search_direction_restarts);
    nh.param("line_search_sufficient_curvature_decrease", solver_options.line_search_sufficient_curvature_decrease,
             solver_options.line_search_sufficient_curvature_decrease);
    nh.param("max_line_search_step_expansion", solver_options.max_line_search_step_expansion,
             solver_options.max_line_search_step_expansion);

    solver_options.trust_region_strategy_type =
        fuse_core::getParam(nh, "trust_region_strategy_type", solver_options.trust_region_strategy_type);
    solver_options.dogleg_type = fuse_core::getParam(nh, "dogleg_type", solver_options.dogleg_type);

    nh.param("use_nonmonotonic_steps", solver_options.use_nonmonotonic_steps, solver_options.use_nonmonotonic_steps);
    nh.param("max_consecutive_nonmonotonic_steps", solver_options.max_consecutive_nonmonotonic_steps,
             solver_options.max_consecutive_nonmonotonic_steps);

    nh.param("max_num_iterations", solver_options.max_num_iterations, solver_options.max_num_iterations);
    nh.param("max_solver_time_in_seconds", solver_options.max_solver_time_in_seconds,
             solver_options.max_solver_time_in_seconds);

    nh.param("num_threads", solver_options.num_threads, solver_options.num_threads);

    nh.param("initial_trust_region_radius", solver_options.initial_trust_region_radius,
             solver_options.initial_trust_region_radius);
    nh.param("max_trust_region_radius", solver_options.max_trust_region_radius, solver_options.max_trust_region_radius);
    nh.param("min_trust_region_radius", solver_options.min_trust_region_radius, solver_options.min_trust_region_radius);

    nh.param("min_relative_decrease", solver_options.min_relative_decrease, solver_options.min_relative_decrease);
    nh.param("min_lm_diagonal", solver_options.min_lm_diagonal, solver_options.min_lm_diagonal);
    nh.param("max_lm_diagonal", solver_options.max_lm_diagonal, solver_options.max_lm_diagonal);
    nh.param("max_num_consecutive_invalid_steps", solver_options.max_num_consecutive_invalid_steps,
             solver_options.max_num_consecutive_invalid_steps);
    nh.param("function_tolerance", solver_options.function_tolerance, solver_options.function_tolerance);
    nh.param("gradient_tolerance", solver_options.gradient_tolerance, solver_options.gradient_tolerance);
    nh.param("parameter_tolerance", solver_options.parameter_tolerance, solver_options.parameter_tolerance);

    solver_options.linear_solver_type =
        fuse_core::getParam(nh, "linear_solver_type", solver_options.linear_solver_type);
    solver_options.preconditioner_type =
        fuse_core::getParam(nh, "preconditioner_type", solver_options.preconditioner_type);
    solver_options.visibility_clustering_type =
        fuse_core::getParam(nh, "visibility_clustering_type", solver_options.visibility_clustering_type);
    solver_options.dense_linear_algebra_library_type =
        fuse_core::getParam(nh, "dense_linear_algebra_library_type", solver_options.dense_linear_algebra_library_type);
    solver_options.sparse_linear_algebra_library_type = fuse_core::getParam(
        nh, "sparse_linear_algebra_library_type", solver_options.sparse_linear_algebra_library_type);

    // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> linear_solver_ordering;

    nh.param("use_explicit_schur_complement", solver_options.use_explicit_schur_complement,
             solver_options.use_explicit_schur_complement);
    nh.param("use_postordering", solver_options.use_postordering, solver_options.use_postordering);
    nh.param("dynamic_sparsity", solver_options.dynamic_sparsity, solver_options.dynamic_sparsity);

#if CERES_VERSION_AT_LEAST(2, 0, 0)
    nh.param("use_mixed_precision_solves", solver_options.use_mixed_precision_solves,
             solver_options.use_mixed_precision_solves);
    nh.param("max_num_refinement_iterations", solver_options.max_num_refinement_iterations,
             solver_options.max_num_refinement_iterations);
#endif

    nh.param("use_inner_iterations", solver_options.use_inner_iterations, solver_options.use_inner_iterations);

    // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> inner_iteration_ordering;

    nh.param("inner_iteration_tolerance", solver_options.inner_iteration_tolerance,
             solver_options.inner_iteration_tolerance);
    nh.param("min_linear_solver_iterations", solver_options.min_linear_solver_iterations,
             solver_options.min_linear_solver_iterations);
    nh.param("max_linear_solver_iterations", solver_options.max_linear_solver_iterations,
             solver_options.max_linear_solver_iterations);
    nh.param("eta", solver_options.eta, solver_options.eta);

    nh.param("jacobi_scaling", solver_options.jacobi_scaling, solver_options.jacobi_scaling);

    // Logging options
    solver_options.logging_type = fuse_core::getParam(nh, "logging_type", solver_options.logging_type);
    nh.param("minimizer_progress_to_stdout", solver_options.minimizer_progress_to_stdout,
             solver_options.minimizer_progress_to_stdout);
    nh.param("trust_region_minimizer_iterations_to_dump", solver_options.trust_region_minimizer_iterations_to_dump,
             solver_options.trust_region_minimizer_iterations_to_dump);
    nh.param("trust_region_problem_dump_directory", solver_options.trust_region_problem_dump_directory,
             solver_options.trust_region_problem_dump_directory);
    solver_options.trust_region_problem_dump_format_type = fuse_core::getParam(
        nh, "trust_region_problem_dump_format_type", solver_options.trust_region_problem_dump_format_type);

    // Finite differences options
    nh.param("check_gradients", solver_options.check_gradients, solver_options.check_gradients);
    nh.param("gradient_check_relative_precision", solver_options.gradient_check_relative_precision,
             solver_options.gradient_check_relative_precision);
    nh.param("gradient_check_numeric_derivative_relative_step_size",
             solver_options.gradient_check_numeric_derivative_relative_step_size,
             solver_options.gradient_check_numeric_derivative_relative_step_size);
    nh.param("update_state_every_iteration", solver_options.update_state_every_iteration,
             solver_options.update_state_every_iteration);

    std::string error;
    if (!solver_options.IsValid(&error))
    {
      throw std::invalid_argument("Invalid solver options in parameter " + nh.getNamespace() + ". Error: " + error);
    }
  }
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_FIXED_LAG_SMOOTHER_PARAMS_H
