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
#include <fuse_core/ceres_options.h>

#include <rclcpp/node.hpp>

#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include <stdexcept>
#include <string>


namespace fuse_core
{

void loadCovarianceOptionsFromROS(rclcpp::Node& node, ceres::Covariance::Options& covariance_options)
{
#if CERES_VERSION_AT_LEAST(1, 13, 0)
  // The sparse_linear_algebra_library_type field was added to ceres::Covariance::Options in version 1.13.0, see
  // https://github.com/ceres-solver/ceres-solver/commit/14d8297cf968e421c5db4e3fb0543b3b111155d7
  covariance_options.sparse_linear_algebra_library_type = fuse_core::getParam(
      node, "sparse_linear_algebra_library_type", covariance_options.sparse_linear_algebra_library_type);
#endif
  covariance_options.algorithm_type = fuse_core::getParam(node, "algorithm_type", covariance_options.algorithm_type);
  covariance_options.min_reciprocal_condition_number = node.declare_parameter("min_reciprocal_condition_number", covariance_options.min_reciprocal_condition_number);
  covariance_options.null_space_rank = node.declare_parameter("null_space_rank", covariance_options.null_space_rank);
  covariance_options.num_threads = node.declare_parameter("num_threads", covariance_options.num_threads);
  covariance_options.apply_loss_function = node.declare_parameter("apply_loss_function", covariance_options.apply_loss_function);
}

void loadProblemOptionsFromROS(rclcpp::Node& node, ceres::Problem::Options& problem_options)
{
  problem_options.enable_fast_removal = node.declare_parameter("enable_fast_removal", problem_options.enable_fast_removal);
  problem_options.disable_all_safety_checks = node.declare_parameter("disable_all_safety_checks", problem_options.disable_all_safety_checks);
}

void loadSolverOptionsFromROS(rclcpp::Node& node, ceres::Solver::Options& solver_options)
{
  // Minimizer options
  solver_options.minimizer_type = fuse_core::getParam(node, "minimizer_type", solver_options.minimizer_type);
  solver_options.line_search_direction_type =
      fuse_core::getParam(node, "line_search_direction_type", solver_options.line_search_direction_type);
  solver_options.line_search_type = fuse_core::getParam(node, "line_search_type", solver_options.line_search_type);
  solver_options.nonlinear_conjugate_gradient_type =
      fuse_core::getParam(node, "nonlinear_conjugate_gradient_type", solver_options.nonlinear_conjugate_gradient_type);

  solver_options.max_lbfgs_rank = node.declare_parameter("max_lbfgs_rank", solver_options.max_lbfgs_rank);
  solver_options.use_approximate_eigenvalue_bfgs_scaling = node.declare_parameter("use_approximate_eigenvalue_bfgs_scaling", solver_options.use_approximate_eigenvalue_bfgs_scaling);

  solver_options.line_search_interpolation_type =
      fuse_core::getParam(node, "line_search_interpolation_type", solver_options.line_search_interpolation_type);
  solver_options.min_line_search_step_size = node.declare_parameter("min_line_search_step_size", solver_options.min_line_search_step_size);

  // Line search parameters
  solver_options.line_search_sufficient_function_decrease = node.declare_parameter("line_search_sufficient_function_decrease", solver_options.line_search_sufficient_function_decrease);
  solver_options.max_line_search_step_contraction = node.declare_parameter("max_line_search_step_contraction", solver_options.max_line_search_step_contraction);
  solver_options.min_line_search_step_contraction = node.declare_parameter("min_line_search_step_contraction", solver_options.min_line_search_step_contraction);
  solver_options.max_num_line_search_step_size_iterations = node.declare_parameter("max_num_line_search_step_size_iterations", solver_options.max_num_line_search_step_size_iterations);
  solver_options.max_num_line_search_direction_restarts = node.declare_parameter("max_num_line_search_direction_restarts", solver_options.max_num_line_search_direction_restarts);
  solver_options.line_search_sufficient_curvature_decrease = node.declare_parameter("line_search_sufficient_curvature_decrease", solver_options.line_search_sufficient_curvature_decrease);
  solver_options.max_line_search_step_expansion = node.declare_parameter("max_line_search_step_expansion", solver_options.max_line_search_step_expansion);

  solver_options.trust_region_strategy_type =
      fuse_core::getParam(node, "trust_region_strategy_type", solver_options.trust_region_strategy_type);
  solver_options.dogleg_type = fuse_core::getParam(node, "dogleg_type", solver_options.dogleg_type);

  solver_options.use_nonmonotonic_steps = node.declare_parameter("use_nonmonotonic_steps", solver_options.use_nonmonotonic_steps);
  solver_options.max_consecutive_nonmonotonic_steps = node.declare_parameter("max_consecutive_nonmonotonic_steps", solver_options.max_consecutive_nonmonotonic_steps);

  solver_options.max_num_iterations = node.declare_parameter("max_num_iterations", solver_options.max_num_iterations);
  solver_options.max_solver_time_in_seconds = node.declare_parameter("max_solver_time_in_seconds", solver_options.max_solver_time_in_seconds);

  solver_options.num_threads = node.declare_parameter("num_threads", solver_options.num_threads);

  solver_options.initial_trust_region_radius = node.declare_parameter("initial_trust_region_radius", solver_options.initial_trust_region_radius);
  solver_options.max_trust_region_radius = node.declare_parameter("max_trust_region_radius", solver_options.max_trust_region_radius);
  solver_options.min_trust_region_radius = node.declare_parameter("min_trust_region_radius", solver_options.min_trust_region_radius);

  solver_options.min_relative_decrease = node.declare_parameter("min_relative_decrease", solver_options.min_relative_decrease);
  solver_options.min_lm_diagonal = node.declare_parameter("min_lm_diagonal", solver_options.min_lm_diagonal);
  solver_options.max_lm_diagonal = node.declare_parameter("max_lm_diagonal", solver_options.max_lm_diagonal);
  solver_options.max_num_consecutive_invalid_steps = node.declare_parameter("max_num_consecutive_invalid_steps", solver_options.max_num_consecutive_invalid_steps);
  solver_options.function_tolerance = node.declare_parameter("function_tolerance", solver_options.function_tolerance);
  solver_options.gradient_tolerance = node.declare_parameter("gradient_tolerance", solver_options.gradient_tolerance);
  solver_options.parameter_tolerance = node.declare_parameter("parameter_tolerance", solver_options.parameter_tolerance);

  solver_options.linear_solver_type =
      fuse_core::getParam(node, "linear_solver_type", solver_options.linear_solver_type);
  solver_options.preconditioner_type =
      fuse_core::getParam(node, "preconditioner_type", solver_options.preconditioner_type);
  solver_options.visibility_clustering_type =
      fuse_core::getParam(node, "visibility_clustering_type", solver_options.visibility_clustering_type);
  solver_options.dense_linear_algebra_library_type =
      fuse_core::getParam(node, "dense_linear_algebra_library_type", solver_options.dense_linear_algebra_library_type);
  solver_options.sparse_linear_algebra_library_type = fuse_core::getParam(
      node, "sparse_linear_algebra_library_type", solver_options.sparse_linear_algebra_library_type);

  // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> linear_solver_ordering;

  solver_options.use_explicit_schur_complement = node.declare_parameter("use_explicit_schur_complement", solver_options.use_explicit_schur_complement);
  solver_options.use_postordering = node.declare_parameter("use_postordering", solver_options.use_postordering);
  solver_options.dynamic_sparsity = node.declare_parameter("dynamic_sparsity", solver_options.dynamic_sparsity);

#if CERES_VERSION_AT_LEAST(2, 0, 0)
  solver_options.use_mixed_precision_solves = node.declare_parameter("use_mixed_precision_solves", solver_options.use_mixed_precision_solves);
  solver_options.max_num_refinement_iterations = node.declare_parameter("max_num_refinement_iterations", solver_options.max_num_refinement_iterations);
#endif

  solver_options.use_inner_iterations = node.declare_parameter("use_inner_iterations", solver_options.use_inner_iterations);

  // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> inner_iteration_ordering;

  solver_options.inner_iteration_tolerance = node.declare_parameter("inner_iteration_tolerance", solver_options.inner_iteration_tolerance);
  solver_options.min_linear_solver_iterations = node.declare_parameter("min_linear_solver_iterations", solver_options.min_linear_solver_iterations);
  solver_options.max_linear_solver_iterations = node.declare_parameter("max_linear_solver_iterations", solver_options.max_linear_solver_iterations);
  solver_options.eta = node.declare_parameter("eta", solver_options.eta);

  solver_options.jacobi_scaling = node.declare_parameter("jacobi_scaling", solver_options.jacobi_scaling);

  // Logging options
  solver_options.logging_type = fuse_core::getParam(node, "logging_type", solver_options.logging_type);
  solver_options.minimizer_progress_to_stdout = node.declare_parameter("minimizer_progress_to_stdout", solver_options.minimizer_progress_to_stdout);
  node.declare_parameter("trust_region_minimizer_iterations_to_dump", rclcpp::PARAMETER_INTEGER_ARRAY);
  std::vector<int64_t> iterations_to_dump_tmp;
  if (node.get_parameter("trust_region_minimizer_iterations_to_dump", iterations_to_dump_tmp)) {
    solver_options.trust_region_minimizer_iterations_to_dump.reserve(iterations_to_dump_tmp.size());
    std::transform(
      iterations_to_dump_tmp.begin(),
      iterations_to_dump_tmp.end(),
      std::back_inserter(solver_options.trust_region_minimizer_iterations_to_dump),
      [](int64_t val){ return val; });
  }

  solver_options.trust_region_problem_dump_directory = node.declare_parameter("trust_region_problem_dump_directory", solver_options.trust_region_problem_dump_directory);
  solver_options.trust_region_problem_dump_format_type = fuse_core::getParam(
      node, "trust_region_problem_dump_format_type", solver_options.trust_region_problem_dump_format_type);

  // Finite differences options
  solver_options.check_gradients = node.declare_parameter("check_gradients", solver_options.check_gradients);
  solver_options.gradient_check_relative_precision = node.declare_parameter("gradient_check_relative_precision", solver_options.gradient_check_relative_precision);
  solver_options.gradient_check_numeric_derivative_relative_step_size = node.declare_parameter("gradient_check_numeric_derivative_relative_step_size",
           solver_options.gradient_check_numeric_derivative_relative_step_size);
  solver_options.update_state_every_iteration = node.declare_parameter("update_state_every_iteration", solver_options.update_state_every_iteration);

  std::string error;
  if (!solver_options.IsValid(&error))
  {
    throw std::invalid_argument("Invalid solver options in parameter " + std::string(node.get_namespace()) + ". Error: " + error);
  }
}

}  // namespace fuse_core
