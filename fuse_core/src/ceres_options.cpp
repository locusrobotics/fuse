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

void loadCovarianceOptionsFromROS(
  node_interfaces::NodeInterfaces<
    node_interfaces::Base,
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  ceres::Covariance::Options& covariance_options,
  const std::string& namespace_string)
{
  rcl_interfaces::msg::ParameterDescriptor tmp_descr;

  std::string namespace_;
  if (namespace_string.empty()) {
    namespace_ = namespace_string;
  } else {
    namespace_ = namespace_string + ".";
  }

#if CERES_VERSION_AT_LEAST(1, 13, 0)
  // The sparse_linear_algebra_library_type field was added to ceres::Covariance::Options in version 1.13.0, see
  // https://github.com/ceres-solver/ceres-solver/commit/14d8297cf968e421c5db4e3fb0543b3b111155d7
  covariance_options.sparse_linear_algebra_library_type = fuse_core::declareCeresParam(
    interfaces, namespace_ + "sparse_linear_algebra_library_type",
    covariance_options.sparse_linear_algebra_library_type);
#endif
  covariance_options.algorithm_type =
    fuse_core::declareCeresParam(interfaces, namespace_ + "algorithm_type", covariance_options.algorithm_type);

  tmp_descr.description = "";
  covariance_options.min_reciprocal_condition_number = fuse_core::getParam(
    interfaces,
    namespace_ + "min_reciprocal_condition_number", covariance_options.min_reciprocal_condition_number, tmp_descr
  );

  tmp_descr.description =
    "the number of singular dimensions to tolerate (-1 unbounded) no effect on `SPARSE_QR`";
  covariance_options.null_space_rank = fuse_core::getParam(
    interfaces,
    namespace_ + "null_space_rank", covariance_options.null_space_rank, tmp_descr
  );

  tmp_descr.description =
    "Number of threads to be used for evaluating the Jacobian and estimation of covariance";
  covariance_options.num_threads = fuse_core::getParam(
    interfaces,
    namespace_ + "num_threads", covariance_options.num_threads, tmp_descr
  );

  tmp_descr.description = (
    "false will turn off the application of the loss function to the output of the cost function "
    "and in turn its effect on the covariance (does not affect residual blocks with built-in loss "
    "functions)");
  covariance_options.apply_loss_function = fuse_core::getParam(
    interfaces,
    namespace_ + "apply_loss_function", covariance_options.apply_loss_function, tmp_descr
  );
}

void loadProblemOptionsFromROS(
  node_interfaces::NodeInterfaces<node_interfaces::Parameters> interfaces,
  ceres::Problem::Options& problem_options,
  const std::string& namespace_string)
{
  rcl_interfaces::msg::ParameterDescriptor tmp_descr;

  std::string namespace_;
  if (namespace_string.empty()) {
    namespace_ = namespace_string;
  } else {
    namespace_ = namespace_string + ".";
  }

  tmp_descr.description = "trades memory for faster Problem::RemoveResidualBlock()";
  problem_options.enable_fast_removal = fuse_core::getParam(
    interfaces,
    namespace_ + "enable_fast_removal", problem_options.enable_fast_removal, tmp_descr
  );

  tmp_descr.description = "If true, trades memory for faster Problem::RemoveResidualBlock()";
  problem_options.disable_all_safety_checks = fuse_core::getParam(
    interfaces,
    namespace_ + "disable_all_safety_checks", problem_options.disable_all_safety_checks, tmp_descr
  );
}

void loadSolverOptionsFromROS(
  node_interfaces::NodeInterfaces<
    node_interfaces::Base,
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  ceres::Solver::Options& solver_options,
  const std::string& namespace_string)
{
  rcl_interfaces::msg::ParameterDescriptor tmp_descr;

  std::string namespace_;
  if (namespace_string.empty()) {
    namespace_ = namespace_string;
  } else {
    namespace_ = namespace_string + ".";
  }

  // Minimizer options
  solver_options.minimizer_type =
    fuse_core::declareCeresParam(interfaces, namespace_ + "minimizer_type", solver_options.minimizer_type);
  solver_options.line_search_direction_type = fuse_core::declareCeresParam(
    interfaces, namespace_ + "line_search_direction_type", solver_options.line_search_direction_type);
  solver_options.line_search_type =
    fuse_core::declareCeresParam(interfaces, namespace_ + "line_search_type", solver_options.line_search_type);
  solver_options.nonlinear_conjugate_gradient_type = fuse_core::declareCeresParam(
    interfaces, namespace_ + "nonlinear_conjugate_gradient_type",
    solver_options.nonlinear_conjugate_gradient_type);

  tmp_descr.description = "";
  solver_options.max_lbfgs_rank = fuse_core::getParam(
    interfaces,
    namespace_ + "max_lbfgs_rank",
    solver_options.max_lbfgs_rank,
    tmp_descr
  );

  tmp_descr.description = "";
  solver_options.use_approximate_eigenvalue_bfgs_scaling = fuse_core::getParam(
    interfaces,
    namespace_ + "use_approximate_eigenvalue_bfgs_scaling",
    solver_options.use_approximate_eigenvalue_bfgs_scaling,
    tmp_descr
  );

  solver_options.line_search_interpolation_type = fuse_core::declareCeresParam(
    interfaces, namespace_ + "line_search_interpolation_type", solver_options.line_search_interpolation_type);

  tmp_descr.description = "";
  solver_options.min_line_search_step_size = fuse_core::getParam(
    interfaces,
    namespace_ + "min_line_search_step_size",
    solver_options.min_line_search_step_size,
    tmp_descr
  );

  // Line search parameters
  tmp_descr.description = "";
  solver_options.line_search_sufficient_function_decrease = fuse_core::getParam(
    interfaces,
    namespace_ + "line_search_sufficient_function_decrease",
    solver_options.line_search_sufficient_function_decrease,
    tmp_descr
  );
  tmp_descr.description = "";
  solver_options.max_line_search_step_contraction = fuse_core::getParam(
    interfaces,
    namespace_ + "max_line_search_step_contraction",
    solver_options.max_line_search_step_contraction,
    tmp_descr
  );
  tmp_descr.description = "";
  solver_options.min_line_search_step_contraction = fuse_core::getParam(
    interfaces,
    namespace_ + "min_line_search_step_contraction",
    solver_options.min_line_search_step_contraction,
    tmp_descr
  );
  tmp_descr.description = "";
  solver_options.max_num_line_search_step_size_iterations = fuse_core::getParam(
    interfaces,
    namespace_ + "max_num_line_search_step_size_iterations",
    solver_options.max_num_line_search_step_size_iterations,
    tmp_descr
  );
  tmp_descr.description = "";
  solver_options.max_num_line_search_direction_restarts = fuse_core::getParam(
    interfaces,
    namespace_ + "max_num_line_search_direction_restarts",
    solver_options.max_num_line_search_direction_restarts,
    tmp_descr
  );
  tmp_descr.description = "";
  solver_options.line_search_sufficient_curvature_decrease = fuse_core::getParam(
    interfaces,
    namespace_ + "line_search_sufficient_curvature_decrease",
    solver_options.line_search_sufficient_curvature_decrease,
    tmp_descr
  );
  tmp_descr.description = "";
  solver_options.max_line_search_step_expansion = fuse_core::getParam(
    interfaces,
    namespace_ + "max_line_search_step_expansion",
    solver_options.max_line_search_step_expansion,
    tmp_descr
  );

  solver_options.trust_region_strategy_type = fuse_core::declareCeresParam(
    interfaces, namespace_ + "trust_region_strategy_type", solver_options.trust_region_strategy_type);
  solver_options.dogleg_type = fuse_core::declareCeresParam(
    interfaces, namespace_ + "dogleg_type", solver_options.dogleg_type);


  tmp_descr.description = "";
  solver_options.use_nonmonotonic_steps = fuse_core::getParam(
    interfaces,
    namespace_ + "use_nonmonotonic_steps",
    solver_options.use_nonmonotonic_steps,
    tmp_descr
  );

  tmp_descr.description = "The window size used by the step selection algorithm to accept non-monotonic steps";
  solver_options.max_consecutive_nonmonotonic_steps = fuse_core::getParam(
    interfaces,
    namespace_ + "max_consecutive_nonmonotonic_steps",
    solver_options.max_consecutive_nonmonotonic_steps,
    tmp_descr
  );


  tmp_descr.description = "Maximum number of iterations for which the solver should run";
  solver_options.max_num_iterations = fuse_core::getParam(
    interfaces,
    namespace_ + "max_num_iterations",
    solver_options.max_num_iterations,
    tmp_descr
  );

  tmp_descr.description = "Maximum amount of time for which the solver should run";
  solver_options.max_solver_time_in_seconds = fuse_core::getParam(
    interfaces,
    namespace_ + "max_solver_time_in_seconds",
    solver_options.max_solver_time_in_seconds,
    tmp_descr
  );

  tmp_descr.description = "Maximum number of iterations for which the solver should run";
  solver_options.num_threads = fuse_core::getParam(
    interfaces,
    namespace_ + "num_threads",
    solver_options.num_threads,
    tmp_descr
  );

  tmp_descr.description = (
    "The size of the initial trust region. When the LEVENBERG_MARQUARDT strategy is used, the "
    "reciprocal of this number is the initial regularization parameter");
  solver_options.initial_trust_region_radius = fuse_core::getParam(
    interfaces,
    namespace_ + "initial_trust_region_radius",
    solver_options.initial_trust_region_radius,
    tmp_descr
  );

  tmp_descr.description = "The trust region radius is not allowed to grow beyond this value";
  solver_options.max_trust_region_radius = fuse_core::getParam(
    interfaces,
    namespace_ + "max_trust_region_radius",
    solver_options.max_trust_region_radius,
    tmp_descr
  );

  tmp_descr.description =
    "The solver terminates when the trust region becomes smaller than this value";
  solver_options.min_trust_region_radius = fuse_core::getParam(
    interfaces,
    namespace_ + "min_trust_region_radius",
    solver_options.min_trust_region_radius,
    tmp_descr
  );

  tmp_descr.description =
    "Lower threshold for relative decrease before a trust-region step is accepted";
  solver_options.min_relative_decrease = fuse_core::getParam(
    interfaces,
    namespace_ + "min_relative_decrease",
    solver_options.min_relative_decrease,
    tmp_descr
  );

  tmp_descr.description = (
    "The LEVENBERG_MARQUARDT strategy, uses a diagonal matrix to regularize the trust region step. "
    "This is the lower bound on the values of this diagonal matrix");
  solver_options.min_lm_diagonal = fuse_core::getParam(
    interfaces,
    namespace_ + "min_lm_diagonal",
    solver_options.min_lm_diagonal,
    tmp_descr
  );

  tmp_descr.description = (
    "The LEVENBERG_MARQUARDT strategy, uses a diagonal matrix to regularize the trust region step. "
    "This is the upper bound on the values of this diagonal matrix");
  solver_options.max_lm_diagonal = fuse_core::getParam(
    interfaces,
    namespace_ + "max_lm_diagonal",
    solver_options.max_lm_diagonal,
    tmp_descr
  );

  tmp_descr.description = (
    "The step returned by a trust region strategy can sometimes be numerically invalid, usually "
    "because of conditioning issues. Instead of crashing or stopping the optimization, the "
    "optimizer can go ahead and try solving with a smaller trust region/better conditioned problem."
    " This parameter sets the number of consecutive retries before the minimizer gives up");
  solver_options.max_num_consecutive_invalid_steps = fuse_core::getParam(
    interfaces,
    namespace_ + "max_num_consecutive_invalid_steps",
    solver_options.max_num_consecutive_invalid_steps,
    tmp_descr
  );

  tmp_descr.description = "";
  solver_options.function_tolerance = fuse_core::getParam(
    interfaces,
    namespace_ + "function_tolerance",
    solver_options.function_tolerance,
    tmp_descr
  );

  tmp_descr.description = "";
  solver_options.gradient_tolerance = fuse_core::getParam(
    interfaces,
    namespace_ + "gradient_tolerance",
    solver_options.gradient_tolerance,
    tmp_descr
  );

  tmp_descr.description = "";
  solver_options.parameter_tolerance = fuse_core::getParam(
    interfaces,
    namespace_ + "parameter_tolerance",
    solver_options.parameter_tolerance,
    tmp_descr
  );

  solver_options.linear_solver_type =
      fuse_core::declareCeresParam(interfaces, namespace_ + "linear_solver_type", solver_options.linear_solver_type);
  solver_options.preconditioner_type =
      fuse_core::declareCeresParam(interfaces, namespace_ + "preconditioner_type", solver_options.preconditioner_type);
  solver_options.visibility_clustering_type =
      fuse_core::declareCeresParam(interfaces, namespace_ + "visibility_clustering_type", solver_options.visibility_clustering_type);
  solver_options.dense_linear_algebra_library_type =
      fuse_core::declareCeresParam(interfaces, namespace_ + "dense_linear_algebra_library_type", solver_options.dense_linear_algebra_library_type);
  solver_options.sparse_linear_algebra_library_type = fuse_core::declareCeresParam(
      interfaces, namespace_ + "sparse_linear_algebra_library_type", solver_options.sparse_linear_algebra_library_type);

  // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> linear_solver_ordering;


  tmp_descr.description = "";
  solver_options.use_explicit_schur_complement = fuse_core::getParam(
    interfaces,
    namespace_ + "use_explicit_schur_complement",
    solver_options.use_explicit_schur_complement,
    tmp_descr
  );

  tmp_descr.description = "";
  solver_options.use_postordering = fuse_core::getParam(
    interfaces,
    namespace_ + "use_postordering",
    solver_options.use_postordering,
    tmp_descr
  );

  tmp_descr.description = "";
  solver_options.dynamic_sparsity = fuse_core::getParam(
    interfaces,
    namespace_ + "dynamic_sparsity",
    solver_options.dynamic_sparsity,
    tmp_descr
  );

#if CERES_VERSION_AT_LEAST(2, 0, 0)

  tmp_descr.description = "";
  solver_options.use_mixed_precision_solves = fuse_core::getParam(
    interfaces,
    namespace_ + "use_mixed_precision_solves",
    solver_options.use_mixed_precision_solves,
    tmp_descr
  );

  tmp_descr.description = "";
  solver_options.max_num_refinement_iterations = fuse_core::getParam(
    interfaces,
    namespace_ + "max_num_refinement_iterations",
    solver_options.max_num_refinement_iterations,
    tmp_descr
  );
#endif


  tmp_descr.description = "";
  solver_options.use_inner_iterations = fuse_core::getParam(
    interfaces,
    namespace_ + "use_inner_iterations",
    solver_options.use_inner_iterations,
    tmp_descr
  );

  // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> inner_iteration_ordering;


  tmp_descr.description = "";
  solver_options.inner_iteration_tolerance = fuse_core::getParam(
    interfaces,
    namespace_ + "inner_iteration_tolerance",
    solver_options.inner_iteration_tolerance,
    tmp_descr
  );

  tmp_descr.description = "Minimum number of iterations used by the linear iterative solver";
  solver_options.min_linear_solver_iterations = fuse_core::getParam(
    interfaces,
    namespace_ + "min_linear_solver_iterations",
    solver_options.min_linear_solver_iterations,
    tmp_descr
  );

  tmp_descr.description = "Maximum number of iterations used by the linear iterative solver";
  solver_options.max_linear_solver_iterations = fuse_core::getParam(
    interfaces,
    namespace_ + "max_linear_solver_iterations",
    solver_options.max_linear_solver_iterations,
    tmp_descr
  );

  tmp_descr.description = "Forcing sequence parameter. The truncated Newton solver uses this number to control the relative accuracy with which the Newton step is computed";
  solver_options.eta = fuse_core::getParam(
    interfaces,
    namespace_ + "eta",
    solver_options.eta,
    tmp_descr
  );


  tmp_descr.description = "true means that the Jacobian is scaled by the norm of its columns before being passed to the linear solver. This improves the numerical conditioning of the normal equations";
  solver_options.jacobi_scaling = fuse_core::getParam(
    interfaces,
    namespace_ + "jacobi_scaling",
    solver_options.jacobi_scaling,
    tmp_descr
  );

  // Logging options
  solver_options.logging_type = fuse_core::declareCeresParam(interfaces, namespace_ + "logging_type", solver_options.logging_type);

  tmp_descr.description = "";
  solver_options.minimizer_progress_to_stdout = fuse_core::getParam(
    interfaces,
    namespace_ + "minimizer_progress_to_stdout",
    solver_options.minimizer_progress_to_stdout,
    tmp_descr
  );
  fuse_core::getParam<std::vector<int64_t>>(
    interfaces, namespace_ + "trust_region_minimizer_iterations_to_dump");
  std::vector<int64_t> iterations_to_dump_tmp = interfaces.get_node_parameters_interface()
    ->get_parameter("trust_region_minimizer_iterations_to_dump")
    .get_value<std::vector<int64_t>>();
  if (!iterations_to_dump_tmp.empty()) {
    solver_options.trust_region_minimizer_iterations_to_dump.reserve(iterations_to_dump_tmp.size());
    std::transform(
      iterations_to_dump_tmp.begin(),
      iterations_to_dump_tmp.end(),
      std::back_inserter(solver_options.trust_region_minimizer_iterations_to_dump),
      [](int64_t val){ return val; });
  }

  tmp_descr.description = "";
  solver_options.trust_region_problem_dump_directory = fuse_core::getParam(
    interfaces,
    namespace_ + "trust_region_problem_dump_directory",
    solver_options.trust_region_problem_dump_directory,
    tmp_descr
  );
  solver_options.trust_region_problem_dump_format_type = fuse_core::declareCeresParam(
      interfaces, namespace_ + "trust_region_problem_dump_format_type", solver_options.trust_region_problem_dump_format_type);

  // Finite differences options
  tmp_descr.description = "Check all Jacobians computed by each residual block with finite differences, abort if numeric and analytic gradients differ substantially)";
  solver_options.check_gradients = fuse_core::getParam(
    interfaces,
    namespace_ + "check_gradients",
    solver_options.check_gradients,
    tmp_descr
  );
  tmp_descr.description = "Precision to check for in the gradient checker. If the relative difference between an element in a Jacobian exceeds this number, then the Jacobian for that cost term is dumped";
  solver_options.gradient_check_relative_precision = fuse_core::getParam(
    interfaces,
    namespace_ + "gradient_check_relative_precision",
    solver_options.gradient_check_relative_precision,
    tmp_descr
  );
  tmp_descr.description = "";
  solver_options.gradient_check_numeric_derivative_relative_step_size = fuse_core::getParam(
    interfaces,
    namespace_ + "gradient_check_numeric_derivative_relative_step_size",
    solver_options.gradient_check_numeric_derivative_relative_step_size,
    tmp_descr
  );
  tmp_descr.description = "If update_state_every_iteration is true, then Ceres Solver will guarantee that at the end of every iteration and before any user IterationCallback is called, the parameter blocks are updated to the current best solution found by the solver. Thus the IterationCallback can inspect the values of the parameter blocks for purposes of computation, visualization or termination";
  solver_options.update_state_every_iteration = fuse_core::getParam(
    interfaces,
    namespace_ + "update_state_every_iteration",
    solver_options.update_state_every_iteration,
    tmp_descr
  );

  std::string error;
  if (!solver_options.IsValid(&error))
  {
    throw std::invalid_argument(
      "Invalid solver options in parameter "
      + std::string(interfaces.get_node_base_interface()->get_namespace())
      + ". Error: " + error);
  }
}

}  // namespace fuse_core
