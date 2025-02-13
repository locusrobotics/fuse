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
#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include <stdexcept>
#include <string>

#include <fuse_core/ceres_macros.hpp>
#include <fuse_core/ceres_options.hpp>
#include <rclcpp/node.hpp>

// NOTE(CH3): Most of the parameter descriptions here were adapted from the parameter descriptions
//            in the Ceres source code.
//
// https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/solver.h
// https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/covariance.h
// https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/problem.h
// ...

namespace fuse_core
{

void loadCovarianceOptionsFromROS(
  node_interfaces::NodeInterfaces<
    node_interfaces::Base,
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  ceres::Covariance::Options & covariance_options,
  const std::string & ns)
{
  rcl_interfaces::msg::ParameterDescriptor tmp_descr;

  // The sparse_linear_algebra_library_type field was added to ceres::Covariance::Options in version
  // 1.13.0, see https://github.com/ceres-solver/ceres-
  // solver/commit/14d8297cf968e421c5db4e3fb0543b3b111155d7
  covariance_options.sparse_linear_algebra_library_type = fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "sparse_linear_algebra_library_type"),
    covariance_options.sparse_linear_algebra_library_type);
  covariance_options.algorithm_type =
    fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "algorithm_type"),
    covariance_options.algorithm_type);

  tmp_descr.description = (
    "If DENSE_SVD is used, this parameter sets the threshold for determining if a Jacobian matrix "
    "is rank deficient following the condition: "
    "\n"
    "min_sigma / max_sigma < sqrt(min_reciprocal_condition_number)"
    "\n"
    "Where min_sigma and max_sigma are the minimum and maximum singular values of J respectively.");
  covariance_options.min_reciprocal_condition_number = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "min_reciprocal_condition_number"),
    covariance_options.min_reciprocal_condition_number,
    tmp_descr
  );

  tmp_descr.description =
    "The number of singular dimensions to tolerate (-1 unbounded) no effect on `SPARSE_QR`";
  covariance_options.null_space_rank = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "null_space_rank"),
    covariance_options.null_space_rank, tmp_descr
  );

  tmp_descr.description =
    "Number of threads to be used for evaluating the Jacobian and estimation of covariance";
  covariance_options.num_threads = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "num_threads"), covariance_options.num_threads, tmp_descr
  );

  tmp_descr.description = (
    "false will turn off the application of the loss function to the output of the cost function "
    "and in turn its effect on the covariance (does not affect residual blocks with built-in loss "
    "functions)");
  covariance_options.apply_loss_function = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "apply_loss_function"),
    covariance_options.apply_loss_function,
    tmp_descr
  );
}

void loadProblemOptionsFromROS(
  node_interfaces::NodeInterfaces<node_interfaces::Parameters> interfaces,
  ceres::Problem::Options & problem_options,
  const std::string & ns)
{
  rcl_interfaces::msg::ParameterDescriptor tmp_descr;

  tmp_descr.description = "trades memory for faster Problem::RemoveResidualBlock()";
  problem_options.enable_fast_removal = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "enable_fast_removal"),
    problem_options.enable_fast_removal,
    tmp_descr
  );

  tmp_descr.description = (
    "By default, Ceres performs a variety of safety checks when constructing "
    "the problem. There is a small but measurable performance penalty to these "
    "checks, typically around 5% of construction time. If you are sure your "
    "problem construction is correct, and 5% of the problem construction time "
    "is truly an overhead you want to avoid, then you can set "
    "disable_all_safety_checks to true."
    "\n"
    "WARNING: Do not set this to true, unless you are absolutely sure of what "
    "you are doing");
  problem_options.disable_all_safety_checks = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "disable_all_safety_checks"),
    problem_options.disable_all_safety_checks,
    tmp_descr
  );
}

void loadSolverOptionsFromROS(
  node_interfaces::NodeInterfaces<
    node_interfaces::Base,
    node_interfaces::Logging,
    node_interfaces::Parameters
  > interfaces,
  ceres::Solver::Options & solver_options,
  const std::string & ns)
{
  rcl_interfaces::msg::ParameterDescriptor tmp_descr;

  // Minimizer options
  solver_options.minimizer_type = fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "minimizer_type"), solver_options.minimizer_type);
  solver_options.line_search_direction_type = fuse_core::declareCeresParam(
    interfaces,
    fuse_core::joinParameterName(ns, "line_search_direction_type"),
    solver_options.line_search_direction_type
  );
  solver_options.line_search_type = fuse_core::declareCeresParam(
    interfaces,
    fuse_core::joinParameterName(ns, "line_search_type"),
    solver_options.line_search_type
  );
  solver_options.nonlinear_conjugate_gradient_type = fuse_core::declareCeresParam(
    interfaces,
    fuse_core::joinParameterName(ns, "nonlinear_conjugate_gradient_type"),
    solver_options.nonlinear_conjugate_gradient_type
  );

  tmp_descr.description = (
    "The rank of the LBFGS hessian approximation. See: Nocedal, J. (1980). 'Updating Quasi-Newton "
    "Matrices with Limited Storage'. Mathematics of Computation 35 (151): 773-782.");
  solver_options.max_lbfgs_rank = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_lbfgs_rank"),
    solver_options.max_lbfgs_rank,
    tmp_descr
  );

  tmp_descr.description = (
    "As part of the (L)BFGS update step (BFGS) / right-multiply step (L-BFGS), "
    "the initial inverse Hessian approximation is taken to be the Identity. "
    "However, Oren showed that using instead I * \\gamma, where \\gamma is "
    "chosen to approximate an eigenvalue of the true inverse Hessian can "
    "result in improved convergence in a wide variety of cases. Setting "
    "use_approximate_eigenvalue_bfgs_scaling to true enables this scaling.");
  solver_options.use_approximate_eigenvalue_bfgs_scaling = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "use_approximate_eigenvalue_bfgs_scaling"),
    solver_options.use_approximate_eigenvalue_bfgs_scaling,
    tmp_descr
  );

  tmp_descr.description = (
    "Degree of the polynomial used to approximate the objective function. Valid values are "
    "BISECTION, QUADRATIC and CUBIC.");
  solver_options.line_search_interpolation_type = fuse_core::declareCeresParam(
    interfaces,
    fuse_core::joinParameterName(ns, "line_search_interpolation_type"),
    solver_options.line_search_interpolation_type);

  tmp_descr.description =
    "If during the line search, the step_size falls below this value, it is truncated to zero.";
  solver_options.min_line_search_step_size = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "min_line_search_step_size"),
    solver_options.min_line_search_step_size,
    tmp_descr
  );

  // Line search parameters
  tmp_descr.description = (
    "Solving the line search problem exactly is computationally "
    "prohibitive. Fortunately, line search based optimization "
    "algorithms can still guarantee convergence if instead of an "
    "exact solution, the line search algorithm returns a solution "
    "which decreases the value of the objective function "
    "sufficiently. More precisely, we are looking for a step_size: "
    "s.t. "
    "f(step_size) <= f(0) + sufficient_decrease * f'(0) * step_size");
  solver_options.line_search_sufficient_function_decrease = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "line_search_sufficient_function_decrease"),
    solver_options.line_search_sufficient_function_decrease,
    tmp_descr
  );

  tmp_descr.description = (
    "In each iteration of the line search, "
    "new_step_size >= max_line_search_step_contraction * step_size"
    "\n"
    "Note that by definition, for contraction: "
    "0 < max_step_contraction < min_step_contraction < 1");
  solver_options.max_line_search_step_contraction = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_line_search_step_contraction"),
    solver_options.max_line_search_step_contraction,
    tmp_descr
  );

  tmp_descr.description = (
    "In each iteration of the line search, "
    "new_step_size <= min_line_search_step_contraction * step_size"
    "\n"
    "Note that by definition, for contraction: "
    "0 < max_step_contraction < min_step_contraction < 1");
  solver_options.min_line_search_step_contraction = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "min_line_search_step_contraction"),
    solver_options.min_line_search_step_contraction,
    tmp_descr
  );

  tmp_descr.description = (
    "Maximum number of trial step size iterations during each line "
    "search, if a step size satisfying the search conditions cannot "
    "be found within this number of trials, the line search will "
    "terminate. "

    "The minimum allowed value is 0 for trust region minimizer and 1 "
    "otherwise. If 0 is specified for the trust region minimizer, "
    "then line search will not be used when solving constrained "
    "optimization problems. ");
  solver_options.max_num_line_search_step_size_iterations = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_num_line_search_step_size_iterations"),
    solver_options.max_num_line_search_step_size_iterations,
    tmp_descr
  );

  tmp_descr.description = (
    "Maximum number of restarts of the line search direction algorithm before "
    "terminating the optimization. Restarts of the line search direction "
    "algorithm occur when the current algorithm fails to produce a new descent "
    "direction. This typically indicates a numerical failure, or a breakdown "
    "in the validity of the approximations used. ");
  solver_options.max_num_line_search_direction_restarts = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_num_line_search_direction_restarts"),
    solver_options.max_num_line_search_direction_restarts,
    tmp_descr
  );

  tmp_descr.description = (
    "The strong Wolfe conditions consist of the Armijo sufficient "
    "decrease condition, and an additional requirement that the "
    "step-size be chosen s.t. the _magnitude_ ('strong' Wolfe "
    "conditions) of the gradient along the search direction "
    "decreases sufficiently. Precisely, this second condition "
    "is that we seek a step_size s.t. "
    "\n"
    "   |f'(step_size)| <= sufficient_curvature_decrease * |f'(0)|"
    "\n"
    "Where f() is the line search objective and f'() is the derivative of f "
    "w.r.t step_size (d f / d step_size).");
  solver_options.line_search_sufficient_curvature_decrease = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "line_search_sufficient_curvature_decrease"),
    solver_options.line_search_sufficient_curvature_decrease,
    tmp_descr
  );

  tmp_descr.description = (
    "During the bracketing phase of the Wolfe search, the step size is "
    "increased until either a point satisfying the Wolfe conditions is "
    "found, or an upper bound for a bracket containing a point satisfying "
    "the conditions is found. Precisely, at each iteration of the expansion:"
    "\n"
    "   new_step_size <= max_step_expansion * step_size."
    "\n"
    "By definition for expansion, max_step_expansion > 1.0.");
  solver_options.max_line_search_step_expansion = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_line_search_step_expansion"),
    solver_options.max_line_search_step_expansion,
    tmp_descr
  );

  solver_options.trust_region_strategy_type = fuse_core::declareCeresParam(
    interfaces,
    fuse_core::joinParameterName(ns, "trust_region_strategy_type"),
    solver_options.trust_region_strategy_type
  );
  solver_options.dogleg_type = fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "dogleg_type"), solver_options.dogleg_type);

  tmp_descr.description = (
    "Enables the non-monotonic trust region algorithm as described by Conn, "
    "Gould & Toint in 'Trust Region Methods', Section 10.1");
  solver_options.use_nonmonotonic_steps = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "use_nonmonotonic_steps"),
    solver_options.use_nonmonotonic_steps,
    tmp_descr
  );

  tmp_descr.description =
    "The window size used by the step selection algorithm to accept non-monotonic steps";
  solver_options.max_consecutive_nonmonotonic_steps = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_consecutive_nonmonotonic_steps"),
    solver_options.max_consecutive_nonmonotonic_steps,
    tmp_descr
  );

  tmp_descr.description = "Maximum number of iterations for which the solver should run";
  solver_options.max_num_iterations = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_num_iterations"),
    solver_options.max_num_iterations,
    tmp_descr
  );

  tmp_descr.description = "Maximum amount of time for which the solver should run";
  solver_options.max_solver_time_in_seconds = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_solver_time_in_seconds"),
    solver_options.max_solver_time_in_seconds,
    tmp_descr
  );

  tmp_descr.description = "Number of threads used by Ceres for evaluating the cost and jacobians";
  solver_options.num_threads = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "num_threads"),
    solver_options.num_threads,
    tmp_descr
  );

  tmp_descr.description = (
    "The size of the initial trust region. When the LEVENBERG_MARQUARDT strategy is used, the "
    "reciprocal of this number is the initial regularization parameter");
  solver_options.initial_trust_region_radius = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "initial_trust_region_radius"),
    solver_options.initial_trust_region_radius,
    tmp_descr
  );

  tmp_descr.description = "The trust region radius is not allowed to grow beyond this value";
  solver_options.max_trust_region_radius = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_trust_region_radius"),
    solver_options.max_trust_region_radius,
    tmp_descr
  );

  tmp_descr.description =
    "The solver terminates when the trust region becomes smaller than this value";
  solver_options.min_trust_region_radius = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "min_trust_region_radius"),
    solver_options.min_trust_region_radius,
    tmp_descr
  );

  tmp_descr.description =
    "Lower threshold for relative decrease before a trust-region step is accepted";
  solver_options.min_relative_decrease = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "min_relative_decrease"),
    solver_options.min_relative_decrease,
    tmp_descr
  );

  tmp_descr.description = (
    "The LEVENBERG_MARQUARDT strategy, uses a diagonal matrix to regularize the trust region step. "
    "This is the lower bound on the values of this diagonal matrix");
  solver_options.min_lm_diagonal = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "min_lm_diagonal"),
    solver_options.min_lm_diagonal,
    tmp_descr
  );

  tmp_descr.description = (
    "The LEVENBERG_MARQUARDT strategy, uses a diagonal matrix to regularize the trust region step. "
    "This is the upper bound on the values of this diagonal matrix");
  solver_options.max_lm_diagonal = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_lm_diagonal"),
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
    fuse_core::joinParameterName(ns, "max_num_consecutive_invalid_steps"),
    solver_options.max_num_consecutive_invalid_steps,
    tmp_descr
  );

  tmp_descr.description =
    "Minimizer terminates when: (new_cost - old_cost) < function_tolerance * old_cost;";
  solver_options.function_tolerance = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "function_tolerance"),
    solver_options.function_tolerance,
    tmp_descr
  );

  tmp_descr.description = (
    "Minimizer terminates when: max_i |x - Project(Plus(x, -g(x))| < gradient_tolerance"
    "\n"
    "This value should typically be 1e-4 * function_tolerance");
  solver_options.gradient_tolerance = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "gradient_tolerance"),
    solver_options.gradient_tolerance,
    tmp_descr
  );

  tmp_descr.description =
    "Minimizer terminates when: |step|_2 <= parameter_tolerance * ( |x|_2 +  parameter_tolerance)";
  solver_options.parameter_tolerance = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "parameter_tolerance"),
    solver_options.parameter_tolerance,
    tmp_descr
  );

  solver_options.linear_solver_type =
    fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "linear_solver_type"),
    solver_options.linear_solver_type);
  solver_options.preconditioner_type =
    fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "preconditioner_type"),
    solver_options.preconditioner_type);
  solver_options.visibility_clustering_type =
    fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "visibility_clustering_type"),
    solver_options.visibility_clustering_type);
  solver_options.dense_linear_algebra_library_type =
    fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "dense_linear_algebra_library_type"),
    solver_options.dense_linear_algebra_library_type);
  solver_options.sparse_linear_algebra_library_type = fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "sparse_linear_algebra_library_type"),
    solver_options.sparse_linear_algebra_library_type);

  // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> linear_solver_ordering;

  tmp_descr.description =
    "Enabling this option tells ITERATIVE_SCHUR to use an explicitly computed Schur complement.";
  solver_options.use_explicit_schur_complement = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "use_explicit_schur_complement"),
    solver_options.use_explicit_schur_complement,
    tmp_descr
  );

#if !CERES_VERSION_AT_LEAST(2, 2, 0)
  tmp_descr.description = (
    "In some rare cases, it is worth using a more complicated "
    "reordering algorithm which has slightly better runtime "
    "performance at the expense of an extra copy of the Jacobian "
    "matrix. Setting use_postordering to true enables this tradeoff.");
  solver_options.use_postordering = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "use_postordering"),
    solver_options.use_postordering,
    tmp_descr
  );
#endif

  tmp_descr.description = "This settings only affects the SPARSE_NORMAL_CHOLESKY solver.";
  solver_options.dynamic_sparsity = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "dynamic_sparsity"),
    solver_options.dynamic_sparsity,
    tmp_descr
  );

#if CERES_VERSION_AT_LEAST(2, 0, 0)
  tmp_descr.description = (
    "NOTE1: EXPERIMENTAL FEATURE, UNDER DEVELOPMENT, USE AT YOUR OWN RISK. "
    "\n"
    "If use_mixed_precision_solves is true, the Gauss-Newton matrix "
    "is computed in double precision, but its factorization is "
    "computed in single precision. This can result in significant "
    "time and memory savings at the cost of some accuracy in the "
    "Gauss-Newton step. Iterative refinement is used to recover some "
    "of this accuracy back.");
  solver_options.use_mixed_precision_solves = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "use_mixed_precision_solves"),
    solver_options.use_mixed_precision_solves,
    tmp_descr
  );

  tmp_descr.description =
    "Number steps of the iterative refinement process to run when computing the Gauss-Newton step.";
  solver_options.max_num_refinement_iterations = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_num_refinement_iterations"),
    solver_options.max_num_refinement_iterations,
    tmp_descr
  );
#endif

#if CERES_VERSION_AT_LEAST(2, 2, 0)
  tmp_descr.description = (
    "Maximum number of iterations performed by SCHUR_POWER_SERIES_EXPANSION. "
    "Each iteration corresponds to one more term in the power series expansion "
    "of the inverse of the Schur complement.  This value controls the maximum "
    "number of iterations whether it is used as a preconditioner or just to "
    "initialize the solution for ITERATIVE_SCHUR.");
  solver_options.max_num_spse_iterations = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_num_spse_iterations"),
    solver_options.max_num_spse_iterations,
    tmp_descr
  );

  tmp_descr.description = (
    "Use SCHUR_POWER_SERIES_EXPANSION to initialize the solution for "
    "ITERATIVE_SCHUR. This option can be set true regardless of what "
    "preconditioner is being used.");
  solver_options.use_spse_initialization = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "use_spse_initialization"),
    solver_options.use_spse_initialization,
    tmp_descr
  );

  tmp_descr.description = (
    "When use_spse_initialization is true, this parameter along with "
    "max_num_spse_iterations controls the number of "
    "SCHUR_POWER_SERIES_EXPANSION iterations performed for initialization. It "
    "is not used to control the preconditioner.");
  solver_options.spse_tolerance = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "spse_tolerance"),
    solver_options.spse_tolerance,
    tmp_descr
  );
#endif

  tmp_descr.description =
    "Enable the use of the non-linear generalization of Ruhe & Wedin's Algorithm II";
  solver_options.use_inner_iterations = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "use_inner_iterations"),
    solver_options.use_inner_iterations,
    tmp_descr
  );

  // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> inner_iteration_ordering;

  tmp_descr.description = (
    "Once the relative decrease in the objective function due to "
    "inner iterations drops below inner_iteration_tolerance, the use "
    "of inner iterations in subsequent trust region minimizer "
    "iterations is disabled.");
  solver_options.inner_iteration_tolerance = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "inner_iteration_tolerance"),
    solver_options.inner_iteration_tolerance,
    tmp_descr
  );

  tmp_descr.description = "Minimum number of iterations used by the linear iterative solver";
  solver_options.min_linear_solver_iterations = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "min_linear_solver_iterations"),
    solver_options.min_linear_solver_iterations,
    tmp_descr
  );

  tmp_descr.description = "Maximum number of iterations used by the linear iterative solver";
  solver_options.max_linear_solver_iterations = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "max_linear_solver_iterations"),
    solver_options.max_linear_solver_iterations,
    tmp_descr
  );

  tmp_descr.description = (
    "Forcing sequence parameter. The truncated Newton solver uses this number to control the "
    "relative accuracy with which the Newton step is computed");
  solver_options.eta = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "eta"),
    solver_options.eta,
    tmp_descr
  );

  tmp_descr.description = (
    "True means that the Jacobian is scaled by the norm of its columns before being passed to the "
    "linear solver. This improves the numerical conditioning of the normal equations");
  solver_options.jacobi_scaling = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "jacobi_scaling"),
    solver_options.jacobi_scaling,
    tmp_descr
  );

  // Logging options
  solver_options.logging_type = fuse_core::declareCeresParam(
    interfaces, fuse_core::joinParameterName(ns, "logging_type"), solver_options.logging_type);

  tmp_descr.description = "If logging_type is not SILENT, sends the logging output to STDOUT";
  solver_options.minimizer_progress_to_stdout = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "minimizer_progress_to_stdout"),
    solver_options.minimizer_progress_to_stdout,
    tmp_descr
  );
  fuse_core::getParam<std::vector<int64_t>>(
    interfaces,
    fuse_core::joinParameterName(ns, "trust_region_minimizer_iterations_to_dump"),
    std::vector<int64_t>()
  );
  std::vector<int64_t> iterations_to_dump_tmp = interfaces.get_node_parameters_interface()
    ->get_parameter(fuse_core::joinParameterName(ns, "trust_region_minimizer_iterations_to_dump"))
    .get_value<std::vector<int64_t>>();
  if (!iterations_to_dump_tmp.empty()) {
    solver_options.trust_region_minimizer_iterations_to_dump.reserve(iterations_to_dump_tmp.size());
    std::transform(
      iterations_to_dump_tmp.begin(),
      iterations_to_dump_tmp.end(),
      std::back_inserter(solver_options.trust_region_minimizer_iterations_to_dump),
      [](int64_t val) {return val;});
  }

  tmp_descr.description = (
    "Directory to which the problems should be written to. Should be "
    "non-empty if trust_region_minimizer_iterations_to_dump is "
    "non-empty and trust_region_problem_dump_format_type is not "
    "CONSOLE.");
  solver_options.trust_region_problem_dump_directory = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "trust_region_problem_dump_directory"),
    solver_options.trust_region_problem_dump_directory,
    tmp_descr
  );
  solver_options.trust_region_problem_dump_format_type =
    fuse_core::declareCeresParam(
    interfaces,
    fuse_core::joinParameterName(ns, "trust_region_problem_dump_format_type"),
    solver_options.trust_region_problem_dump_format_type
    );

  // Finite differences options
  tmp_descr.description = (
    "Check all Jacobians computed by each residual block with finite differences, abort if numeric "
    "and analytic gradients differ substantially)");
  solver_options.check_gradients = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "check_gradients"),
    solver_options.check_gradients,
    tmp_descr
  );
  tmp_descr.description = (
    "Precision to check for in the gradient checker. If the relative "
    "difference between an element in a Jacobian exceeds this number, then the Jacobian for that "
    "cost term is dumped");
  solver_options.gradient_check_relative_precision = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "gradient_check_relative_precision"),
    solver_options.gradient_check_relative_precision,
    tmp_descr
  );
  tmp_descr.description = (
    "Relative shift used for taking numeric derivatives when "
    "Solver::Options::check_gradients is true.");
  solver_options.gradient_check_numeric_derivative_relative_step_size = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "gradient_check_numeric_derivative_relative_step_size"),
    solver_options.gradient_check_numeric_derivative_relative_step_size,
    tmp_descr
  );
  tmp_descr.description = (
    "If update_state_every_iteration is true, then Ceres Solver will guarantee that at the end of "
    "every iteration and before any user IterationCallback is called, the parameter blocks are "
    "updated to the current best solution found by the solver. Thus the IterationCallback can "
    "inspect the values of the parameter blocks for purposes of computation, visualization or "
    "termination");
  solver_options.update_state_every_iteration = fuse_core::getParam(
    interfaces,
    fuse_core::joinParameterName(ns, "update_state_every_iteration"),
    solver_options.update_state_every_iteration,
    tmp_descr
  );

  std::string error;
  if (!solver_options.IsValid(&error)) {
    throw std::invalid_argument(
            "Invalid solver options in parameter " +
            std::string(interfaces.get_node_base_interface()->get_namespace()) +
            ". Error: " + error);
  }
}  // NOLINT [readability/fn_size]

}  // namespace fuse_core
