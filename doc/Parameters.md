# Parameters

This document lists the parameters used by the various classes in Fuse, and the constraints on those parameters.
In ROS2, parameters are associated with distinct nodes


## fuse_optimizers::Optimizer:
declared in `fuse_optimizers/src/optimizer.cpp`
associated with ros node (default name): `batch_optimizer_node` and `fixed_lag_smoother_node`
stored in: `fuse_optimizers::Optimizer` as a std::unordered_map of pluginlib objects

`motion_models`
type: XmlRpc::XmlRpcValue::TypeArray
constraints: array elements are of the form: `{name: string, type: string}` type must match a derived class of `fuse_core::MotionModel` as declared by `PLUGINLIB_EXPORT_CLASS`
default: empty
description: the motion models to load

`sensor_models`
type: XmlRpc::XmlRpcValue::TypeArray
constraints: array elements are of the form: `{name: string, type: string, ignition: bool, motion_models: [name1, name2, ...]}` type must match a derived class of `fuse_core::SensorModel` as declared by `PLUGINLIB_EXPORT_CLASS`, the motion_models must be listed in the `motion_models` parameter above
default: empty
description: the sensor models to load, and the motion models to associate with each sensor model

`publishers`
type: XmlRpc::XmlRpcValue::TypeArray
constraints: array elements are of the form: `{name: string, type: string}` type must match a derived class of `fuse_core::Publisher` as declared by `PLUGINLIB_EXPORT_CLASS`
default: empty
description: the publishers to load


## fuse_optimizers::FixedLagSmoother
declared in file: `fuse_optimizers/include/fixed_lag_smoother_params.h`
associated with ros node (default name): `fixed_lag_smoother_node`
stored in: `FixedLagSmootherParams`

`lag_duration`
type: double
constraint: positive
default: 5.0
description: The duration of the smoothing window in seconds

`optimization_period`
type: double
constraint: positive
default: 0.1
description: The target duration for optimization cycles

`optimization_frequency`
type: double
constraint: positive
default: 10.0
description: The target frequency for optimization cycles (not stored directly, see `optimization_period`)

`transaction_timeout`
type: double
constraint: positive
default: 0.1
description: The maximum time to wait for motion models to be generated for a received transaction.


## fuse_optimizers::BatchOptimizer
declared in file: `fuse_optimizers/include/batch_optimizer_params.h`
associated with ros node (default name): `batch_optimizer_node`
stored in: `fuse_optimizers::BatchOptimizerParams`

`optimization_period`
type: double
constraint: positive
default: 0.1
description: The target duration for optimization cycles

`optimization_frequency`
type: double
constraint: positive
default: 10.0
description: The target frequency for optimization cycles (not stored directly, see `optimization_period`)

`transaction_timeout`
type: double
constraint: positive
default: 0.1
description: The maximum time to wait for motion models to be generated for a received transaction.


## ceres options
declared in file: `fuse_core::/src/ceres_options.cpp`
stored in `fuse_optimizers::BatchOptimizerParams.solver_options` and `fuse_optimizers::FixedLagSmootherParams.solver_options`




loadCovarianceOptionsFromROS `ceres::Covariance::Options` covariance_options

  `sparse_linear_algebra_library_type`
  type: string
  default: `SUITE_SPARSE`
  constraint: `EIGEN_SPARSE`, `SUITE_SPARSE`
  description: 
  stored in: covariance_options.sparse_linear_algebra_library_type

  `algorithm_type`
  type: string
  default: `SPARSE_QR`
  constraint: `SPARSE_QR`, `DENSE_SVD`
  description: 
  stored in: covariance_options.algorithm_type
  
  `min_reciprocal_condition_number`
  type: double (docs say int)
  default: 10^-14
  constraint: 
  description: 
  stored in: covariance_options.min_reciprocal_condition_number
  
  `null_space_rank`
  type: int
  default: 
  constraint: -1 to intmax
  description: the number of singular dimensions to tolerate (-1 unbounded) no effect on `SPARSE_QR`
  stored in: covariance_options.null_space_rank
  
  `num_threads`
  type: int
  default: 1
  constraint: positive non-zero
  description: Number of threads to be used for evaluating the Jacobian and estimation of covariance.
  stored in: covariance_options.num_threads
  
  `apply_loss_function`
  type: bool
  default: true
  constraint: 
  description: Even though the residual blocks in the problem may contain loss functions, setting apply_loss_function to false will turn off the application of the loss function to the output of the cost function and in turn its effect on the covariance.
  stored in: covariance_options.apply_loss_function
   
   loadProblemOptionsFromROS `ceres::Problem::Options` problem_options
  
  `enable_fast_removal`
  type: bool
  default: false
  constraint: 
  description: If true, trades memory for faster Problem::RemoveResidualBlock()
  stored in: problem_options.enable_fast_removal

  `disable_all_safety_checks`
  type: bool
  default: false
  constraint: 
  description: disable safety checks for up to 5% performance
  stored in: problem_options.disable_all_safety_checks


loadSolverOptionsFromROS `ceres::Solver::Options` solver_options
  // Minimizer options

  `minimizer_type`
  type: string
  default: `TRUST_REGION`
  constraint: `LINE_SEARCH`, `TRUST_REGION`
  description: 
  stored in: solver_options.minimizer_type

  `line_search_direction_type`
  type: string
  default: `LBFGS`
  constraint: `STEEPEST_DESCENT`, `NONLINEAR_CONJUGATE_GRADIENT`, `BFGS`, `LBFGS`
  description: 
  stored in: solver_options.line_search_direction_type

  `line_search_type`
  type: string
  default: `WOLFE`
  constraint: `WOLFE`, `ARMIJO`
  description: 
  stored in: solver_options.line_search_type

  `nonlinear_conjugate_gradient_type`
  type: string
  default: `FLETCHER_REEVES`
  constraint: `FLETCHER_REEVES`, `POLAK_RIBIERE`, `HESTENES_STIEFEL`
  description: 
  stored in: solver_options.nonlinear_conjugate_gradient_type

  `max_lbfgs_rank`
  type: int
  default: 20
  constraint:  
  description: 
  stored in: solver_options.max_lbfgs_rank

  `use_approximate_eigenvalue_bfgs_scaling`
  type: bool
  default: false
  constraint: 
  description: 
  stored in: solver_options.use_approximate_eigenvalue_bfgs_scaling


  `line_search_interpolation_type`
  type: string
  default: `CUBIC`
  constraint: `BISECTION`, `QUADRATIC` and `CUBIC`.
  description: Degree of the polynomial used to approximate the objective function. 
  stored in: solver_options.line_search_interpolation_type

  `min_line_search_step_size`
  type: double
  default: 
  constraint:
  description: 
  stored in: solver_options.min_line_search_step_size

  // Line search parameters

  `line_search_sufficient_function_decrease`
  type: double
  default: `1e-4`
  constraint: 
  description: 
  stored in: solver_options.line_search_sufficient_function_decrease

  `max_line_search_step_contraction`
  type: double
  default: `1e-3`
  constraint: 
  description: 
  stored in: solver_options.max_line_search_step_contraction

  `min_line_search_step_contraction`
  type: double
  default: 0.6
  constraint: 
  description: 
  stored in: solver_options.min_line_search_step_contraction

  `max_num_line_search_step_size_iterations`
  type: int
  default: 20
  constraint: 
  description: 
  stored in: solver_options.max_num_line_search_step_size_iterations

  `max_num_line_search_direction_restarts`
  type: int
  default: 5
  constraint: 
  description: 
  stored in: solver_options.max_num_line_search_direction_restarts

  `line_search_sufficient_curvature_decrease`
  type: double
  default: 0.9
  constraint: 
  description: 
  stored in: solver_options.line_search_sufficient_curvature_decrease

  `max_line_search_step_expansion`
  type: double
  default: 10.0
  constraint: 
  description: 
  stored in: solver_options.max_line_search_step_expansion

  `trust_region_strategy_type`
  type: 
  default: `LEVENBERG_MARQUARDT`
  constraint: `DOGLEG`, `LEVENBERG_MARQUARDT` 
  description: 
  stored in: solver_options.trust_region_strategy_type

  `dogleg_type`
  type: 
  default: 
  constraint: 
  description: 
  stored in: solver_options.dogleg_type


  `use_nonmonotonic_steps`
  type: 
  default: `TRADITIONAL_DOGLEG`
  constraint: `TRADITIONAL_DOGLEG`, `SUBSPACE_DOGLEG`
  description: 
  stored in: solver_options.use_nonmonotonic_steps

  `max_consecutive_nonmonotonic_steps`
  type: int
  default: 5
  constraint: 
  description: The window size used by the step selection algorithm to accept non-monotonic steps.
  stored in: solver_options.max_consecutive_nonmonotonic_steps

  `max_num_iterations`
  type: int
  default: 50
  constraint: 
  description: Maximum number of iterations for which the solver should run.
  stored in: solver_options.max_num_iterations

  `max_solver_time_in_seconds`
  type: double
  default: `1e6`
  constraint:
  description: Maximum amount of time for which the solver should run.
  stored in: solver_options.max_solver_time_in_seconds


  `num_threads`
  type: int
  default: 1
  constraint: 
  description: Number of threads used by Ceres to evaluate the Jacobian.
  stored in: solver_options.num_threads


  `initial_trust_region_radius`
  type: double
  default: `1e4`
  constraint: 
  description: The size of the initial trust region. When the LEVENBERG_MARQUARDT strategy is used, the reciprocal of this number is the initial regularization parameter.
  stored in: solver_options.initial_trust_region_radius

  `max_trust_region_radius`
  type: double
  default: `1e16`
  constraint: 
  description: The trust region radius is not allowed to grow beyond this value.
  stored in: solver_options.max_trust_region_radius

  `min_trust_region_radius`
  type: double
  default: 1e-32
  constraint: 
  description: The solver terminates when the trust region becomes smaller than this value.
  stored in: solver_options.min_trust_region_radius


  `min_relative_decrease`
  type: double
  default: `1e3`
  constraint: 
  description: Lower threshold for relative decrease before a trust-region step is accepted.
  stored in: solver_options.min_relative_decrease

  `min_lm_diagonal`
  type: double
  default: 1e-6
  constraint: 
  description: The LEVENBERG_MARQUARDT strategy, uses a diagonal matrix to regularize the trust region step. This is the lower bound on the values of this diagonal matrix.
  stored in: solver_options.min_lm_diagonal

  `max_lm_diagonal`
  type: double
  default: 1e32
  constraint: 
  description: The LEVENBERG_MARQUARDT strategy, uses a diagonal matrix to regularize the trust region step. This is the upper bound on the values of this diagonal matrix.
  stored in: solver_options.max_lm_diagonal

  `max_num_consecutive_invalid_steps`
  type: int
  default: 5
  constraint: 
  description: The step returned by a trust region strategy can sometimes be numerically invalid, usually because of conditioning issues. Instead of crashing or stopping the optimization, the optimizer can go ahead and try solving with a smaller trust region/better conditioned problem. This parameter sets the number of consecutive retries before the minimizer gives u
  stored in: solver_options.max_num_consecutive_invalid_steps

  `function_tolerance`
  type: double
  default: 1e-6
  constraint: 
  description: 
  stored in: solver_options.function_tolerance

  `gradient_tolerance`
  type: double
  default: 1e-10
  constraint: 
  description: 
  stored in: solver_options.gradient_tolerance

  `parameter_tolerance`
  type: double
  default: 1e-8
  constraint: 
  description: 
  stored in: solver_options.parameter_tolerance


  `linear_solver_type`
  type: string
  default: `SPARSE_NORMAL_CHOLESKY`
  constraint: `SPARSE_NORMAL_CHOLESKY`, `DENSE_QR`
  description: Type of linear solver used to compute the solution to the linear least squares problem in each iteration of the Levenberg-Marquardt algorithm.
  stored in: solver_options.linear_solver_type

  `preconditioner_type`
  type: 
  default: `JACOBI`
  constraint: `IDENTITY`, `JACOBI`, `SCHUR_JACOBI`, `CLUSTER_JACOBI`, `CLUSTER_TRIDIAGONAL`
  description: The preconditioner used by the iterative linear solver. 
  stored in: solver_options.preconditioner_type

  `visibility_clustering_type`
  type: string
  default: `CANONICAL_VIEWS`
  constraint: `CANONICAL_VIEWS`, `SINGLE_LINKAGE`
  description: 
  stored in: solver_options.visibility_clustering_type

  `dense_linear_algebra_library_type`
  type: 
  default: `EIGEN`
  constraint: `EIGEN`, `LAPACK`, `CUDA`
  description: Ceres supports using multiple dense linear algebra libraries for dense matrix factorizations.
  stored in: solver_options.dense_linear_algebra_library_type

  `sparse_linear_algebra_library_type`
  type: 
  default: `SUITE_SPARSE`
  constraint: `SUITE_SPARSE`, `CX_SPARSE`, `EIGEN_SPARSE`, `NO_SPARSE`
  description: 
  stored in: solver_options.sparse_linear_algebra_library_type

  // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> linear_solver_ordering;


  `use_explicit_schur_complement`
  type: bool
  default: false
  constraint: 
  description: 
  stored in: solver_options.use_explicit_schur_complement

  `use_postordering`
  type: bool
  default: false
  constraint: 
  description: 
  stored in: solver_options.use_postordering

  `dynamic_sparsity`
  type: bool
  default: false
  constraint: 
  description: 
  stored in: solver_options.dynamic_sparsity



  `use_mixed_precision_solves`
  type: bool
  default: false
  constraint: 
  description: 
  stored in: solver_options.use_mixed_precision_solves

  `max_num_refinement_iterations`
  type: bool
  default: false
  constraint: 
  description: 
  stored in: solver_options.max_num_refinement_iterations



  `use_inner_iterations`
  type: bool
  default: false
  constraint: 
  description: 
  stored in: solver_options.use_inner_iterations

  // No parameter is loaded for: std::shared_ptr<ParameterBlockOrdering> inner_iteration_ordering;


  `inner_iteration_tolerance`
  type: double
  default: 1e-3
  constraint: 
  description: 
  stored in: solver_options.inner_iteration_tolerance

  `min_linear_solver_iterations`
  type: int
  default: 0
  constraint: 
  description: Minimum number of iterations used by the linear iterative solver.
  stored in: solver_options.min_linear_solver_iterations

  `max_linear_solver_iterations`
  type: int
  default: 500
  constraint: 
  description: Minimum number of iterations used by the linear iterative solver.
  stored in: solver_options.max_linear_solver_iterations

  `eta`
  type: double
  default: 0.1
  constraint: 
  description: Forcing sequence parameter. The truncated Newton solver uses this number to control the relative accuracy with which the Newton step is computed.
  stored in: solver_options.eta


  `jacobi_scaling`
  type: bool
  default: true
  constraint: 
  description: true means that the Jacobian is scaled by the norm of its columns before being passed to the linear solver. This improves the numerical conditioning of the normal equations.
  stored in: solver_options.jacobi_scaling

  // Logging options

  `logging_type`
  type: string
  default: `PER_MINIMIZER_ITERATION`
  constraint: (undocumented)
  description: (undocumented)
  stored in: solver_options.logging_type

  `minimizer_progress_to_stdout`
  type: bool
  default: false
  constraint: 
  description: normally sent to stderr
  stored in: solver_options.minimizer_progress_to_stdout



  `trust_region_minimizer_iterations_to_dump`,
  type: rclcpp::PARAMETER_INTEGER_ARRAY
  default: empty
  constraint: 
  description: List of iterations at which the trust region minimizer should dump the trust region problem. Useful for testing and benchmarking. If empty, no problems are dumped.
  stored in: solver_options.trust_region_minimizer_iterations_to_dump


  `trust_region_problem_dump_directory`
  type: string
  default: `/tmp`
  constraint: 
  description: 
  stored in: solver_options.trust_region_problem_dump_directory

  `trust_region_problem_dump_format_type`
  type: 
  default: TEXTFILE
  constraint: TEXTFILE, CONSOLE
  description: Directory to which the problems should be written to. Should be non-empty if `Solver::Options::trust_region_minimizer_iterations_to_dump` is non-empty and `Solver::Options::trust_region_problem_dump_format_type` is not `CONSOLE`.
  stored in: solver_options.trust_region_problem_dump_format_type

  // Finite differences options

  `check_gradients`
  type: bool
  default: false
  constraint: 
  description: Check all Jacobians computed by each residual block with finite differences. This is expensive since it involves computing the derivative by normal means (e.g. user specified, autodiff, etc), then also computing it using finite differences. The results are compared, and if they differ substantially, the optimization fails and the details are stored in the solver summary.
  stored in: solver_options.check_gradients

  `gradient_check_relative_precision`
  type: double
  default: 1e-8
  constraint: 
  description: Precision to check for in the gradient checker. If the relative difference between an element in a Jacobian exceeds this number, then the Jacobian for that cost term is dumped.
  stored in: solver_options.gradient_check_relative_precision

  `gradient_check_numeric_derivative_relative_step_size`
  type: double
  default: 1e-6
  constraint: 
  description: 
  stored in: solver_options.gradient_check_numeric_derivative_relative_step_size

  `update_state_every_iteration`
  type: bool
  default: false
  constraint: 
  description: If update_state_every_iteration is true, then Ceres Solver will guarantee that at the end of every iteration and before any user IterationCallback is called, the parameter blocks are updated to the current best solution found by the solver. Thus the IterationCallback can inspect the values of the parameter blocks for purposes of computation, visualization or termination.
  stored in: solver_options.update_state_every_iteration


