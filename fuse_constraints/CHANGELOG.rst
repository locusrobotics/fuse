^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-03-03)
------------------

1.0.0 (2023-03-03)
------------------
* Use rclcpp::Clock::wait_until_started (`#303 <https://github.com/locusrobotics/fuse/issues/303>`_)
* fuse -> ROS 2 : Doc Generation (`#278 <https://github.com/locusrobotics/fuse/issues/278>`_)
  * Port doc generation and fix package.xml for linting
  * Fix small bugs in package.xml
  * Use default rosdoc2 settings
  * Use default rosdoc2 settings
  * Update fuse_doc for rosdoc2
  ---------
  Co-authored-by: Shane Loretz <sloretz@google.com>
* fuse -> ROS 2 fuse_publishers: Port fuse_publishers (`#299 <https://github.com/locusrobotics/fuse/issues/299>`_)
  Co-authored-by: Shane Loretz <shane.loretz@gmail.com>
* ROS 2 port of fuse_viz (`#301 <https://github.com/locusrobotics/fuse/issues/301>`_)
  * Initial attempt at porting fuse_viz
  * Make sure suitesparse can be found downstream
  * Plugin library names can't have lib/ prefix
  * Plugin library names can't have lib/ prefix
  * Add lint tests (they currently fail)
* fuse -> ROS 2 fuse_constraints : Linting (`#298 <https://github.com/locusrobotics/fuse/issues/298>`_)
* fuse -> ROS 2 fuse_constraints: Port fuse_constraints (`#290 <https://github.com/locusrobotics/fuse/issues/290>`_)
  Co-authored-by: Shane Loretz <sloretz@osrfoundation.org>
* fuse -> ROS 2 fuse_graphs: Linting (`#297 <https://github.com/locusrobotics/fuse/issues/297>`_)
* fuse -> ROS 2 fuse_variables: Linting (`#296 <https://github.com/locusrobotics/fuse/issues/296>`_)
  * Migrate to .hpp files
  * Create redirection headers
  * Make xmllint and uncrustify happy
  * Wrap most comment lines
  * Satisfy cpplint
* fuse -> ROS 2 fuse_core: Linting (`#292 <https://github.com/locusrobotics/fuse/issues/292>`_)
* fuse -> ROS 2 fuse_core : Parameters and Tests (`#286 <https://github.com/locusrobotics/fuse/issues/286>`_)
  Co-authored-by: Shane Loretz <sloretz@osrfoundation.org>
  Co-authored-by: Ivor Wanders <ivor@iwanders.net>
* fuse -> ROS 2 : Port Time (`#283 <https://github.com/locusrobotics/fuse/issues/283>`_)
* Fuse -> ROS 2 fuse_core: Partial port of fuse_core (`#281 <https://github.com/locusrobotics/fuse/issues/281>`_)
  Co-authored-by: Brett Downing
* fuse -> ROS 2: Clean up macro usage warnings (`#280 <https://github.com/locusrobotics/fuse/issues/280>`_)
* fuse -> ROS 2 fuse_msgs : Port package and ignore unported packages for now (`#277 <https://github.com/locusrobotics/fuse/issues/277>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* [RST-4186] Fix fuse macro names (`#263 <https://github.com/locusrobotics/fuse/issues/263>`_)
  * Namespace all macros with the FUSE\_ prefix. Mark original macros as deprecated.
  * Update all fuse objects to use the new macro names
* [RST-4390] Allow variables to be held constant during optimization (`#243 <https://github.com/locusrobotics/fuse/issues/243>`_)
  * Add support for holding variables constant
  * Create a 'fixed' landmark
  * Added initial support for marginalizing constant variables
* Adding doxygen to all packages (`#241 <https://github.com/locusrobotics/fuse/issues/241>`_)
* Contributors: Shane Loretz, Stephen Williams, Tom Moore, methylDragon

0.4.2 (2021-07-20)
------------------
* Adding roslint dependency to fuse_viz (`#231 <https://github.com/locusrobotics/fuse/issues/231>`_)
  * Adding roslint dependency to fuse_viz
  * Silence CMP0048 warnings
* Contributors: Tom Moore

0.4.1 (2021-07-13)
------------------
* Use analytic relative pose 2d cost function (`#193 <https://github.com/locusrobotics/fuse/issues/193>`_)
  * Use analytic relative pose 2d cost function
  * Add analytic `NormalDeltaPose2D` cost function
  * Use analytic `NormalDeltaPose2D` cost function in
  `RelativePose2DStampedConstraint`
  * Test `NormalDeltaPose2D` jacobians are correct, comparing against
  `NormalDeltaPose2DCostFunctor` using automatic differentiation
  * Benchmark `NormalDeltaPose2D` vs `NormalDeltaPose2DCostFunctor` using
  automatic differentiation for 1, 2 and 3 residuals. The latency
  speedup is approximately 1.35, 1.49 and 1.55, respectively.
* Use analytic absolute pose 2d cost function (`#192 <https://github.com/locusrobotics/fuse/issues/192>`_)
  * Add analytic `NormalPriorPose2D` cost function
  * Use analytic `NormalPriorPose2D` cost function in
  `AbsolutePose2DStampedConstraint`
  * Test `NormalPriorPose2D` jacobians are correct, comparing against
  `NormalPriorPose2DCostFunctor` using automatic differentiation
  * Benchmark `NormalPriorPose2D` vs `NormalPriorPose2DCostFunctor` using
  automatic differentiation for 1, 2 and 3 residuals. The latency
  speedup is approximately 2.36, 2.76 and 3.44, respectively.
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Fix Unicycle2DIgnition set_pose (`#154 <https://github.com/locusrobotics/fuse/issues/154>`_)
  * Initialize StateHistoryElement::velocity_yaw
  * Process ignition transactions individually
  * Call motion model generator with last stamp
  * Skip optimization cycle if transaction is empty
* Fix compute elimination order with orphan variables (`#136 <https://github.com/locusrobotics/fuse/issues/136>`_)
  * Test computeEliminationOrder with orphan variables
  * Fix computeEliminationOrder with orphan variables
* Fix get constraints with lvalue iterator input (`#134 <https://github.com/locusrobotics/fuse/issues/134>`_)
  * Change getConstraints signature to return iterator
  * Update marginalize code to fix the previous usage error
* Support printing VariableConstraints objects (`#132 <https://github.com/locusrobotics/fuse/issues/132>`_)
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* Revert "Fix build with ceres 2.0 with CMake < 3.8 (`#106 <https://github.com/locusrobotics/fuse/issues/106>`_)" (`#120 <https://github.com/locusrobotics/fuse/issues/120>`_)
  This reverts commit 9933456ecc24ba9b649a8a2885be3f852306efee.
* Wrap normal delta pose 2d orientation angle error (`#122 <https://github.com/locusrobotics/fuse/issues/122>`_)
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* Support empty linear terms in marginalizeNext (`#111 <https://github.com/locusrobotics/fuse/issues/111>`_)
  * Enforce constness
  * Unshadow variable_uuid in inner for loop
  * Support empty linear terms in marginalizeNext
* Fix build with ceres 2.0 with CMake < 3.8 (`#106 <https://github.com/locusrobotics/fuse/issues/106>`_)
  * Note that while the Ceres 2.0 build completes, there may still be some lingering issues.
* [RST-1951] speed optimizations (`#100 <https://github.com/locusrobotics/fuse/issues/100>`_)
  * Improved random UUID generator
  * Minor Eigen assignment speed improvements
* [RST-2427] Added a 'source' field to the constraints. This is an API-breaking change. (`#101 <https://github.com/locusrobotics/fuse/issues/101>`_)
* [RST-2340] Add serialization support to fuse (`#98 <https://github.com/locusrobotics/fuse/issues/98>`_)
* Fix -Wall -Wextra warnings in tests (`#80 <https://github.com/locusrobotics/fuse/issues/80>`_)
* Fix -Wall -Wextra warnings (`#77 <https://github.com/locusrobotics/fuse/issues/77>`_)
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams

0.4.0 (2019-07-12)
------------------
* Depend on libceres-dev instead of ceres-solver (`#71 <https://github.com/locusrobotics/fuse/issues/71>`_)
* [RST-2144] Support proper Eigen memory alignment (`#65 <https://github.com/locusrobotics/fuse/issues/65>`_)
* [RST-2088] Fix bug causing bad marginal computations occasionally (`#60 <https://github.com/locusrobotics/fuse/issues/60>`_)
* [RST-1747] fixed lag smoother implementation (`#52 <https://github.com/locusrobotics/fuse/issues/52>`_)
* Modified Variable class to make the UUID immutable (`#55 <https://github.com/locusrobotics/fuse/issues/55>`_)
* RST-2025 Fixing size issues (`#53 <https://github.com/locusrobotics/fuse/issues/53>`_)
  * Fixing size issue with 2D poses
  * Fixed the size check in MarginalConstraint (`#54 <https://github.com/locusrobotics/fuse/issues/54>`_)
* [RST-1745] Added a marginalizeVariables() function (`#48 <https://github.com/locusrobotics/fuse/issues/48>`_)
* [RST-1745] Created a container to hold the list of constraints by variable (`#47 <https://github.com/locusrobotics/fuse/issues/47>`_)
* [RST-1745] Created a uuid<-->index bidirectional lookup class (`#46 <https://github.com/locusrobotics/fuse/issues/46>`_)
* [RST-1744] Added a marginal constraint class (`#43 <https://github.com/locusrobotics/fuse/issues/43>`_)
* [RST-1940] Added a localSize() method to the Variable class (`#42 <https://github.com/locusrobotics/fuse/issues/42>`_)
* [RST-1927] Update the local parameterization for the orientation variables (`#41 <https://github.com/locusrobotics/fuse/issues/41>`_)
* [RST-1926] Extend the local parameter definition to include Minus() (`#40 <https://github.com/locusrobotics/fuse/issues/40>`_)
* Contributors: Enrique Fernández Perdomo, Stephen Williams, Tom Moore

0.3.0 (2019-03-18)
------------------

0.2.0 (2019-01-16)
------------------
* [RST-1567] Check the system has started before attempting to optimize (`#33 <https://github.com/locusrobotics/fuse/issues/33>`_)
  * Check the system has started before attempting to optimize.
  * Fixed linter issues
* RST-1559 Adding partial measurement support for relative 2D pose data (`#32 <https://github.com/locusrobotics/fuse/issues/32>`_)
  * Adding partial measurement support for relative 2D pose data
* Adding subset measurement support for AbsolutePose2DStampedConstraint (`#31 <https://github.com/locusrobotics/fuse/issues/31>`_)
  * Adding subset measurement support
* [RST-1554] test depends (`#30 <https://github.com/locusrobotics/fuse/issues/30>`_)
  * Refactored all CMakeLists.txt to avoid path issues when using workspace overlays
* RST-1239 Adding 3D relative pose constraint (`#27 <https://github.com/locusrobotics/fuse/issues/27>`_)
  * Adding 3D relative pose constraint
  * Updating expected covariance matrices for anything involving rotations
* Contributors: Stephen Williams, Tom Moore

0.1.1 (2018-08-15)
------------------

0.1.0 (2018-08-12)
------------------
* [RST-1121] Moved the pose publishers (`#19 <https://github.com/locusrobotics/fuse/issues/19>`_)
  * Clean up Eigen depends and includes
* Adding absolute 3d pose
* Fixing quaternion delta computation
* Converted all Eigen objects to use row-major order (`#22 <https://github.com/locusrobotics/fuse/issues/22>`_)
* Fixed covariance matrix size (`#21 <https://github.com/locusrobotics/fuse/issues/21>`_)
* Adding 3D orientation constraints
* Contributors: Stephen Williams, Tom Moore

0.0.2 (2018-07-16)
------------------
* fixed cut&paste error (`#13 <https://github.com/locusrobotics/fuse/issues/13>`_)
* Added absolute and relative 2D constraints (`#8 <https://github.com/locusrobotics/fuse/issues/8>`_)
* Contributors: Stephen Williams

0.0.1 (2018-07-05)
------------------
