^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2021-07-13)
------------------
* Support throttling serialized graph publisher (`#204 <https://github.com/locusrobotics/fuse/issues/204>`_)
  * Change sensor proc from gtest to gmock target
  * Move ThrottledCallback to fuse_core
  * Support generic callbacks in ThrottledCallback
  * Throttle graph publishing
  * Overload getPositiveParam for ros::Duration
  * Use getPositiveParam for ros::Duration parameters
* Use std::enable_if_t (`#187 <https://github.com/locusrobotics/fuse/issues/187>`_)
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Remove variables on hold (`#185 <https://github.com/locusrobotics/fuse/issues/185>`_)
  * Test variables on hold are removed when removing a variable
  * Erase variable on hold when removing variable
* Filter out transactions older than the lag window (`#173 <https://github.com/locusrobotics/fuse/issues/173>`_)
  * Filter out transactions older than the lag window
  * Fix expiration time computation
  * Reset the lag expiration time when the smoother is reset
  * Reorganize class variables by their mutex guard
  * Add a mutex guard for start_time\_; use start_time\_ as a min time in the lag expiation computation
  * Add minStamp() and maxStamp() accessors to the Transaction class
  * Use the minStamp() and maxStamp() accessors to filter and purge transactions correctly
* Call onStop() directly if !ros::ok() in stop() (`#182 <https://github.com/locusrobotics/fuse/issues/182>`_)
  * Call onStop() directly if !ros::ok() in stop()
  * Stop spinner before calling onStop()
* Only call generator if motion model history empty (`#181 <https://github.com/locusrobotics/fuse/issues/181>`_)
  * Only call generator if motion model history empty
  * Handle dt == 0 special case in motion model
  * Revert test_timestamp_manager.cpp `#154 <https://github.com/locusrobotics/fuse/issues/154>`_
  * Add EmptySingleStamp test
* Fix doxygen comment (`#177 <https://github.com/locusrobotics/fuse/issues/177>`_)
* Delay throttle no valid state message (`#175 <https://github.com/locusrobotics/fuse/issues/175>`_)
  This requires rosconsole >=1.13.8.
* Replace ignition_sensors list param with ignition field (`#163 <https://github.com/locusrobotics/fuse/issues/163>`_)
  * Remove ignition_sensors param and use a per-sensor ignition field
* Throttle (`#162 <https://github.com/locusrobotics/fuse/issues/162>`_)
  * Support throttling sensor model inputs
  * Add ThrottledCallback rostest
* Use a static Boost random UUID generator (`#171 <https://github.com/locusrobotics/fuse/issues/171>`_)
* Fix motion model history (`#168 <https://github.com/locusrobotics/fuse/issues/168>`_)
  * Fix the motion model history to maintain *at least* the requested time interval
  * Apply a similar fix to the MessageBuffer class
* Get positive param API change (`#169 <https://github.com/locusrobotics/fuse/issues/169>`_)
  * Change getPositiveParam API
  In order to match the getParam and getRequiredParam, so the value is
  not returned, but set in an in/out argument.
  * Move getPositiveParam and other param related functions to
  parameter.h from util.h, and updated the ros/unit tests accordingly.
  * Fix wrapAngle2D expected range to [-Pi, +Pi)
  Instead of (-Pi, +Pi], and update unit test to reflect that.
* Fix thread issue with UUID generation (`#167 <https://github.com/locusrobotics/fuse/issues/167>`_)
  * Add a mutex lock to the random UUID generation. The STL random number generator is not thread-safe.
* Patch Tukey loss for Ceres < 2.0.0 (`#159 <https://github.com/locusrobotics/fuse/issues/159>`_)
  * Patch Tukey loss for Ceres < 2.0.0
  * Create ceres_macros.h header
* Fix Unicycle2DIgnition set_pose (`#154 <https://github.com/locusrobotics/fuse/issues/154>`_)
  * Initialize StateHistoryElement::velocity_yaw
  * Process ignition transactions individually
  * Call motion model generator with last stamp
  * Skip optimization cycle if transaction is empty
* Add evaluate method to graph (`#151 <https://github.com/locusrobotics/fuse/issues/151>`_)
* Support ScaledLoss (`#141 <https://github.com/locusrobotics/fuse/issues/141>`_)
* Const deserialize (`#148 <https://github.com/locusrobotics/fuse/issues/148>`_)
  * Make TransactionDeserializer::deserialize const
  * Make GraphDeserializer::deserialize const
  This requires the graph_loader\_ to be mutable.
* Cleanup validation checks (`#139 <https://github.com/locusrobotics/fuse/issues/139>`_)
  * Add getCovarianceDiagonalParam helper
  This allows to load a covariance matrix from the parameter server,
  provided in a list with the diagonal values.
  * Add isSymmetric and isPositiveDefinite helper functions
* Better validation of partial measurement output (`#131 <https://github.com/locusrobotics/fuse/issues/131>`_)
  * Relax the default precision when validating the covariance matrix is
  symmetric.
  * Print the covariance matrix with `Eigen::FullPrecision` when the
  symmetry test fails with `isApprox`, so we can see the magnitude of
  the error.
  * Show source if validation fails
  * Changes from throwing/crashing to ROS_ERROR.
  * Add eigenvalues to non-PSD error check
  * Add disable_checks param to sensor models
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Validate partial measurements (`#125 <https://github.com/locusrobotics/fuse/issues/125>`_)
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* Revert "Fix build with ceres 2.0 with CMake < 3.8 (`#106 <https://github.com/locusrobotics/fuse/issues/106>`_)" (`#120 <https://github.com/locusrobotics/fuse/issues/120>`_)
  This reverts commit 9933456ecc24ba9b649a8a2885be3f852306efee.
* Predict jacobians per parameter block (`#115 <https://github.com/locusrobotics/fuse/issues/115>`_)
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* Support ceres 2.0 in tests (`#117 <https://github.com/locusrobotics/fuse/issues/117>`_)
  In Ceres 2.0 we should call AutoDifferentiate<...>(...) instead of
  AutoDiff<...>::Differentiate(...).
* Fix build with ceres 2.0 with CMake < 3.8 (`#106 <https://github.com/locusrobotics/fuse/issues/106>`_)
  * Note that while the Ceres 2.0 build completes, there may still be some lingering issues.
* [RST-2438] Make ceres params loaders reusable (`#104 <https://github.com/locusrobotics/fuse/issues/104>`_)
  * Moved the Ceres loadFromROS functions into reusable functions in fuse_core
  * Load solver parameters for the batch optimizer
* [RST-1951] speed optimizations (`#100 <https://github.com/locusrobotics/fuse/issues/100>`_)
  * Improved random UUID generator
  * Minor Eigen assignment speed improvements
* [RST-2437] Ensure that all variables are updated by the motion model (`#103 <https://github.com/locusrobotics/fuse/issues/103>`_)
* Expose Ceres Solver, Problem and Covariance Options as ROS parameters (`#78 <https://github.com/locusrobotics/fuse/issues/78>`_)
* [RST-2427] Added a 'source' field to the constraints. This is an API-breaking change. (`#101 <https://github.com/locusrobotics/fuse/issues/101>`_)
* [RST-2340] Add serialization support to fuse (`#98 <https://github.com/locusrobotics/fuse/issues/98>`_)
* Fix -Wall -Wextra warnings in tests (`#80 <https://github.com/locusrobotics/fuse/issues/80>`_)
* Stamp merged transaction (`#79 <https://github.com/locusrobotics/fuse/issues/79>`_)
  Set stamp in merged transactions
  Otherwise, merged transactions don't have a stamp.
  The stamp used is the maximum stamp of the two transactions merged.
* [RST-2148] Added start() and stop() methods to the MotionModel, SensorModel, and Publisher API (`#75 <https://github.com/locusrobotics/fuse/issues/75>`_)
  * Added start() and stop() methods to the MotionModel, SensorModel, and Publisher API
  * Added the ability to clear the callback queue of the optimizer
  * Refactor the fixed-lag reset callback to use the plugins' stop() and start() methods
* Fix -Wall -Wextra warnings (`#77 <https://github.com/locusrobotics/fuse/issues/77>`_)
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams

0.4.0 (2019-07-12)
------------------
* Comment unused parameters (`#76 <https://github.com/locusrobotics/fuse/issues/76>`_)
  Otherwise the compilation fails with: -Werror=unused-parameter
  This happens with these flags: -Wall -Wextra
* Add print method to Graph and HashGraph (`#70 <https://github.com/locusrobotics/fuse/issues/70>`_)
* Depend on libceres-dev instead of ceres-solver (`#71 <https://github.com/locusrobotics/fuse/issues/71>`_)
* Return variable UUID by const reference (`#69 <https://github.com/locusrobotics/fuse/issues/69>`_)
* Added a default implementation for the type() method (`#67 <https://github.com/locusrobotics/fuse/issues/67>`_)
* [RST-2144] Support proper Eigen memory alignment (`#65 <https://github.com/locusrobotics/fuse/issues/65>`_)
* [RST-2128] Added a "reset" service to the fixed lag smoother (`#61 <https://github.com/locusrobotics/fuse/issues/61>`_)
* Modified Variable class to make the UUID immutable (`#55 <https://github.com/locusrobotics/fuse/issues/55>`_)
* [RST-1960] Added a tangent/parameter space flag to the covariance function (`#50 <https://github.com/locusrobotics/fuse/issues/50>`_)
* Some minor header cleanup of fuse_core (`#51 <https://github.com/locusrobotics/fuse/issues/51>`_)
* [RST-1949] Added getConnectedVariables() and getConnectedConstraints() (`#45 <https://github.com/locusrobotics/fuse/issues/45>`_)
* [RST-1746] Remove the marginalizeVariable() methods from the Graph class. (`#44 <https://github.com/locusrobotics/fuse/issues/44>`_)
* [RST-1744] Added a marginal constraint class (`#43 <https://github.com/locusrobotics/fuse/issues/43>`_)
* [RST-1940] Added a localSize() method to the Variable class (`#42 <https://github.com/locusrobotics/fuse/issues/42>`_)
* [RST-1927] Update the local parameterization for the orientation variables (`#41 <https://github.com/locusrobotics/fuse/issues/41>`_)
* [RST-1926] Extend the local parameter definition to include Minus() (`#40 <https://github.com/locusrobotics/fuse/issues/40>`_)
* Contributors: Enrique Fernandez Perdomo, Enrique Fernández Perdomo, Stephen Williams

0.3.0 (2019-03-18)
------------------
* [RST-1653] transaction stamps (`#37 <https://github.com/locusrobotics/fuse/issues/37>`_)
  * Moved the set<ros::Time> object that always accompanies a Transaction into the Transaction itself.
  * Updated all related classes to support that change
* [RST-1477] Simplified the sensor<-->optimizer API (`#35 <https://github.com/locusrobotics/fuse/issues/35>`_)
  * Simplified the sensor<-->optimizer API. Moved the implementation details of the optimizer transaction callback into the optimizer where it belongs.
* Contributors: Stephen Williams

0.2.0 (2019-01-16)
------------------
* Fix tests for bionic (`#34 <https://github.com/locusrobotics/fuse/issues/34>`_)
* [RST-1554] test depends (`#30 <https://github.com/locusrobotics/fuse/issues/30>`_)
  * Refactored all CMakeLists.txt to avoid path issues when using workspace overlays
* Contributors: Gary Servin, Stephen Williams

0.1.1 (2018-08-15)
------------------

0.1.0 (2018-08-12)
------------------
* [RST-1121] move optimizers (`#25 <https://github.com/locusrobotics/fuse/issues/25>`_)
  * Added a clone() method to the Transaction object
  * Changed optimizer to unique ownership of the graph. This better captures the usage.
* [RST-1121] Moved the pose publishers (`#19 <https://github.com/locusrobotics/fuse/issues/19>`_)
  * Clean up Eigen depends and includes
* [RST-1121] move publishers (`#18 <https://github.com/locusrobotics/fuse/issues/18>`_)
* [RST-1121] move motion models (`#17 <https://github.com/locusrobotics/fuse/issues/17>`_)
* [RST-1121] move sensor classes (`#16 <https://github.com/locusrobotics/fuse/issues/16>`_)
* [RST-1121] Moved the Graph classes (`#15 <https://github.com/locusrobotics/fuse/issues/15>`_)
* Adding absolute 3d pose
* Converted all Eigen objects to use row-major order (`#22 <https://github.com/locusrobotics/fuse/issues/22>`_)
* Contributors: Stephen Williams, Tom Moore

0.0.2 (2018-07-16)
------------------
* Added the Transaction class and unit tests (`#2 <https://github.com/locusrobotics/fuse/issues/2>`_)
* Added missing test depend (`#3 <https://github.com/locusrobotics/fuse/issues/3>`_)
* Contributors: Stephen Williams

0.0.1 (2018-07-05)
------------------
* Moved the Variable and Constraint classed into the public fuse repo
* Contributors: Stephen Williams
