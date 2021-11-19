^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_optimizers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2021-07-20)
------------------
* Adding roslint dependency to fuse_viz (`#231 <https://github.com/locusrobotics/fuse/issues/231>`_)
  * Adding roslint dependency to fuse_viz
  * Silence CMP0048 warnings
* Contributors: Tom Moore

0.4.1 (2021-07-13)
------------------
* Improve logs on graph update exception thrown (`#227 <https://github.com/locusrobotics/fuse/issues/227>`_)
  * Catch graph update exceptions and log fatal msg
  When the graph update method throws an exception the node crashes and
  there is no way to know what transaction caused the exception because we
  call notify after successfully updating the graph.
  This catches any exception thrown by the graph update method and logs a
  FATAL message that includes the current graph and the new transaction.
  * Add full stop to exception messages
  This homogenizes the exception messages from all exceptions thrown by
  the HashGraph class.
* Fix diagnostics typo (`#218 <https://github.com/locusrobotics/fuse/issues/218>`_)
* Target frame optional (`#217 <https://github.com/locusrobotics/fuse/issues/217>`_)
  * Make target_frame optional
  * Remove optional target_frame parameters in tests
* Transform message in differential mode (`#216 <https://github.com/locusrobotics/fuse/issues/216>`_)
  * Transform message in differential mode
  This is important because the relative transformation is not the same if
  the sensor and target frame are different.
  Consider for example the case of an IMU sensor upside down:
  * The robot base frame is base_link
  * The IMU sensor frame is imu_link
  * The imu_link transformation wrt base_link is 180 degrees wrt the y or
  x axis
  * The angular velocity around the z axis has opposite sign in the
  sensor frame wrt the target frame
  * Require pose_target_frame in differential mode
  * Roslint
  * Add pose_target_frame to optimizer test config
* Check empty transaction with empty() method (`#210 <https://github.com/locusrobotics/fuse/issues/210>`_)
* Add solver summary info to diagnostics (`#208 <https://github.com/locusrobotics/fuse/issues/208>`_)
  * Add solver summary info to diagnostics
  * Abort if optimization failed
* Support throttling serialized graph publisher (`#204 <https://github.com/locusrobotics/fuse/issues/204>`_)
  * Change sensor proc from gtest to gmock target
  * Move ThrottledCallback to fuse_core
  * Support generic callbacks in ThrottledCallback
  * Throttle graph publishing
  * Overload getPositiveParam for ros::Duration
  * Use getPositiveParam for ros::Duration parameters
* Fix optimizer test config (`#202 <https://github.com/locusrobotics/fuse/issues/202>`_)
  * Add dimensions to the sensor models, so we do not get these warnings:
  ``` bash
  [ WARN] [/Optimizer] [1604556511.895230377]: No dimensions were specified. Data from topic /imu will be ignored.
  [ WARN] [/Optimizer] [1604556511.899319309]: No dimensions were specified. Data from topic /pose will be ignored.
  [ WARN] [/Optimizer] [1604556511.905227451]: No dimensions were specified. Data from topic /odom will be ignored.
  ```
  * Add `twist_target_frame` to `imu` sensor model, so we do not get this
  error:
  ``` bash
  [FATAL] [/Optimizer] [1604557006.977288017]: Could not find required parameter twist_target_frame in namespace /Optimizer/imu
  ```
* Purge transactions older than ignition max stamp (`#183 <https://github.com/locusrobotics/fuse/issues/183>`_)
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Filter out transactions older than the lag window (`#173 <https://github.com/locusrobotics/fuse/issues/173>`_)
  * Filter out transactions older than the lag window
  * Fix expiration time computation
  * Reset the lag expiration time when the smoother is reset
  * Reorganize class variables by their mutex guard
  * Add a mutex guard for start_time\_; use start_time\_ as a min time in the lag expiation computation
  * Add minStamp() and maxStamp() accessors to the Transaction class
  * Use the minStamp() and maxStamp() accessors to filter and purge transactions correctly
* The started/ignited variables are accessed from multiple threads. (`#172 <https://github.com/locusrobotics/fuse/issues/172>`_)
* Replace ignition_sensors list param with ignition field (`#163 <https://github.com/locusrobotics/fuse/issues/163>`_)
  * Remove ignition_sensors param and use a per-sensor ignition field
* Get positive param API change (`#169 <https://github.com/locusrobotics/fuse/issues/169>`_)
  * Change getPositiveParam API
  In order to match the getParam and getRequiredParam, so the value is
  not returned, but set in an in/out argument.
  * Move getPositiveParam and other param related functions to
  parameter.h from util.h, and updated the ros/unit tests accordingly.
  * Fix wrapAngle2D expected range to [-Pi, +Pi)
  Instead of (-Pi, +Pi], and update unit test to reflect that.
* Add missed fuse_models dependencies (`#164 <https://github.com/locusrobotics/fuse/issues/164>`_)
* From a usage standpoint, the 'element' variable is getting modified and should not be const. The const was not causing compilation issues before because of some pointer indirection. (`#160 <https://github.com/locusrobotics/fuse/issues/160>`_)
* Added unit test to illustrate variable initialization bug (`#158 <https://github.com/locusrobotics/fuse/issues/158>`_)
* Fix Unicycle2DIgnition set_pose (`#154 <https://github.com/locusrobotics/fuse/issues/154>`_)
  * Initialize StateHistoryElement::velocity_yaw
  * Process ignition transactions individually
  * Call motion model generator with last stamp
  * Skip optimization cycle if transaction is empty
* Support YAML struct for models and publishers (`#149 <https://github.com/locusrobotics/fuse/issues/149>`_)
  * Support YAML struct for models and publishers
  This allows to compound multiple YAML files that provide additional
  models or publishers. This cannot be done with a list/array, because the
  previous values get overwritten/lost.
* Throttle optimization duration exceeded warning (`#140 <https://github.com/locusrobotics/fuse/issues/140>`_)
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* Initialize diagnostic_updater_timer_period\_ (`#114 <https://github.com/locusrobotics/fuse/issues/114>`_)
* Add diagnostic updater (`#108 <https://github.com/locusrobotics/fuse/issues/108>`_)
* [RST-2438] Make ceres params loaders reusable (`#104 <https://github.com/locusrobotics/fuse/issues/104>`_)
  * Moved the Ceres loadFromROS functions into reusable functions in fuse_core
  * Load solver parameters for the batch optimizer
* Expose Ceres Solver, Problem and Covariance Options as ROS parameters (`#78 <https://github.com/locusrobotics/fuse/issues/78>`_)
* [RST-2427] Added a 'source' field to the constraints. This is an API-breaking change. (`#101 <https://github.com/locusrobotics/fuse/issues/101>`_)
* [RST-2432] Reworked the transaction queue to skip transactions on a per-sensor basis (`#102 <https://github.com/locusrobotics/fuse/issues/102>`_)
* [RST-2340] Add serialization support to fuse (`#98 <https://github.com/locusrobotics/fuse/issues/98>`_)
* Fix -Wall -Wextra warnings in tests (`#80 <https://github.com/locusrobotics/fuse/issues/80>`_)
* [RST-2148] Added start() and stop() methods to the MotionModel, SensorModel, and Publisher API (`#75 <https://github.com/locusrobotics/fuse/issues/75>`_)
  * Added start() and stop() methods to the MotionModel, SensorModel, and Publisher API
  * Added the ability to clear the callback queue of the optimizer
  * Refactor the fixed-lag reset callback to use the plugins' stop() and start() methods
* Fix -Wall -Wextra warnings (`#77 <https://github.com/locusrobotics/fuse/issues/77>`_)
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams

0.4.0 (2019-07-12)
------------------
* Wait for a valid timestamp before loading the plugins. This prevents the ignition sensor from generating an initial state with the wrong stamp. (`#74 <https://github.com/locusrobotics/fuse/issues/74>`_)
* Default private ~reset service name (`#72 <https://github.com/locusrobotics/fuse/issues/72>`_)
* [RST-2186] Added a FixedLagSmootherParams struct (`#68 <https://github.com/locusrobotics/fuse/issues/68>`_)
* [RST-2185] Fix "fixed-lag crash on reset" bug (`#66 <https://github.com/locusrobotics/fuse/issues/66>`_)
* [RST-2144] Support proper Eigen memory alignment (`#65 <https://github.com/locusrobotics/fuse/issues/65>`_)
* [RST-2158] Fix thread locking issue that was causing underconstained solver errors (`#63 <https://github.com/locusrobotics/fuse/issues/63>`_)
* [RST-2128] Added a "reset" service to the fixed lag smoother (`#61 <https://github.com/locusrobotics/fuse/issues/61>`_)
* [RST-1747] fixed lag smoother implementation (`#52 <https://github.com/locusrobotics/fuse/issues/52>`_)
* Contributors: Enrique Fernández Perdomo, Stephen Williams

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
* [RST-1567] Check the system has started before attempting to optimize (`#33 <https://github.com/locusrobotics/fuse/issues/33>`_)
  * Check the system has started before attempting to optimize.
  * Fixed linter issues
* [RST-1554] test depends (`#30 <https://github.com/locusrobotics/fuse/issues/30>`_)
  * Refactored all CMakeLists.txt to avoid path issues when using workspace overlays
* Contributors: Stephen Williams

0.1.1 (2018-08-15)
------------------

0.1.0 (2018-08-12)
------------------
* [RST-1121] move optimizers (`#25 <https://github.com/locusrobotics/fuse/issues/25>`_)
  * Moved the Optimizer and BatchOptimizer classes into the public repo
  * Added fuse_optimizers to the metapackage depends
  * Changed optimizer to unique ownership of the graph. This better captures the usage.
* Contributors: Stephen Williams

0.0.2 (2018-07-16)
------------------

0.0.1 (2018-07-05)
------------------
