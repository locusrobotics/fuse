^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2021-07-20)
------------------
* Adding roslint dependency to fuse_viz (`#231 <https://github.com/locusrobotics/fuse/issues/231>`_)
  * Adding roslint dependency to fuse_viz
  * Silence CMP0048 warnings
* Contributors: Tom Moore

0.4.1 (2021-07-13)
------------------
* Getting versions in sync
* Substract minimum twist covariance from twist covariance (`#222 <https://github.com/locusrobotics/fuse/issues/222>`_)
  * Substract min twist cov from twist cov
  If the twist covariance already had a minimum twist covariance added to
  it to prevent ill-conditioned covariance matrices, we need a way to
  substract that minimum twist covariance from it before we compute the
  pose relative covariance. Otherwise, we cannot get the original pose
  relative covariance because the minimum twist covariance term is
  multiplies by the time delta, which could actually make the resulting
  pose relative covariance ill-conditioned or very small, i.e.
  overconfident.
* [Issue `#223 <https://github.com/locusrobotics/fuse/issues/223>`_] Add an optional tf_timeout parameter to the sensor models (`#224 <https://github.com/locusrobotics/fuse/issues/224>`_)
* Factorize differential mode processing (`#219 <https://github.com/locusrobotics/fuse/issues/219>`_)
  * Factorize differential mode processing
  * Throttle log message when transform message fails
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
* Use fuse_core::getPositiveParam for all ros::Duration parameters (`#212 <https://github.com/locusrobotics/fuse/issues/212>`_)
  * Use fuse_core::getPositiveParam for ros::Duration
  * Use fuse_core::getPositiveParam for TF durations
* Add tcp_no_delay parameter to sensor models (`#211 <https://github.com/locusrobotics/fuse/issues/211>`_)
* Add ability to throttle covariance computation (`#209 <https://github.com/locusrobotics/fuse/issues/209>`_)
* Add invert_tf to Odometry2DPublisher (`#206 <https://github.com/locusrobotics/fuse/issues/206>`_)
  This allows to publish the inverse TF transform.
  This can be useful to skip the odom->base_link lookupTransform() when
  braodcasting map->base_link, which must be broadcasted as map->odom
  because TF tree doesn't support more than a single parent per frame,
  base_link in this case. This is particular relevant when
  predict_to_current_time is enabled, because the lookupTransform() could
  take a while, causing delays.
* Support throttling serialized graph publisher (`#204 <https://github.com/locusrobotics/fuse/issues/204>`_)
  * Change sensor proc from gtest to gmock target
  * Move ThrottledCallback to fuse_core
  * Support generic callbacks in ThrottledCallback
  * Throttle graph publishing
  * Overload getPositiveParam for ros::Duration
  * Use getPositiveParam for ros::Duration parameters
* Add linear acceleration to synchronizer (`#205 <https://github.com/locusrobotics/fuse/issues/205>`_)
* Use local latest_stamp in notifyCallback (`#203 <https://github.com/locusrobotics/fuse/issues/203>`_)
* Use dedicated spinner for publish timer callback (`#201 <https://github.com/locusrobotics/fuse/issues/201>`_)
  * Use dedicated spinner for publishTimerCallback
  * This reduces the jitter in the output topics and TF transform stamp
  because it allows the notifyCallback and publishTimerCallback to run
  concurrently. The notifyCallback might take longer than the timer
  period sometimes, mostly because the covariance computation is an
  expensive operation.
  * There is a subtle change of behaviour with this implementation!
  Before, the publishTimerCallback overwrote the odom_output\_ and
  acceleration_output\_ with the predicted state. Now it does not, and
  if it gets called twice or more times consecutively, it predicts since
  the last time the state was computed and updated in the
  notifyCallback. With the notifyCallback and publishTimerCallback
  running concurrently it is not trivial to keep the previous behaviour
  efficiently, because we would have to lock the entire callbacks to
  avoid the publishTimerCallback to overwrite a new state being computed
  concurrently in the notifyCallback. That being said, the predicted
  state is likely the same in both implementation. That is, the result
  is likely the same if we use multiple steps or a single one to predict
  the last state forward to the current time.
* Add fuse_models::GraphIgnition sensor model (`#196 <https://github.com/locusrobotics/fuse/issues/196>`_)
* Add fuse_models::Transaction sensor model (`#195 <https://github.com/locusrobotics/fuse/issues/195>`_)
* Fix Unicycle2D constructor doxygen (`#198 <https://github.com/locusrobotics/fuse/issues/198>`_)
* Remove deprecated ::Model models leftovers (`#194 <https://github.com/locusrobotics/fuse/issues/194>`_)
  * Remove fuse_models::twist_2d::Model plugin declaration
  * Remove empty space in fuse_plugins.xml
  * Update ::Model names to new names in doxygen comments
* Conditionally test_depend on benchmark (`#189 <https://github.com/locusrobotics/fuse/issues/189>`_)
* Fix typo in jacobian comments (`#191 <https://github.com/locusrobotics/fuse/issues/191>`_)
* Fix throttle (`#190 <https://github.com/locusrobotics/fuse/issues/190>`_)
  * Update last called time adding throttle period
  Instead of setting to now, which could be larger than the expected call
  time.
  * Init last called time to now the first time
  * Fix check for init/zero last called time
  We cannot use isValid because that does not check the last called time
  is zero, but a completely different thing. We must use isZero.
* Check canTransform output and show error if false (`#188 <https://github.com/locusrobotics/fuse/issues/188>`_)
  * Check canTransform output and show error if false
  * Fix pose -> twist typo
  * Lookup transform directly
* Use std::enable_if_t (`#187 <https://github.com/locusrobotics/fuse/issues/187>`_)
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Only call generator if motion model history empty (`#181 <https://github.com/locusrobotics/fuse/issues/181>`_)
  * Only call generator if motion model history empty
  * Handle dt == 0 special case in motion model
  * Revert test_timestamp_manager.cpp `#154 <https://github.com/locusrobotics/fuse/issues/154>`_
  * Add EmptySingleStamp test
* Add use_twist_covariance ROS param and logic to Imu2D differential orientation measurements (`#178 <https://github.com/locusrobotics/fuse/issues/178>`_)
  * Move pose into previous_pose\_
  This makes the Odometry2D do the same as the Imu2D.
  * Allow Imu2D to use twist covariance
  For differential orientation measurements.
  * Move pose relative covariance closer to use
* Validate unicycle 2d (`#180 <https://github.com/locusrobotics/fuse/issues/180>`_)
  * Remove unused EPSILON constexpr
  * Validate Unicyle2D state and process noise
  * Add disable_checks param (defaults to false)
  * Validate state1 and state2 are finite
  * Validate process noise covariance (after it's been scaled and
  multiplied by dt)
* Fix doxygen comment (`#177 <https://github.com/locusrobotics/fuse/issues/177>`_)
* Delay throttle no valid state message (`#175 <https://github.com/locusrobotics/fuse/issues/175>`_)
  This requires rosconsole >=1.13.8.
* Throttle (`#162 <https://github.com/locusrobotics/fuse/issues/162>`_)
  * Support throttling sensor model inputs
  * Add ThrottledCallback rostest
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
* Don't require frame if empty indices (`#166 <https://github.com/locusrobotics/fuse/issues/166>`_)
* Fix Unicycle2DIgnition set_pose (`#154 <https://github.com/locusrobotics/fuse/issues/154>`_)
  * Initialize StateHistoryElement::velocity_yaw
  * Process ignition transactions individually
  * Call motion model generator with last stamp
  * Skip optimization cycle if transaction is empty
* C++14 for test_unicycle_2d_state_cost_function (`#157 <https://github.com/locusrobotics/fuse/issues/157>`_)
* Print state history (`#156 <https://github.com/locusrobotics/fuse/issues/156>`_)
  * Add print method to StateHistoryElement
  * Add print method to Unicycle2D
  It only prints the history state for now though.
* Minor typo fixes (`#155 <https://github.com/locusrobotics/fuse/issues/155>`_)
* Get minimum_pose_relative_covariance_diagonal (`#150 <https://github.com/locusrobotics/fuse/issues/150>`_)
  Regardless of the value of `independent`, because the
  `fuse_models::Odometry2D` sensor model checks for `use_twist_covariance`
  before `independent`, and we could end up with an uninitialized
  `minimum_pose_relative_covariance_diagonal`.
* Support ScaledLoss (`#141 <https://github.com/locusrobotics/fuse/issues/141>`_)
* Remove duplicated roslint build_depend (`#146 <https://github.com/locusrobotics/fuse/issues/146>`_)
* Remove old acceleration_2d folder (`#145 <https://github.com/locusrobotics/fuse/issues/145>`_)
* Cleanup validation checks (`#139 <https://github.com/locusrobotics/fuse/issues/139>`_)
  * Add getCovarianceDiagonalParam helper
  This allows to load a covariance matrix from the parameter server,
  provided in a list with the diagonal values.
  * Add isSymmetric and isPositiveDefinite helper functions
* Use twist covariance for differential dependent (`#138 <https://github.com/locusrobotics/fuse/issues/138>`_)
  In the `fuse_models::Odometry2D` sensor model, when `differential: true`
  and `independent: false`, the relative pose covariance should NOT be
  computed from the consecutive absolute pose covariance matrices because
  they grow unbounded, so the resulting relative pose covariance suffers
  from numerical issues.
  Instead, we can use the twist covariance of the last pose to compute the
  relative pose covariance, using the time difference between the
  consecutive absolute poses.
  The only limitation is that we cannot throttle the input topics, because
  otherwise the twist covariance from the intermediate/throttled messages
  is missed. We'll have to throttle inside the sensor model, by
  integrating the intermediate messages.
* Support dependent relative pose measurements (`#137 <https://github.com/locusrobotics/fuse/issues/137>`_)
  * Added a "dependent" covariance calculation option to the "differential" mode
  * Added an `independent` param that defaults to `true` to keep the current behaviour
  * Added a `minimum_pose_relative_covariance_diagonal` param that is added to the
  resulting pose relative covariance in order to guarantee that it's not zero or ill-conditioned.
* Scale process noise covariance (`#130 <https://github.com/locusrobotics/fuse/issues/130>`_)
  * Scale process noise covariance
  This scales the process noise covariance pose by the norm of the current
  state velocity.
  A new parameter `velocity_norm_min` is added, that prevents the process
  noise scaling from setting the pose components to zero or a very small
  value that could lead to NaN or a rank deficient Jacobian in the problem
  solved, due to an ill-condition covariance for the process noise.
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
* Publish linear acceleration (`#129 <https://github.com/locusrobotics/fuse/issues/129>`_)
  * Publish linear acceleration
  * Also use linear acceleration if predicting to the current time if the
  new param `predict_with_acceleration` is `true` (default value).
* Explicitly call boost::range::join (`#128 <https://github.com/locusrobotics/fuse/issues/128>`_)
  Otherwise we could get a compilation error due to an ambiguous overloaded `join` function when  some additional `boost/algorithm` headers are included.
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Validate partial measurements (`#125 <https://github.com/locusrobotics/fuse/issues/125>`_)
* Don't read pose_target_frame if differential (`#126 <https://github.com/locusrobotics/fuse/issues/126>`_)
  If differential is true, the pose_target_frame is not used.
* Only allow exact timestamp transformations (`#123 <https://github.com/locusrobotics/fuse/issues/123>`_)
* Benchmark unicycle_2d state cost function (`#121 <https://github.com/locusrobotics/fuse/issues/121>`_)
  The benchmark targets are now only build if CATKIN_ENABLE_TESTING is ON,
  which means that benchmark is now a test_depend and not a depend.
  However, the benchmarks are NOT gtests, so they are built directly on
  catkin build, i.e. there is no need to run make run_tests after. For
  this reason, the find_package on benchmark is no longer REQUIRED,
  but QUIET instead. The benchmark is built only if the benchmark package
  is FOUND.
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* Predict jacobians per parameter block (`#115 <https://github.com/locusrobotics/fuse/issues/115>`_)
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* Wait for reset service existence (`#116 <https://github.com/locusrobotics/fuse/issues/116>`_)
* Publish odometry with timer and allow to predict it (`#109 <https://github.com/locusrobotics/fuse/issues/109>`_)
* Use measurement stamps for transformed variables (`#113 <https://github.com/locusrobotics/fuse/issues/113>`_)
* [RST-2149] Added the configured device_id to the log message (`#110 <https://github.com/locusrobotics/fuse/issues/110>`_)
* [RST-2438] Make ceres params loaders reusable (`#104 <https://github.com/locusrobotics/fuse/issues/104>`_)
  * Moved the Ceres loadFromROS functions into reusable functions in fuse_core
  * Load solver parameters for the batch optimizer
* Expose Ceres Solver, Problem and Covariance Options as ROS parameters (`#78 <https://github.com/locusrobotics/fuse/issues/78>`_)
* [RST-2427] Added a 'source' field to the constraints. This is an API-breaking change. (`#101 <https://github.com/locusrobotics/fuse/issues/101>`_)
* [RST-2340] Add serialization support to fuse (`#98 <https://github.com/locusrobotics/fuse/issues/98>`_)
* RST-2390 Renaming unicycle_2d (`#90 <https://github.com/locusrobotics/fuse/issues/90>`_)
  * Renaming unicycle_2d
* Renaming twist_2d (`#89 <https://github.com/locusrobotics/fuse/issues/89>`_)
* Renaming pose_2d (`#88 <https://github.com/locusrobotics/fuse/issues/88>`_)
* Renaming odometry_2d (`#87 <https://github.com/locusrobotics/fuse/issues/87>`_)
* Renaming imu_2d (`#86 <https://github.com/locusrobotics/fuse/issues/86>`_)
* RST-2390 Renaming acceleration_2d (`#85 <https://github.com/locusrobotics/fuse/issues/85>`_)
  * Renaming acceleration_2d
* Renaming package to fuse_models
* Preparing for move
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams, Tom Moore, sjphilli

0.4.0 (2019-08-14)
------------------

0.3.0 (2019-08-14)
------------------

0.2.0 (2019-07-12)
------------------
* Get predict_to_current_time ROS param (`#17 <https://github.com/locusrobotics/fuse_rl/issues/17>`_)
* [RST-2202] Catch potential errors when computing the covariances (`#18 <https://github.com/locusrobotics/fuse_rl/issues/18>`_)
  * Clear the covariance on error
* Remove angles header not used (`#16 <https://github.com/locusrobotics/fuse_rl/issues/16>`_)
* Default to private ~reset and ~set_pose names (`#14 <https://github.com/locusrobotics/fuse_rl/issues/14>`_)
* Depend on sensor_msgs and nav_msgs (`#15 <https://github.com/locusrobotics/fuse_rl/issues/15>`_)
* Depend on libceres-dev instead of ceres-solver (`#11 <https://github.com/locusrobotics/fuse_rl/issues/11>`_)
  * Depend on libceres-dev instead of ceres-solver
  * Add missed depend on angles
* Resolve names before subscribing (`#10 <https://github.com/locusrobotics/fuse_rl/issues/10>`_)
* Linter/style changes
* Use std::bind instead of std::bind2nd
  std::bind2nd is marked as deprecated in C++11
  Co-Authored-By: Stephen Williams <stephen.vincent.williams@gmail.com>
* Add sensor_proc test
  Only for:
  * mergeIndices
  * appendPartialMeasurement
* Fix appendPartialMeasurement by merging indices
  Position and orientation indices are merged together into a single
  std::vector<size_t> of indices, applying the appropriate offset to the
  orientation indices.
  This is passed to appendPartialMeasurement, which should be called only
  once. It doesn't need the base_index and offset args anymore.
* [RST-2128] fuse rl ignition sensor (`#6 <https://github.com/locusrobotics/fuse_rl/issues/6>`_)
* [RST-2144] Updated macro calls on all objects to support proper Eigen memory alignment (`#8 <https://github.com/locusrobotics/fuse_rl/issues/8>`_)
* Bug in motion model history (`#7 <https://github.com/locusrobotics/fuse_rl/issues/7>`_)
* Use linear indices for linear velocity (`#5 <https://github.com/locusrobotics/fuse_rl/issues/5>`_)
  Not angular indices, which is wrong and produces a crash at runtime
  because an assert fails.
* Fix appendPartialMeasurement assignments (`#4 <https://github.com/locusrobotics/fuse_rl/issues/4>`_)
* Store ros::Subscriber in sensor model attribute (`#3 <https://github.com/locusrobotics/fuse_rl/issues/3>`_)
  Store ros::Subscriber in sensor model attribute
* Tailor: Creating Jenkinsfile
* Adding base_link_output_frame_id to the 2D odom publisher (`#1 <https://github.com/locusrobotics/fuse_rl/issues/1>`_)
* Contributors: Enrique Fernandez, Enrique Fernandez Perdomo, Enrique Fernández Perdomo, Stephen Williams, Tom Moore, locus-services

0.1.0 (2019-03-18)
------------------
* [RST-1625] Use the stamped variable synchronizer (`#13 <https://github.com/locusrobotics/fuse_rl/issues/13>`_)
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Updated package for changes to fuse_core::Transaction (`#11 <https://github.com/locusrobotics/fuse_rl/issues/11>`_)
* Tailor: Creating Jenkinsfile
* Simplifying
* Just using rotation
* Adding tf2 overloads for twist and acceleration
* Updated derived sensors for recent change to the sensor API
* Adding 2D odometry publisher
* Adding 2D IMU sensor model
* Feature/pedantic style change (`#7 <https://github.com/locusrobotics/fuse_rl/issues/7>`_)
  * White spaaaaaaaaaaaaaaaaaaaaaaace
  * Moar whitespace
* PR feedback
* Adding support for partial measurements
* Adding 2D odometry sensor
* Enabling partial measurements for fuse_rl
* PR feedback
* PR feedback
* Adding 2d pose sensor
* Adding 2D odometry sensor
* Adding 2D acceleration sensor
* Adding 2D twist sensor
* More comments
* Removing comment
* Using Jacobians to rotate covariances
* PR feedback
* PR feedback
* Adding ability to transform poses
* Adding 2d pose sensor
* Update README.md
* Adding 2D kinematic constraint
* Adding README
* Contributors: David V. Lu!!, Stephen Williams, Tom Moore, locus-services
