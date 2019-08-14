^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
