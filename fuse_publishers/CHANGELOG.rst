^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_publishers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.2 (2026-05-05)
------------------

1.3.1 (2025-08-29)
------------------
* Fix tf2_ros header order
* Contributors: Stephen Williams

1.3.0 (2025-07-28)
------------------

1.2.4 (2025-07-28)
------------------
* Update headers for tf2_ros (`#417 <https://github.com/locusrobotics/fuse/issues/417>`_)
* Porting StampedVariableSynchronizer changes to ROS 2 (`#414 <https://github.com/locusrobotics/fuse/issues/414>`_)
  * Porting effort to ROS 2
  * Porting this functionality to ROS 2
  * Responding to comments
* Contributors: David Murdoch, Gary Servin

1.2.3 (2025-05-24)
------------------
* Fix linter errors related to header ordering (`#407 <https://github.com/locusrobotics/fuse/issues/407>`_)
* Removed deprecations warnings (`#406 <https://github.com/locusrobotics/fuse/issues/406>`_)
* Contributors: Alejandro Hernández Cordero, Stephen Williams

1.2.2 (2025-04-26)
------------------
* * Added dependencies in required CMakeLists.txt and package.xml files
  * Added ament_cmake_ros and gtest_vendor dependencies
  * Removed duplicate package depends, alphabetized lists
  See https://www.linkedin.com/posts/open-source-robotics-foundation_were-looking-for-half-a-dozen-new-open-activity-7317690134764605440-jm3h/
  Author: KB1110 <kartikbakshi10@gmail.com>
* [RST-7809] Port fix for negative pi initial conditions from ROS1 to ROS2 (`#335 <https://github.com/locusrobotics/fuse/issues/335>`_)
  * Add some unit tests for the 2D orientation constraints; Create getters/setters for the 2D orientation variable is preparation for a fix.
  * Force the 2D orientation value to be is minimum phase
* Contributors: KB1110, Stephen Williams

1.2.1 (2024-09-21)
------------------

1.2.0 (2024-05-02)
------------------

1.1.1 (2024-05-02)
------------------

1.1.0 (2024-04-20)
------------------
* Port support for Ceres 2.1.0 Manifold class into ROS 2 Rolling (`#366 <https://github.com/locusrobotics/fuse/issues/366>`_)
  * Support gcc12 and ceres 2.1.0
  * Add support for the Manifold class when using Ceres Solver version 2.1.0 and above
  * General clean up for Ceres 2.2.0 support
  * Updated serialization support to be backwards compatible with previously serialized files
  * Formatting changes required for ROS 2 Rolling / Ubuntu Noble
* Contributors: Stephen Williams

1.0.1 (2023-03-03)
------------------

1.0.0 (2023-03-03)
------------------
* Use upstream rclcpp::node_interfaces::NodeInterfaces (`#313 <https://github.com/locusrobotics/fuse/issues/313>`_)
  * Use upstream rclcpp::node_interfaces::NodeInterfaces
  * Dereference node arguments to NodeInterfaces
  ---------
  Co-authored-by: methylDragon <methylDragon@gmail.com>
* fuse -> ROS 2 fuse_models: Linting (`#315 <https://github.com/locusrobotics/fuse/issues/315>`_)
* Use getParameterName and namespace parameters for publishers (`#314 <https://github.com/locusrobotics/fuse/issues/314>`_)
* Fix SerializedPublisher not being able to read it's parameters (`#311 <https://github.com/locusrobotics/fuse/issues/311>`_)
* Use rclcpp::Clock::wait_until_started (`#303 <https://github.com/locusrobotics/fuse/issues/303>`_)
* fuse -> ROS 2 : Doc Generation (`#278 <https://github.com/locusrobotics/fuse/issues/278>`_)
  * Port doc generation and fix package.xml for linting
  * Fix small bugs in package.xml
  * Use default rosdoc2 settings
  * Use default rosdoc2 settings
  * Update fuse_doc for rosdoc2
  ---------
  Co-authored-by: Shane Loretz <sloretz@google.com>
* fuse -> ROS 2 fuse_optimizers: Port fuse_optimizers (`#307 <https://github.com/locusrobotics/fuse/issues/307>`_)
  Co-authored-by: Shane Loretz <sloretz@osrfoundation.org>
  Co-authored-by: Shane Loretz <shane.loretz@gmail.com>
* fuse -> ROS 2 fuse_publishers : Linting (`#305 <https://github.com/locusrobotics/fuse/issues/305>`_)
* fuse -> ROS 2 fuse_publishers: Port fuse_publishers (`#299 <https://github.com/locusrobotics/fuse/issues/299>`_)
  Co-authored-by: Shane Loretz <shane.loretz@gmail.com>
* fuse -> ROS 2 fuse_constraints : Linting (`#298 <https://github.com/locusrobotics/fuse/issues/298>`_)
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
* fuse -> ROS 2 : Port Logging (`#279 <https://github.com/locusrobotics/fuse/issues/279>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* fuse -> ROS 2: Clean up macro usage warnings (`#280 <https://github.com/locusrobotics/fuse/issues/280>`_)
* fuse -> ROS 2 fuse_msgs : Port package and ignore unported packages for now (`#277 <https://github.com/locusrobotics/fuse/issues/277>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* [RST-4186] Fix fuse macro names (`#263 <https://github.com/locusrobotics/fuse/issues/263>`_)
  * Namespace all macros with the FUSE\_ prefix. Mark original macros as deprecated.
  * Update all fuse objects to use the new macro names
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
* Fixing license
* Support throttling serialized graph publisher (`#204 <https://github.com/locusrobotics/fuse/issues/204>`_)
  * Change sensor proc from gtest to gmock target
  * Move ThrottledCallback to fuse_core
  * Support generic callbacks in ThrottledCallback
  * Throttle graph publishing
  * Overload getPositiveParam for ros::Duration
  * Use getPositiveParam for ros::Duration parameters
* Set latch param in serialized publisher to false by default (`#184 <https://github.com/locusrobotics/fuse/issues/184>`_)
* Add latch param to serialized publisher (`#165 <https://github.com/locusrobotics/fuse/issues/165>`_)
* Use transaction stamp in SerializedPublisher (`#147 <https://github.com/locusrobotics/fuse/issues/147>`_)
  By using the transaction stamp instead of `ros::Time::now()` it's
  possible to replay things with the same transaction and compare the
  original and new generated graphs.
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* [RST-2149] Added the configured device_id to the log message (`#110 <https://github.com/locusrobotics/fuse/issues/110>`_)
* [RST-2427] Added a 'source' field to the constraints. This is an API-breaking change. (`#101 <https://github.com/locusrobotics/fuse/issues/101>`_)
* [RST-2340] Add serialization support to fuse (`#98 <https://github.com/locusrobotics/fuse/issues/98>`_)
* [RST-2148] Added start() and stop() methods to the MotionModel, SensorModel, and Publisher API (`#75 <https://github.com/locusrobotics/fuse/issues/75>`_)
  * Added start() and stop() methods to the MotionModel, SensorModel, and Publisher API
  * Added the ability to clear the callback queue of the optimizer
  * Refactor the fixed-lag reset callback to use the plugins' stop() and start() methods
* Fix -Wall -Wextra warnings (`#77 <https://github.com/locusrobotics/fuse/issues/77>`_)
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams, Tom Moore

0.4.0 (2019-07-12)
------------------
* [RST-1747] fixed lag smoother implementation (`#52 <https://github.com/locusrobotics/fuse/issues/52>`_)
* [RST-1926] Extend the local parameter definition to include Minus() (`#40 <https://github.com/locusrobotics/fuse/issues/40>`_)
* Contributors: Stephen Williams

0.3.0 (2019-03-18)
------------------
* [RST-1625] Created a StampedVariableSynchronizer helper class (`#39 <https://github.com/locusrobotics/fuse/issues/39>`_)
* [RST-1653] transaction stamps (`#37 <https://github.com/locusrobotics/fuse/issues/37>`_)
  * Moved the set<ros::Time> object that always accompanies a Transaction into the Transaction itself.
  * Updated all related classes to support that change
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
* [RST-1121] Moved the pose publishers (`#19 <https://github.com/locusrobotics/fuse/issues/19>`_)
  * Moved the publisher base classes to the public repo
  * Moved the pose publisher implementations to the public repo
  * Added the option to publish the robot trajectory as a PoseArray message
  * Clean up Eigen depends and includes
* Contributors: Stephen Williams

0.0.2 (2018-07-16)
------------------

0.0.1 (2018-07-05)
------------------
