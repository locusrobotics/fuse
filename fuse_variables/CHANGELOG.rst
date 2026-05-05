^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.5 (2026-05-05)
------------------

1.2.4 (2025-07-28)
------------------
* Porting StampedVariableSynchronizer changes to ROS 2 (`#414 <https://github.com/locusrobotics/fuse/issues/414>`_)
  * Porting effort to ROS 2
  * Porting this functionality to ROS 2
  * Responding to comments
* [RST-12952] Modified the loadDeviceId() function to accept optional parameters (`#409 <https://github.com/locusrobotics/fuse/issues/409>`_)
  * Modified the loadDeviceId() function to accept optional parameter names for the device UUID and device name
  * Use the logging interface from the node
* Contributors: David Murdoch, Stephen Williams

1.2.3 (2025-05-24)
------------------

1.2.2 (2025-04-26)
------------------
* * Added dependencies in required CMakeLists.txt and package.xml files
  * Added ament_cmake_ros and gtest_vendor dependencies
  * Removed duplicate package depends, alphabetized lists
  See https://www.linkedin.com/posts/open-source-robotics-foundation_were-looking-for-half-a-dozen-new-open-activity-7317690134764605440-jm3h/
  Author: KB1110 <kartikbakshi10@gmail.com>
* Port landmark cleanup from ROS1 to ROS2 (`#330 <https://github.com/locusrobotics/fuse/issues/330>`_)
  Port landmark cleanup from ROS1 to ROS2 (`#259 <https://github.com/locusrobotics/fuse/issues/259>`_)
* [RST-7809] Port fix for negative pi initial conditions from ROS1 to ROS2 (`#335 <https://github.com/locusrobotics/fuse/issues/335>`_)
  * Add some unit tests for the 2D orientation constraints; Create getters/setters for the 2D orientation variable is preparation for a fix.
  * Force the 2D orientation value to be is minimum phase
* Port patch `#385 <https://github.com/locusrobotics/fuse/issues/385>`_ to ROS 2 Rolling (`#403 <https://github.com/locusrobotics/fuse/issues/403>`_)
  Update orientation_3d_stamped.h (`#385 <https://github.com/locusrobotics/fuse/issues/385>`_)
  * Update orientation_3d_stamped.h description of Orientation3DManifold
  authored-by: Jake McLaughlin <jmclaughlin@ottomotors.com>
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
* fuse -> ROS 2 fuse_publishers: Port fuse_publishers (`#299 <https://github.com/locusrobotics/fuse/issues/299>`_)
  Co-authored-by: Shane Loretz <shane.loretz@gmail.com>
* ROS 2 port of fuse_viz (`#301 <https://github.com/locusrobotics/fuse/issues/301>`_)
  * Initial attempt at porting fuse_viz
  * Make sure suitesparse can be found downstream
  * Plugin library names can't have lib/ prefix
  * Plugin library names can't have lib/ prefix
  * Add lint tests (they currently fail)
* fuse -> ROS 2 fuse_variables: Linting (`#296 <https://github.com/locusrobotics/fuse/issues/296>`_)
  * Migrate to .hpp files
  * Create redirection headers
  * Make xmllint and uncrustify happy
  * Wrap most comment lines
  * Satisfy cpplint
* fuse -> ROS 2 fuse_variables: Port fuse_variables (`#288 <https://github.com/locusrobotics/fuse/issues/288>`_)
* fuse -> ROS 2 fuse_core: Linting (`#292 <https://github.com/locusrobotics/fuse/issues/292>`_)
* fuse -> ROS 2 : Port Time (`#283 <https://github.com/locusrobotics/fuse/issues/283>`_)
* fuse -> ROS 2: Clean up macro usage warnings (`#280 <https://github.com/locusrobotics/fuse/issues/280>`_)
* fuse -> ROS 2 fuse_msgs : Port package and ignore unported packages for now (`#277 <https://github.com/locusrobotics/fuse/issues/277>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* [RST-4186] Fix fuse macro names (`#263 <https://github.com/locusrobotics/fuse/issues/263>`_)
  * Namespace all macros with the FUSE\_ prefix. Mark original macros as deprecated.
  * Update all fuse objects to use the new macro names
* Make 2D versions of the landmark variables (`#250 <https://github.com/locusrobotics/fuse/issues/250>`_)
* [RST-4390] Allow variables to be held constant during optimization (`#243 <https://github.com/locusrobotics/fuse/issues/243>`_)
  * Add support for holding variables constant
  * Create a 'fixed' landmark
  * Added initial support for marginalizing constant variables
* Adding doxygen to all packages (`#241 <https://github.com/locusrobotics/fuse/issues/241>`_)
* Add unstamped 3D point variable (`#233 <https://github.com/locusrobotics/fuse/issues/233>`_) (`#239 <https://github.com/locusrobotics/fuse/issues/239>`_)
  * Add unstamped 3D landmark variable
  * Add landmark test and new uuid generator
  Co-authored-by: Stephen Williams <swilliams@locusrobotics.com>
  Co-authored-by: Jake McLaughlin <jake.mclaughlin98@gmail.com>
* Contributors: Shane Loretz, Stephen Williams, Tom Moore, methylDragon

0.4.2 (2021-07-20)
------------------
* Adding roslint dependency to fuse_viz (`#231 <https://github.com/locusrobotics/fuse/issues/231>`_)
  * Adding roslint dependency to fuse_viz
  * Silence CMP0048 warnings
* Contributors: Tom Moore

0.4.1 (2021-07-13)
------------------
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* [RST-2340] Add serialization support to fuse (`#98 <https://github.com/locusrobotics/fuse/issues/98>`_)
* Fix -Wall -Wextra warnings in tests (`#80 <https://github.com/locusrobotics/fuse/issues/80>`_)
* Fix -Wall -Wextra warnings (`#77 <https://github.com/locusrobotics/fuse/issues/77>`_)
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams

0.4.0 (2019-07-12)
------------------
* Depend on libceres-dev instead of ceres-solver (`#71 <https://github.com/locusrobotics/fuse/issues/71>`_)
* Modified Variable class to make the UUID immutable (`#55 <https://github.com/locusrobotics/fuse/issues/55>`_)
* The node handle in the loadDeviceId() function does not need to be mutable (`#56 <https://github.com/locusrobotics/fuse/issues/56>`_)
* Fixed missing header. It was moved to a different package. (`#49 <https://github.com/locusrobotics/fuse/issues/49>`_)
* [RST-1940] Added a localSize() method to the Variable class (`#42 <https://github.com/locusrobotics/fuse/issues/42>`_)
* [RST-1927] Update the local parameterization for the orientation variables (`#41 <https://github.com/locusrobotics/fuse/issues/41>`_)
* [RST-1926] Extend the local parameter definition to include Minus() (`#40 <https://github.com/locusrobotics/fuse/issues/40>`_)
* Contributors: Enrique Fernández Perdomo, Stephen Williams

0.3.0 (2019-03-18)
------------------

0.2.0 (2019-01-16)
------------------
* [RST-1554] test depends (`#30 <https://github.com/locusrobotics/fuse/issues/30>`_)
  * Refactored all CMakeLists.txt to avoid path issues when using workspace overlays
* Contributors: Stephen Williams

0.1.1 (2018-08-15)
------------------
* [RST-1121] Load device id from parameter server (`#26 <https://github.com/locusrobotics/fuse/issues/26>`_)
  * Made a load function for getting the device id from the parameter server
* Contributors: Stephen Williams

0.1.0 (2018-08-12)
------------------
* [RST-1121] move optimizers (`#25 <https://github.com/locusrobotics/fuse/issues/25>`_)
* [RST-1121] Moved the pose publishers (`#19 <https://github.com/locusrobotics/fuse/issues/19>`_)
* Adding absolute 3d pose
* Adding 3D orientation constraints
* Contributors: Stephen Williams, Tom Moore

0.0.2 (2018-07-16)
------------------
* Added a base class for all stamped variables (`#14 <https://github.com/locusrobotics/fuse/issues/14>`_)
* Added 3D variable types
* Removed fuse_variables default constructors (`#5 <https://github.com/locusrobotics/fuse/issues/5>`_)
* Move the fuse_variables package into the public repo (`#4 <https://github.com/locusrobotics/fuse/issues/4>`_)
* Contributors: Stephen Williams, Tom Moore

0.0.1 (2018-07-05)
------------------
