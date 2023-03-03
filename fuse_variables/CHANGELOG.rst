^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
