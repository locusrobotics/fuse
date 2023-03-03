^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-03-03)
------------------
* Fixing rviz2 dependency (`#320 <https://github.com/locusrobotics/fuse/issues/320>`_)
* Contributors: Tom Moore

1.0.0 (2023-03-03)
------------------
* Use upstream rclcpp::node_interfaces::NodeInterfaces (`#313 <https://github.com/locusrobotics/fuse/issues/313>`_)
  * Use upstream rclcpp::node_interfaces::NodeInterfaces
  * Dereference node arguments to NodeInterfaces
  ---------
  Co-authored-by: methylDragon <methylDragon@gmail.com>
* fuse -> ROS 2 fuse_tutorials Linting (`#317 <https://github.com/locusrobotics/fuse/issues/317>`_)
  * Migrate headers
  * Uncrustify
  * Nitpick
  * cpplint
  ---------
* fuse -> ROS 2 : Doc Generation (`#278 <https://github.com/locusrobotics/fuse/issues/278>`_)
  * Port doc generation and fix package.xml for linting
  * Fix small bugs in package.xml
  * Use default rosdoc2 settings
  * Use default rosdoc2 settings
  * Update fuse_doc for rosdoc2
  ---------
  Co-authored-by: Shane Loretz <sloretz@google.com>
* fuse -> ROS 2 fuse_tutorials: Port fuse_tutorials (`#309 <https://github.com/locusrobotics/fuse/issues/309>`_)
  Co-authored-by: Shane Loretz <shane.loretz@gmail.com>
* fuse -> ROS 2 fuse_models: Port fuse_models (`#304 <https://github.com/locusrobotics/fuse/issues/304>`_)
  * Port messages
  * Port fuse_models
  * Fix alloc error and some bugs
  * Wait on result
* fuse -> ROS 2 fuse_publishers: Port fuse_publishers (`#299 <https://github.com/locusrobotics/fuse/issues/299>`_)
  Co-authored-by: Shane Loretz <shane.loretz@gmail.com>
* fuse -> ROS 2 fuse_constraints : Linting (`#298 <https://github.com/locusrobotics/fuse/issues/298>`_)
* fuse -> ROS 2 fuse_variables: Linting (`#296 <https://github.com/locusrobotics/fuse/issues/296>`_)
  * Migrate to .hpp files
  * Create redirection headers
  * Make xmllint and uncrustify happy
  * Wrap most comment lines
  * Satisfy cpplint
* fuse -> ROS 2 fuse_core: Linting (`#292 <https://github.com/locusrobotics/fuse/issues/292>`_)
* fuse -> ROS 2 : Port Time (`#283 <https://github.com/locusrobotics/fuse/issues/283>`_)
* fuse -> ROS 2 : Port Logging (`#279 <https://github.com/locusrobotics/fuse/issues/279>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* fuse -> ROS 2: Clean up macro usage warnings (`#280 <https://github.com/locusrobotics/fuse/issues/280>`_)
* fuse -> ROS 2 fuse_msgs : Port package and ignore unported packages for now (`#277 <https://github.com/locusrobotics/fuse/issues/277>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* [RST-4186] Fix fuse macro names (`#263 <https://github.com/locusrobotics/fuse/issues/263>`_)
  * Namespace all macros with the FUSE\_ prefix. Mark original macros as deprecated.
  * Update all fuse objects to use the new macro names
* Fix install space for fuse_tutorials (`#264 <https://github.com/locusrobotics/fuse/issues/264>`_)
* Added simple tutorial files from the S3 bucket (`#253 <https://github.com/locusrobotics/fuse/issues/253>`_)
* Sensor tutorial (`#251 <https://github.com/locusrobotics/fuse/issues/251>`_)
  * Create a new sensor type with a non-trivial measurement function, a new publisher to visualize the results, and a simplistic robot simulator to demonstrate the sensor in action.
* Contributors: Paul Bovbel, Shane Loretz, Stephen Williams, methylDragon

* Use upstream rclcpp::node_interfaces::NodeInterfaces (`#313 <https://github.com/locusrobotics/fuse/issues/313>`_)
  * Use upstream rclcpp::node_interfaces::NodeInterfaces
  * Dereference node arguments to NodeInterfaces
  ---------
  Co-authored-by: methylDragon <methylDragon@gmail.com>
* fuse -> ROS 2 fuse_tutorials Linting (`#317 <https://github.com/locusrobotics/fuse/issues/317>`_)
  * Migrate headers
  * Uncrustify
  * Nitpick
  * cpplint
  ---------
* fuse -> ROS 2 : Doc Generation (`#278 <https://github.com/locusrobotics/fuse/issues/278>`_)
  * Port doc generation and fix package.xml for linting
  * Fix small bugs in package.xml
  * Use default rosdoc2 settings
  * Use default rosdoc2 settings
  * Update fuse_doc for rosdoc2
  ---------
  Co-authored-by: Shane Loretz <sloretz@google.com>
* fuse -> ROS 2 fuse_tutorials: Port fuse_tutorials (`#309 <https://github.com/locusrobotics/fuse/issues/309>`_)
  Co-authored-by: Shane Loretz <shane.loretz@gmail.com>
* fuse -> ROS 2 fuse_models: Port fuse_models (`#304 <https://github.com/locusrobotics/fuse/issues/304>`_)
  * Port messages
  * Port fuse_models
  * Fix alloc error and some bugs
  * Wait on result
* fuse -> ROS 2 fuse_publishers: Port fuse_publishers (`#299 <https://github.com/locusrobotics/fuse/issues/299>`_)
  Co-authored-by: Shane Loretz <shane.loretz@gmail.com>
* fuse -> ROS 2 fuse_constraints : Linting (`#298 <https://github.com/locusrobotics/fuse/issues/298>`_)
* fuse -> ROS 2 fuse_variables: Linting (`#296 <https://github.com/locusrobotics/fuse/issues/296>`_)
  * Migrate to .hpp files
  * Create redirection headers
  * Make xmllint and uncrustify happy
  * Wrap most comment lines
  * Satisfy cpplint
* fuse -> ROS 2 fuse_core: Linting (`#292 <https://github.com/locusrobotics/fuse/issues/292>`_)
* fuse -> ROS 2 : Port Time (`#283 <https://github.com/locusrobotics/fuse/issues/283>`_)
* fuse -> ROS 2 : Port Logging (`#279 <https://github.com/locusrobotics/fuse/issues/279>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* fuse -> ROS 2: Clean up macro usage warnings (`#280 <https://github.com/locusrobotics/fuse/issues/280>`_)
* fuse -> ROS 2 fuse_msgs : Port package and ignore unported packages for now (`#277 <https://github.com/locusrobotics/fuse/issues/277>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* [RST-4186] Fix fuse macro names (`#263 <https://github.com/locusrobotics/fuse/issues/263>`_)
  * Namespace all macros with the FUSE\_ prefix. Mark original macros as deprecated.
  * Update all fuse objects to use the new macro names
* Fix install space for fuse_tutorials (`#264 <https://github.com/locusrobotics/fuse/issues/264>`_)
* Added simple tutorial files from the S3 bucket (`#253 <https://github.com/locusrobotics/fuse/issues/253>`_)
* Sensor tutorial (`#251 <https://github.com/locusrobotics/fuse/issues/251>`_)
  * Create a new sensor type with a non-trivial measurement function, a new publisher to visualize the results, and a simplistic robot simulator to demonstrate the sensor in action.
* Contributors: Paul Bovbel, Shane Loretz, Stephen Williams, methylDragon

0.4.2 (2021-07-20)
------------------

0.4.1 (2021-07-13)
------------------

0.4.0 (2019-07-12)
------------------

0.3.0 (2019-03-18)
------------------

0.2.0 (2019-01-16)
------------------

0.1.1 (2018-08-15)
------------------

0.1.0 (2018-08-12)
------------------

0.0.2 (2018-07-16)
------------------

0.0.1 (2018-07-05)
------------------
