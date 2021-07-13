^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_graphs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Move parameter blocks vector outside loop (`#215 <https://github.com/locusrobotics/fuse/issues/215>`_)
  This reduces the amount of allocations required, since the same vector
  and its previously allocated memory is reused among all the graph
  constraints when the ceres::Problem is created.
* Add HashGraph::createProblem benchmark (`#214 <https://github.com/locusrobotics/fuse/issues/214>`_)
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Remove variables on hold (`#185 <https://github.com/locusrobotics/fuse/issues/185>`_)
  * Test variables on hold are removed when removing a variable
  * Erase variable on hold when removing variable
* Add evaluate method to graph (`#151 <https://github.com/locusrobotics/fuse/issues/151>`_)
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* [RST-2438] Make ceres params loaders reusable (`#104 <https://github.com/locusrobotics/fuse/issues/104>`_)
  * Moved the Ceres loadFromROS functions into reusable functions in fuse_core
  * Load solver parameters for the batch optimizer
* Expose Ceres Solver, Problem and Covariance Options as ROS parameters (`#78 <https://github.com/locusrobotics/fuse/issues/78>`_)
* [RST-2427] Added a 'source' field to the constraints. This is an API-breaking change. (`#101 <https://github.com/locusrobotics/fuse/issues/101>`_)
* [RST-2340] Add serialization support to fuse (`#98 <https://github.com/locusrobotics/fuse/issues/98>`_)
* Fix -Wall -Wextra warnings in tests (`#80 <https://github.com/locusrobotics/fuse/issues/80>`_)
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams

0.4.0 (2019-07-12)
------------------
* Add print method to Graph and HashGraph (`#70 <https://github.com/locusrobotics/fuse/issues/70>`_)
* Depend on libceres-dev instead of ceres-solver (`#71 <https://github.com/locusrobotics/fuse/issues/71>`_)
* [RST-2144] Support proper Eigen memory alignment (`#65 <https://github.com/locusrobotics/fuse/issues/65>`_)
* [RST-2128] Added a "reset" service to the fixed lag smoother (`#61 <https://github.com/locusrobotics/fuse/issues/61>`_)
* Modified Variable class to make the UUID immutable (`#55 <https://github.com/locusrobotics/fuse/issues/55>`_)
* [RST-1960] Added a tangent/parameter space flag to the covariance function (`#50 <https://github.com/locusrobotics/fuse/issues/50>`_)
* [RST-1949] Added getConnectedVariables() and getConnectedConstraints() (`#45 <https://github.com/locusrobotics/fuse/issues/45>`_)
* [RST-1746] Remove the marginalizeVariable() methods from the Graph class. (`#44 <https://github.com/locusrobotics/fuse/issues/44>`_)
* Contributors: Enrique Fernández Perdomo, Stephen Williams

0.3.0 (2019-03-18)
------------------
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
* [RST-1121] Moved the Graph classes (`#15 <https://github.com/locusrobotics/fuse/issues/15>`_)
  * Moved the Graph and HashGraph classes into the public repo
* Contributors: Stephen Williams

0.0.2 (2018-07-16)
------------------

0.0.1 (2018-07-05)
------------------
