^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
