^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_graphs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
