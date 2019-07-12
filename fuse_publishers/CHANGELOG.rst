^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_publishers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
