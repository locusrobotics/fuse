^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
