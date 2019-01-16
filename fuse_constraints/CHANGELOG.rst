^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2019-01-16)
------------------
* [RST-1567] Check the system has started before attempting to optimize (`#33 <https://github.com/locusrobotics/fuse/issues/33>`_)
  * Check the system has started before attempting to optimize.
  * Fixed linter issues
* RST-1559 Adding partial measurement support for relative 2D pose data (`#32 <https://github.com/locusrobotics/fuse/issues/32>`_)
  * Adding partial measurement support for relative 2D pose data
* Adding subset measurement support for AbsolutePose2DStampedConstraint (`#31 <https://github.com/locusrobotics/fuse/issues/31>`_)
  * Adding subset measurement support
* [RST-1554] test depends (`#30 <https://github.com/locusrobotics/fuse/issues/30>`_)
  * Refactored all CMakeLists.txt to avoid path issues when using workspace overlays
* RST-1239 Adding 3D relative pose constraint (`#27 <https://github.com/locusrobotics/fuse/issues/27>`_)
  * Adding 3D relative pose constraint
  * Updating expected covariance matrices for anything involving rotations
* Contributors: Stephen Williams, Tom Moore

0.1.1 (2018-08-15)
------------------

0.1.0 (2018-08-12)
------------------
* [RST-1121] Moved the pose publishers (`#19 <https://github.com/locusrobotics/fuse/issues/19>`_)
  * Clean up Eigen depends and includes
* Adding absolute 3d pose
* Fixing quaternion delta computation
* Converted all Eigen objects to use row-major order (`#22 <https://github.com/locusrobotics/fuse/issues/22>`_)
* Fixed covariance matrix size (`#21 <https://github.com/locusrobotics/fuse/issues/21>`_)
* Adding 3D orientation constraints
* Contributors: Stephen Williams, Tom Moore

0.0.2 (2018-07-16)
------------------
* fixed cut&paste error (`#13 <https://github.com/locusrobotics/fuse/issues/13>`_)
* Added absolute and relative 2D constraints (`#8 <https://github.com/locusrobotics/fuse/issues/8>`_)
* Contributors: Stephen Williams

0.0.1 (2018-07-05)
------------------
