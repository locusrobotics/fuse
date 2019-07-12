^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2019-07-12)
------------------
* Depend on libceres-dev instead of ceres-solver (`#71 <https://github.com/locusrobotics/fuse/issues/71>`_)
* [RST-2144] Support proper Eigen memory alignment (`#65 <https://github.com/locusrobotics/fuse/issues/65>`_)
* [RST-2088] Fix bug causing bad marginal computations occasionally (`#60 <https://github.com/locusrobotics/fuse/issues/60>`_)
* [RST-1747] fixed lag smoother implementation (`#52 <https://github.com/locusrobotics/fuse/issues/52>`_)
* Modified Variable class to make the UUID immutable (`#55 <https://github.com/locusrobotics/fuse/issues/55>`_)
* RST-2025 Fixing size issues (`#53 <https://github.com/locusrobotics/fuse/issues/53>`_)
  * Fixing size issue with 2D poses
  * Fixed the size check in MarginalConstraint (`#54 <https://github.com/locusrobotics/fuse/issues/54>`_)
* [RST-1745] Added a marginalizeVariables() function (`#48 <https://github.com/locusrobotics/fuse/issues/48>`_)
* [RST-1745] Created a container to hold the list of constraints by variable (`#47 <https://github.com/locusrobotics/fuse/issues/47>`_)
* [RST-1745] Created a uuid<-->index bidirectional lookup class (`#46 <https://github.com/locusrobotics/fuse/issues/46>`_)
* [RST-1744] Added a marginal constraint class (`#43 <https://github.com/locusrobotics/fuse/issues/43>`_)
* [RST-1940] Added a localSize() method to the Variable class (`#42 <https://github.com/locusrobotics/fuse/issues/42>`_)
* [RST-1927] Update the local parameterization for the orientation variables (`#41 <https://github.com/locusrobotics/fuse/issues/41>`_)
* [RST-1926] Extend the local parameter definition to include Minus() (`#40 <https://github.com/locusrobotics/fuse/issues/40>`_)
* Contributors: Enrique Fernández Perdomo, Stephen Williams, Tom Moore

0.3.0 (2019-03-18)
------------------

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
