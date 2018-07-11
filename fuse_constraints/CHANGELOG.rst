^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2018-07-05)
------------------
* [RST-1118] refactor 2d constraints (`#102 <https://github.com/locusrobotics/fuse_locus/issues/102>`_)
  * Added new constraints to support the new variable types. The old constraints will be deleted in a future PR.
  * Clean up headers and comments
  * Created generic relative constraints
* Contributors: Stephen Williams

0.3.0 (2018-06-30)
------------------

0.2.0 (2018-04-16)
------------------
* [RST-734] graph refactor `#1 <https://github.com/locusrobotics/fuse/issues/1>`_ (`#55 <https://github.com/locusrobotics/fuse/issues/55>`_)
  * Added a publisher that converts laserscans and optimized poses into a 2D occupancy map
  * Added a relative pose sensor for adding constraints from m3rsm (or anything else that publishes RelativePoseWithCovarianceStamped messages)
  * Added a typedef for UUIDs
  * Added a nil uuid constant
  * Added UUID generator functions to the fuse_ros namespace
  * Fixed unit tests
* Contributors: Stephen Williams

0.1.2 (2018-02-14)
------------------

0.1.1 (2018-02-14)
------------------

0.1.0 (2018-02-14)
------------------
* Changed template argument to be CamelCase
* Renamed the template implementation files XXX_impl.h
* Modified the constraints to be type-safe with the intended variable type.
* Added absolute pose and relative pose constraint types
* Contributors: Stephen Williams
