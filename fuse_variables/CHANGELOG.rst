^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
