^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2021-07-13)
------------------
* Changelogs
* Adding QT5 dependency in package.xml (`#229 <https://github.com/locusrobotics/fuse/issues/229>`_)
  * Adding QT5 dependency in package.xml
  * Fixing viz build issues in noetic (`#230 <https://github.com/locusrobotics/fuse/issues/230>`_)
* Fixed compile error when using the latest version of rviz (`#220 <https://github.com/locusrobotics/fuse/issues/220>`_)
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Replace ignition_sensors list param with ignition field (`#163 <https://github.com/locusrobotics/fuse/issues/163>`_)
  * Remove ignition_sensors param and use a per-sensor ignition field
* Keep constraint properties sorted by source (`#161 <https://github.com/locusrobotics/fuse/issues/161>`_)
* Visualize loss (`#144 <https://github.com/locusrobotics/fuse/issues/144>`_)
  * Visualize error scaled by loss cost factor
  If a constraint has a loss function, the cost is computed with and
  without loss, and the quotient of the cost with loss by the cost without
  loss is used to scale the error line. This is used to draw a "loss"
  error line, so we can see the contribution of the loss function to the
  constraint.
* Set QT_INCLUDE_DIRS and QT_LIBRARIES properly (`#153 <https://github.com/locusrobotics/fuse/issues/153>`_)
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* Draw RelativePose2DStampedConstraint constraints (`#107 <https://github.com/locusrobotics/fuse/issues/107>`_)
  * Draw RelativePose2DStampedConstraint constraints
  * Dynamically generate display properties for the constraint sources
  * Cache the constraint sources properties config so it's applied when
  the properties are later created
  * Create Variable visual + property, as we do for the Constraint
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* Add fuse_viz pkg with rviz SerializedGraph display (`#99 <https://github.com/locusrobotics/fuse/issues/99>`_)
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams, Tom Moore

* Adding QT5 dependency in package.xml (`#229 <https://github.com/locusrobotics/fuse/issues/229>`_)
  * Adding QT5 dependency in package.xml
  * Fixing viz build issues in noetic (`#230 <https://github.com/locusrobotics/fuse/issues/230>`_)
* Fixed compile error when using the latest version of rviz (`#220 <https://github.com/locusrobotics/fuse/issues/220>`_)
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Replace ignition_sensors list param with ignition field (`#163 <https://github.com/locusrobotics/fuse/issues/163>`_)
  * Remove ignition_sensors param and use a per-sensor ignition field
* Keep constraint properties sorted by source (`#161 <https://github.com/locusrobotics/fuse/issues/161>`_)
* Visualize loss (`#144 <https://github.com/locusrobotics/fuse/issues/144>`_)
  * Visualize error scaled by loss cost factor
  If a constraint has a loss function, the cost is computed with and
  without loss, and the quotient of the cost with loss by the cost without
  loss is used to scale the error line. This is used to draw a "loss"
  error line, so we can see the contribution of the loss function to the
  constraint.
* Set QT_INCLUDE_DIRS and QT_LIBRARIES properly (`#153 <https://github.com/locusrobotics/fuse/issues/153>`_)
* Removed the explicit '-std=c++14' compile flag (`#119 <https://github.com/locusrobotics/fuse/issues/119>`_)
  * Removed the explicit '-std=c++14' compile flag
  * Changed the CXX_STANDARD setting to be per-target instead of global
  * Added the CXX_STANDARD_REQUIRED setting to all targets
* Draw RelativePose2DStampedConstraint constraints (`#107 <https://github.com/locusrobotics/fuse/issues/107>`_)
  * Draw RelativePose2DStampedConstraint constraints
  * Dynamically generate display properties for the constraint sources
  * Cache the constraint sources properties config so it's applied when
  the properties are later created
  * Create Variable visual + property, as we do for the Constraint
* fix compilation in Kinetic (`#112 <https://github.com/locusrobotics/fuse/issues/112>`_)
* Add fuse_viz pkg with rviz SerializedGraph display (`#99 <https://github.com/locusrobotics/fuse/issues/99>`_)
* Contributors: Davide Faconti, Enrique Fernandez Perdomo, Stephen Williams, Tom Moore

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
