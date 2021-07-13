^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_loss
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2021-07-13)
------------------
* Changelogs
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Replace ignition_sensors list param with ignition field (`#163 <https://github.com/locusrobotics/fuse/issues/163>`_)
  * Remove ignition_sensors param and use a per-sensor ignition field
* Add ComposedLoss (`#170 <https://github.com/locusrobotics/fuse/issues/170>`_)
* Patch Tukey loss for Ceres < 2.0.0 (`#159 <https://github.com/locusrobotics/fuse/issues/159>`_)
  * Patch Tukey loss for Ceres < 2.0.0
  * Create ceres_macros.h header
* Plot loss (`#143 <https://github.com/locusrobotics/fuse/issues/143>`_)
  * Add test to plot loss rho, influence and weight
  * Add BUILD_WITH_PLOT_TESTS option (defaults OFF)
* Remove Pseudo-Huber loss, it duplicates SoftLOne (`#152 <https://github.com/locusrobotics/fuse/issues/152>`_)
  * Remove Pseudo-Huber loss, it duplicates SoftLOne
* Add new loss functions (`#142 <https://github.com/locusrobotics/fuse/issues/142>`_)
  The following loss functions, not available in Ceres solver are
  provided:
  * Geman-McClure
  * DCS (Dynamic Covariance Scaling)
  * Fair
  * Pseudo-Huber
  * Welsch
* Support ScaledLoss (`#141 <https://github.com/locusrobotics/fuse/issues/141>`_)
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Contributors: Enrique Fernandez Perdomo, Stephen Williams, Tom Moore

* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Replace ignition_sensors list param with ignition field (`#163 <https://github.com/locusrobotics/fuse/issues/163>`_)
  * Remove ignition_sensors param and use a per-sensor ignition field
* Add ComposedLoss (`#170 <https://github.com/locusrobotics/fuse/issues/170>`_)
* Patch Tukey loss for Ceres < 2.0.0 (`#159 <https://github.com/locusrobotics/fuse/issues/159>`_)
  * Patch Tukey loss for Ceres < 2.0.0
  * Create ceres_macros.h header
* Plot loss (`#143 <https://github.com/locusrobotics/fuse/issues/143>`_)
  * Add test to plot loss rho, influence and weight
  * Add BUILD_WITH_PLOT_TESTS option (defaults OFF)
* Remove Pseudo-Huber loss, it duplicates SoftLOne (`#152 <https://github.com/locusrobotics/fuse/issues/152>`_)
  * Remove Pseudo-Huber loss, it duplicates SoftLOne
* Add new loss functions (`#142 <https://github.com/locusrobotics/fuse/issues/142>`_)
  The following loss functions, not available in Ceres solver are
  provided:
  * Geman-McClure
  * DCS (Dynamic Covariance Scaling)
  * Fair
  * Pseudo-Huber
  * Welsch
* Support ScaledLoss (`#141 <https://github.com/locusrobotics/fuse/issues/141>`_)
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Contributors: Enrique Fernandez Perdomo, Stephen Williams

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
