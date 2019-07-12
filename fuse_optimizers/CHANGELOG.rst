^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_optimizers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2019-07-12)
------------------
* Wait for a valid timestamp before loading the plugins. This prevents the ignition sensor from generating an initial state with the wrong stamp. (`#74 <https://github.com/locusrobotics/fuse/issues/74>`_)
* Default private ~reset service name (`#72 <https://github.com/locusrobotics/fuse/issues/72>`_)
* [RST-2186] Added a FixedLagSmootherParams struct (`#68 <https://github.com/locusrobotics/fuse/issues/68>`_)
* [RST-2185] Fix "fixed-lag crash on reset" bug (`#66 <https://github.com/locusrobotics/fuse/issues/66>`_)
* [RST-2144] Support proper Eigen memory alignment (`#65 <https://github.com/locusrobotics/fuse/issues/65>`_)
* [RST-2158] Fix thread locking issue that was causing underconstained solver errors (`#63 <https://github.com/locusrobotics/fuse/issues/63>`_)
* [RST-2128] Added a "reset" service to the fixed lag smoother (`#61 <https://github.com/locusrobotics/fuse/issues/61>`_)
* [RST-1747] fixed lag smoother implementation (`#52 <https://github.com/locusrobotics/fuse/issues/52>`_)
* Contributors: Enrique Fernández Perdomo, Stephen Williams

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
* [RST-1567] Check the system has started before attempting to optimize (`#33 <https://github.com/locusrobotics/fuse/issues/33>`_)
  * Check the system has started before attempting to optimize.
  * Fixed linter issues
* [RST-1554] test depends (`#30 <https://github.com/locusrobotics/fuse/issues/30>`_)
  * Refactored all CMakeLists.txt to avoid path issues when using workspace overlays
* Contributors: Stephen Williams

0.1.1 (2018-08-15)
------------------

0.1.0 (2018-08-12)
------------------
* [RST-1121] move optimizers (`#25 <https://github.com/locusrobotics/fuse/issues/25>`_)
  * Moved the Optimizer and BatchOptimizer classes into the public repo
  * Added fuse_optimizers to the metapackage depends
  * Changed optimizer to unique ownership of the graph. This better captures the usage.
* Contributors: Stephen Williams

0.0.2 (2018-07-16)
------------------

0.0.1 (2018-07-05)
------------------
