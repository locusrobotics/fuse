.. _index:

.. toctree::
   :hidden:

   getting_started
   CHANGELOG

fuse wiki
*********

The fuse package brings a modern graph-based approach to ROS sensor fusion, providing a flexible and extensible solution to state estimation. fuse allows users to develop custom sensor and motion models, add additional state variables as needed for tracking visual landmarks or sensor calibration parameters, and control the length of the state history anywhere from just the most recent states to maintaining the full state history for SLAM-like applications.

Features
========

* The state vector is customizable. In fact, there is no single state vector. Measurements can add additional state as needed. This means that vision-based landmarks can be added to the state as they are observed. Or that sensor-specific calibration variables (e.g. an IMU bias) can be added and estimated along with the robot state.
* The robot state at multiple time points is available to sensor models. This means a sensor that measures a state change (e.g. scan-to-scan matches) can represent that as a measurement on the difference between two states.
* Nonlinear measurement models are linearized multiple times. Ceres Solver is an iterative nonlinear solver that will linearize each measurement during each iteration. It also allows past measurements to be relinearized as new measurements are added and the state estimate changes. This reduces linearization errors in highly nonlinear measurement models.
* The length of state history can be controlled. This can range anywhere from maintaining the previous and current state, to maintaining every state added to the system ever. This allows a single framework to be used for both live state estimation on a resource-constrained robot, to a full SLAM mapping system.
* The motion models are plugin-based. If an adequate motion model is not available, the user is able to write their own models for the specifics of their robot.
* The sensor models are also plugin-based. If an adequate sensor model is not available, the user is able to write their own.
* A new “publisher” concept is introduced that allows the output of the sensor fusion framework to be customized. Any information contained within the graph can be extracted, processed, and made available to the rest of the ROS ecosystem. This can be simple, like publishing the most recent robot pose to the tf system, or complex, like generating an environment map from every robot pose.
* All states are tagged with a robot ID, allowing a single graph to contain data from multiple robots. This enables applications like cooperative mapping.
* The plugin architecture allows users to make their custom motion models, sensor models, and publishers open-source as separate repositories, without the need to fork the main fuse repository or submit pull requests. It also allows users to keep their custom models private, if that is desired.
* Robust loss functions can be applied to any sensor model to minimize the impact of outlier measurements.
* Automatic differentiation allows users to design new sensor models without having to compute function derivatives explicitly. This is often the most difficult aspect of implementing a new mathematical model. However, analytically derived function derivatives are also supported.

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
