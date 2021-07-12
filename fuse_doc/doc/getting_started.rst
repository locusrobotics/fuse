.. _getting_started:

Tutorial: Getting Started
#########################

In this tutorial, we will show users how to configure `fuse` to combine sensor data from multiple sensors and produce a state estimate.

To start, you'll need a launch file, a yaml configuration file, and the supporting bag file and rviz display for this tutorial:

.. code-block:: bash

  mkdir ~/fuse_tutorials
  cd ~/fuse_tutorials
  wget https://locus-ros-public.s3.amazonaws.com/fuse_tutorial_data.tar.xz
  md5sum fuse_tutorial_data.tar.xz # Should return ef82dceb57ed83dd597cf4973f31a175
  tar -xf fuse_tutorial_data.tar.xz && rm fuse_tutorial_data.tar.xz
  touch fuse_simple_tutorial.launch
  touch fuse_simple_tutorial.yaml

Launch File
***********

The core state estimation node in fuse is known as the `fixed_lag_smoother_node`. We'll begin by adding it to our `fuse_simple_tutorial.launch` file, along with the `rosbag play` and `rviz` nodes:

.. code-block:: xml

  <launch>
    <param name="/use_sim_time" value="true"/>

    <node name="state_estimator" pkg="fuse_optimizers" type="fixed_lag_smoother_node">
      <rosparam command="load" file="$(env HOME)/fuse_tutorials/fuse_simple_tutorial.yaml"/>
    </node>

    <node name="bag_play" pkg="rosbag" type="play" args="$(env HOME)/fuse_tutorials/turtlebot3.bag --clock -d 3"/>

    <node name="rviz" pkg="rviz" type="rviz" args="-d $(env HOME)/fuse_tutorials/fuse_tutorials.rviz"/>
  </launch>

Basic Configuration
*******************

Our initial configuration will simply fuse a single wheel odometry sensor. Open up `fuse_simple_tutorial.yaml` and paste this into it:

.. code-block:: yaml

  # Fixed-lag smoother configuration
  optimization_frequency: 20
  transaction_timeout: 0.01
  lag_duration: 0.5

  motion_models:
    unicycle_motion_model:
      type: fuse_models::Unicycle2D

  unicycle_motion_model:
    #                         x      y      yaw    vx     vy     vyaw   ax   ay
    process_noise_diagonal: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.1, 0.1]

  sensor_models:
    initial_localization_sensor:
      type: fuse_models::Unicycle2DIgnition
      motion_models: [unicycle_motion_model]
      ignition: true
    odometry_sensor:
      type: fuse_models::Odometry2D
      motion_models: [unicycle_motion_model]

  initial_localization_sensor:
    publish_on_startup: true
    #                x      y      yaw    vx     vy     vyaw    ax     ay
    initial_state: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
    initial_sigma: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100]

  odometry_sensor:
    topic: "odom"
    twist_target_frame: "base_footprint"
    linear_velocity_dimensions: ['x', 'y']
    angular_velocity_dimensions: ['yaw']

  publishers:
    filtered_publisher:
      type: fuse_models::Odometry2DPublisher

  filtered_publisher:
    topic: "odom_filtered"
    base_link_frame_id: "base_footprint"
    odom_frame_id: "odom"
    map_frame_id: "map"
    world_frame_id: "odom"
    publish_tf: true
    publish_frequency: 10

There's a lot to unpack here, so we'll look at one section at a time.

.. code-block:: yaml

  # Fixed-lag smoother configuration
  optimization_frequency: 20
  transaction_timeout: 0.01
  lag_duration: 0.5


In this section, we specify the `optimization_frequency`, which is the how often we run our solver and produce a state estimate (technically, it is the frequency with which all variables in the graph are updated).

We also specify the `transaction_timeout`, which specifies how long we wait for motion models to be generated when adding constraints to the graph. If this time is exceeded, the constraints are not added to the graph.

The `lag_duration` parameter specifies the length of the smoothing window. Variables added to the fixed-lag smoother will stay in the graph for at least `lag_duration` seconds. After that time, old variables are removed/marginalized out.

.. code-block:: yaml

  motion_models:
    unicycle_motion_model:
      type: fuse_models::Unicycle2D

  unicycle_motion_model:
    #                         x      y      yaw    vx     vy     vyaw   ax   ay
    process_noise_diagonal: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.1, 0.1]

This section specifies the motion (kinematic) model that we will use in this problem. As our robot is a differential-drive bot, we use a 2D unicycle model. Note that `fuse` supports multiple motion models to be used, but most applications will only require one.

The motion model will be used to add constraints to the graph between sensor measurements. The model we have specified is of type `fuse_models::Unicycle2D`, which is a plugin with its own parameters. Those parameters are specified in the next block.

The `process_noise_diagonal` specifies the error growth for each of our state variables when we apply the kinematic model. This is equivalent to the process noise covariance you might see in an EKF application. Here, we just specify the diagonals for that matrix.

.. code-block:: yaml

  sensor_models:
    initial_localization_sensor:
      type: fuse_models::Unicycle2DIgnition
      motion_models: [unicycle_motion_model]
      ignition: true
    odometry_sensor:
      type: fuse_models::Odometry2D
      motion_models: [unicycle_motion_model]

  initial_localization_sensor:
    publish_on_startup: true
    #                x      y      yaw    vx     vy     vyaw    ax     ay
    initial_state: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
    initial_sigma: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100]

  odometry_sensor:
    topic: "odom"
    twist_target_frame: "base_footprint"
    linear_velocity_dimensions: ['x', 'y']
    angular_velocity_dimensions: ['yaw']

In this section, we specify two sensor models.

The first is an "ignition" model of type `fuse_models::Unicycle2DIgnition`. It is responsible for adding a constraint to our graph for the robot's initial pose.
  - The `publish_on_startup` parameter will cause it to add a constraint to the graph as soon as it initializes
  - The `initial_state` and `initial_sigma` provide the starting state and covariance diagonal values

The second sensor model is of type `fuse_models::Odometry2D`. This particular sensor model takes in ROS `nav_msgs/Odometry` messages and creates graph constraints from them.
  - The `topic` parameter is the ROS topic on which to listen for ROS `nav_msgs/Odometry` messages.
  - The `twist_target_frame` is the frame into which we want to transform the twist (velocity) data in the incoming message. In this case, we want to transform it into the *base_footprint* frame.
  - The `fuse_models::Odometry2D` model allows users to specify which dimensions should be fused into the state estimate. In this case, we are fusing `x` velocity, `y` velocity, and `yaw` velocity.

.. code-block:: yaml

  publishers:
    filtered_publisher:
      type: fuse_models::Odometry2DPublisher

  filtered_publisher:
    topic: "odom_filtered"
    base_link_frame_id: "base_footprint"
    odom_frame_id: "odom"
    map_frame_id: "map"
    world_frame_id: "odom"
    publish_tf: true
    publish_frequency: 10

Here, we configure the plugin that will publish our state estimate. The `fuse_publishers::Odometry2DPublisher` publishes a ROS `nav_msgs/Odometry` message, as well as a transform from the frame specified in the `world_frame` parameter to the frame specified in the `base_link_frame_id` parameter.

- The `topic` is the ROS topic on which the output will be published.
- The `*_frame_id` parameters specify the various coordinate frame IDs that will be used when publishing the `nav_msgs/Odometry` message.
- The `publish_tf` parameter can be used to enable or disable publishing the transform for use by `tf2`.

Try running the launch file:

.. code-block:: bash

  cd ~/fuse_tutorials
  roslaunch fuse_simple_tutorial.launch


You should see the state estimate output. The covariance display for the output `odom_filtered` topic is not enabled by default.

Adding a Second Sensor
**********************

The example so far fuses only a single odometry source, which isn't especially useful. In order to benefit from actual sensor fusion, we should add another sensor. In this case, we will add an IMU. We will augment our existing configuration.

.. code-block:: yaml

  # Fixed-lag smoother configuration
  optimization_frequency: 20
  transaction_timeout: 0.01
  lag_duration: 0.5

  motion_models:
    unicycle_motion_model:
      type: fuse_models::Unicycle2D

  unicycle_motion_model:
    #                         x      y      yaw    vx     vy     vyaw   ax   ay
    process_noise_diagonal: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.1, 0.1]

  sensor_models:
    initial_localization_sensor:
      type: fuse_models::Unicycle2DIgnition
      motion_models: [unicycle_motion_model]
      ignition: true
    odometry_sensor:
      type: fuse_models::Odometry2D
      motion_models: [unicycle_motion_model]
    imu_sensor:
      type: fuse_models::Imu2D
      motion_models: [unicycle_motion_model]

  initial_localization_sensor:
    publish_on_startup: true
    #                x      y      yaw    vx     vy     vyaw    ax     ay
    initial_state: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
    initial_sigma: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100]

  odometry_sensor:
    topic: "odom"
    twist_target_frame: "base_footprint"
    linear_velocity_dimensions: ['x', 'y']
    angular_velocity_dimensions: ['yaw']

  imu_sensor:
    topic: "imu"
    angular_velocity_dimensions: ['yaw']
    linear_acceleration_dimensions: ['x', 'y']
    twist_target_frame: "base_footprint"

  publishers:
    filtered_publisher:
      type: fuse_models::Odometry2DPublisher

  filtered_publisher:
    topic: "odom_filtered"
    base_link_frame_id: "base_footprint"
    odom_frame_id: "odom"
    map_frame_id: "map"
    world_frame_id: "odom"
    publish_tf: true
    publish_frequency: 10

Note that we have added an `imu_sensor` section to `sensor_models`, and then specified the parameters for that new model.

- The `topic` specifies the topic on which to listen for the `sensor_msgs/IMU` IMU data.
- As with the odometry model, we can specify which state dimensions we want to fuse from this sensor. In this case, we want to fuse yaw velocity.
- Also in keeping with the odometry model, we specify a `twist_target_frame` into which the incoming data must be transformed before being fused.

Now running the launch file again:

.. code-block:: bash

  cd ~/fuse_tutorials
  roslaunch fuse_simple_tutorial.launch
