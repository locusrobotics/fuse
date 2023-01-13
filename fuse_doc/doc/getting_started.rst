.. _getting_started:

Tutorial: Getting Started
#########################

In this tutorial, we will show users how to configure `fuse` to combine sensor data from multiple sensors and produce a state estimate.

To start, you'll need a launch file, a yaml configuration file, and the supporting bag file and rviz display for this tutorial:

.. code-block:: bash

  mkdir ~/fuse_tutorials
  cd ~/fuse_tutorials
  wget https://github.com/locusrobotics/fuse/raw/rolling/fuse_tutorials/data/turtlebot3.bag
  md5sum turtlebot3.bag # Should return e13723bcce819036734bc76b657c7aaf
  wget https://raw.githubusercontent.com/locusrobotics/fuse/rolling/fuse_tutorials/config/range_sensor_tutorial.rviz
  touch fuse_simple_tutorial.launch.py
  touch fuse_simple_tutorial.yaml

Launch File
***********

The core state estimation node in fuse is known as the `fixed_lag_smoother_node`. We'll begin by adding it to our `fuse_simple_tutorial.launch.py` file, along with the `rosbag2 play` and `rviz2` nodes:

.. code-block:: python

  from launch_ros.actions import SetParameter, Node
  from launch_ros.substitutions import FindPackageShare

  from launch import LaunchDescription
  from launch.actions import ExecuteProcess
  from launch.substitutions import PathJoinSubstitution

  import os

  def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play',
                 PathJoinSubstitution([os.getcwd(), 'turtlebot3.bag']),
                 '--clock', '-l', '-d', '3'],
            output='screen'
        ),

        SetParameter(name='use_sim_time', value=True),
        Node(
            package='fuse_optimizers',
            executable='fixed_lag_smoother_node',
            name='state_estimator',
            parameters=[PathJoinSubstitution([
                os.getcwd(), 'fuse_simple_tutorial.yaml'
            ])]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=[
                '-d', [PathJoinSubstitution([os.getcwd(), 'fuse_simple_tutorial.rviz'])]
            ]
        )
    ])

Basic Configuration
*******************

Our initial configuration will simply fuse a single wheel odometry sensor. Open up `fuse_simple_tutorial.yaml` and paste this into it:

.. code-block:: yaml

  state_estimator:
    ros__parameters:
      # parameters for the fixed-lag optimiser
      optimization_frequency: 20.0
      transaction_timeout: 0.01
      lag_duration: 0.5

      # parameters for the model loader (handled by the optimizer base-class)
      # ROS2 requires forward-declaration of parameters,
      # so mention the names of plugins you want to load first

      # list of motion models to load config for
      motion_models:
        # a list of human-readable names for the models
        motion_model_list:
          - unicycle_motion_model

        # model-loader config for motion models mentioned above
        # this specifies the state variables Fuse will attempt to estimate
        # these motion models load their own config as nodes below
        unicycle_motion_model:
          type: fuse_models::Unicycle2D

      # list of sensor models to load config for
      sensor_models:
        # a list of human-readable names for the models
        sensor_model_list:
          - initial_localization_sensor
          - odometry_sensor

        # model-loader config for sensor models mentioned above
        # this specifies which constraints will be generated
        # and how the constraints will link to the state variables
        # these sensor models load their own config as nodes below
        initial_localization_sensor:
          type: fuse_models::Unicycle2DIgnition
          motion_models: [unicycle_motion_model]
          ignition: true
        odometry_sensor:
          type: fuse_models::Odometry2D
          motion_models: [unicycle_motion_model]

      # list of state estimation publishers to load config for
      publishers:
        # a list of human-readable names for the publishers
        publisher_list:
          - filtered_publisher

        # model-loader config for state estimation publishers mentioned above
        # these publishers load their own config as nodes below
        filtered_publisher:
          type: fuse_models::Odometry2DPublisher


      # Motion Models
      unicycle_motion_model:
        #                         x      y      yaw    vx     vy     vyaw   ax   ay
        process_noise_diagonal: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.1, 0.1]

      # Sensor Models
      initial_localization_sensor:
        publish_on_startup: true
        #                x      y      yaw    vx     vy     vyaw    ax     ay
        initial_state: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
        initial_sigma: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100]

      odometry_sensor:
        topic: 'odom'
        twist_target_frame: 'base_footprint'
        linear_velocity_dimensions: ['x', 'y']
        angular_velocity_dimensions: ['yaw']

      # Publishers
      filtered_publisher:
        topic: 'odom_filtered'
        base_link_frame_id: 'base_footprint'
        odom_frame_id: 'odom'
        map_frame_id: 'map'
        world_frame_id: 'odom'
        publish_tf: true
        publish_frequency: 10.0

There's a lot to unpack here, so we'll look at one section at a time.

.. code-block:: yaml

  optimization_frequency: 20.0
  transaction_timeout: 0.01
  lag_duration: 0.5


In this section, we specify the `optimization_frequency`, which is the how often we run our solver and produce a state estimate (technically, it is the frequency with which all variables in the graph are updated).

We also specify the `transaction_timeout`, which specifies how long we wait for motion models to be generated when adding constraints to the graph. If this time is exceeded, the constraints are not added to the graph.

The `lag_duration` parameter specifies the length of the smoothing window. Variables added to the fixed-lag smoother will stay in the graph for at least `lag_duration` seconds. After that time, old variables are removed/marginalized out.

.. code-block:: yaml

  # list of motion models to load config for
  motion_models:
    # a list of human-readable names for the models
    motion_model_list:
      - unicycle_motion_model

    unicycle_motion_model:
      type: fuse_models::Unicycle2D

  unicycle_motion_model:
    #                         x      y      yaw    vx     vy     vyaw   ax   ay
    process_noise_diagonal: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.1, 0.1]

This section specifies the motion (kinematic) model that we will use in this problem. As our robot is a differential-drive bot, we use a 2D unicycle model. Note that `fuse` supports multiple motion models to be used, but most applications will only require one.

All motion models that are meant to be used must be listed in the `motion_model_list` parameter.

The motion model will be used to add constraints to the graph between sensor measurements. The model we have specified is of type `fuse_models::Unicycle2D`, which is a plugin with its own parameters. Those parameters are specified in the next block.

The `process_noise_diagonal` specifies the error growth for each of our state variables when we apply the kinematic model. This is equivalent to the process noise covariance you might see in an EKF application. Here, we just specify the diagonals for that matrix.

.. code-block:: yaml

  sensor_models:
    # a list of human-readable names for the models
    sensor_model_list:
      - initial_localization_sensor
      - odometry_sensor

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
All sensor models that are meant to be used must be listed in the `sensor_model_list` parameter.

The first is an "ignition" model of type `fuse_models::Unicycle2DIgnition`. It is responsible for adding a constraint to our graph for the robot's initial pose.
  - The `publish_on_startup` parameter will cause it to add a constraint to the graph as soon as it initializes
  - The `initial_state` and `initial_sigma` provide the starting state and covariance diagonal values

The second sensor model is of type `fuse_models::Odometry2D`. This particular sensor model takes in ROS `nav_msgs/Odometry` messages and creates graph constraints from them.
  - The `topic` parameter is the ROS topic on which to listen for ROS `nav_msgs/Odometry` messages.
  - The `twist_target_frame` is the frame into which we want to transform the twist (velocity) data in the incoming message. In this case, we want to transform it into the *base_footprint* frame.
  - The `fuse_models::Odometry2D` model allows users to specify which dimensions should be fused into the state estimate. In this case, we are fusing `x` velocity, `y` velocity, and `yaw` velocity.

.. code-block:: yaml

  publishers:
    # a list of human-readable names for the publishers
    publisher_list:
      - filtered_publisher

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

All publishers that are meant to be used must be listed in the `publisher_list` parameter.

Try running the launch file:

.. code-block:: bash

  cd ~/fuse_tutorials
  ros2 launch fuse_simple_tutorial.launch.py


You should see the state estimate output. The covariance display for the output `odom_filtered` topic is not enabled by default.

Adding a Second Sensor
**********************

The example so far fuses only a single odometry source, which isn't especially useful. In order to benefit from actual sensor fusion, we should add another sensor. In this case, we will add an IMU. We will augment our existing configuration.

.. code-block:: yaml

  state_estimator:
    ros__parameters:
      # parameters for the fixed-lag optimiser
      optimization_frequency: 20.0
      transaction_timeout: 0.01
      lag_duration: 0.5

      # parameters for the model loader (handled by the optimizer base-class)
      # ROS2 requires forward-declaration of parameters,
      # so mention the names of plugins you want to load first

      # list of motion models to load config for
      motion_models:
        # a list of human-readable names for the models
        motion_model_list:
          - unicycle_motion_model

        # model-loader config for motion models mentioned above
        # this specifies the state variables Fuse will attempt to estimate
        # these motion models load their own config as nodes below
        unicycle_motion_model:
          type: fuse_models::Unicycle2D

      # list of sensor models to load config for
      sensor_models:
        # a list of human-readable names for the models
        sensor_model_list:
          - initial_localization_sensor
          - odometry_sensor
          - imu_sensor

        # model-loader config for sensor models mentioned above
        # this specifies which constraints will be generated
        # and how the constraints will link to the state variables
        # these sensor models load their own config as nodes below
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

      # list of state estimation publishers to load config for
      publishers:
        # a list of human-readable names for the publishers
        publisher_list:
          - filtered_publisher

        # model-loader config for state estimation publishers mentioned above
        # these publishers load their own config as nodes below
        filtered_publisher:
          type: fuse_models::Odometry2DPublisher


      # Motion Models
      unicycle_motion_model:
        #                         x      y      yaw    vx     vy     vyaw   ax   ay
        process_noise_diagonal: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.1, 0.1]

      # Sensor Models
      initial_localization_sensor:
        publish_on_startup: true
        #                x      y      yaw    vx     vy     vyaw    ax     ay
        initial_state: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
        initial_sigma: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100]

      odometry_sensor:
        topic: 'odom'
        twist_target_frame: 'base_footprint'
        linear_velocity_dimensions: ['x', 'y']
        angular_velocity_dimensions: ['yaw']

      imu_sensor:
        topic: 'imu'
        twist_target_frame: 'base_footprint'
        angular_velocity_dimensions: ['yaw']

      # Publishers
      filtered_publisher:
        topic: 'odom_filtered'
        base_link_frame_id: 'base_footprint'
        odom_frame_id: 'odom'
        map_frame_id: 'map'
        world_frame_id: 'odom'
        publish_tf: true
        publish_frequency: 10.0

Note that we have added an `imu_sensor` section to `sensor_models`, and then specified the parameters for that new model.

- The `topic` specifies the topic on which to listen for the `sensor_msgs/IMU` IMU data.
- As with the odometry model, we can specify which state dimensions we want to fuse from this sensor. In this case, we want to fuse yaw velocity.
- Also in keeping with the odometry model, we specify a `twist_target_frame` into which the incoming data must be transformed before being fused.

Now running the launch file again:

.. code-block:: bash

  cd ~/fuse_tutorials
  ros2 launch fuse_simple_tutorial.launch.py
