#! /usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch_ros.actions import SetParameter, Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_dir = FindPackageShare('fuse_tutorials')

    return LaunchDescription([
        Node(package="tf2_ros",
             executable = "static_transform_publisher",
             arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        Node(
            package='fuse_tutorials',
            executable='range_sensor_simulator',
            name='range_sensor_simulator',
            output='screen'
        ),
        Node(
            package='fuse_optimizers',
            executable='fixed_lag_smoother_node',
            name='state_estimation',
            output='screen',
            parameters=[PathJoinSubstitution([
                pkg_dir, 'config', 'range_sensor_tutorial.yaml'
            ])]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=[
                '-d', [PathJoinSubstitution([pkg_dir, 'config', 'range_sensor_tutorial.rviz'])]
            ]
        )
    ])
