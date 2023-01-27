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
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play',
                 PathJoinSubstitution([pkg_dir, 'data', 'turtlebot3.bag']),
                 '--clock', '-l', '-d', '3'],
            output='screen'
        ),

        SetParameter(name='use_sim_time', value=True),
        Node(
            package='fuse_optimizers',
            executable='fixed_lag_smoother_node',
            name='state_estimator',
            parameters=[PathJoinSubstitution([
                pkg_dir, 'config', 'fuse_simple_tutorial.yaml'
            ])]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=[
                '-d', [PathJoinSubstitution([pkg_dir, 'config', 'fuse_simple_tutorial.rviz'])]
            ]
        )
    ])
