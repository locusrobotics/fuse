#! /usr/bin/env python3

# Copyright 2024 PickNik Robotics
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

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = FindPackageShare("fuse_tutorials")

    return LaunchDescription(
        [
            # tell tf2 that map is the same as odom
            # without this, visualization won't work as we have no reference
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),
            # run our simulator
            Node(
                package="fuse_tutorials",
                executable="three_dimensional_simulator",
                name="three_dimensional_simulator",
                output="screen",
            ),
            # run our estimator
            Node(
                package="fuse_optimizers",
                executable="fixed_lag_smoother_node",
                name="state_estimator",
                parameters=[
                    PathJoinSubstitution([pkg_dir, "config", "fuse_3d_tutorial.yaml"])
                ],
            ),
            # run visualization
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "-d",
                    [
                        PathJoinSubstitution(
                            [pkg_dir, "config", "fuse_3d_tutorial.rviz"]
                        )
                    ],
                ],
            ),
        ]
    )
