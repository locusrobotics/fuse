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

import os

import unittest

from ament_index_python.packages import get_package_prefix

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess
from launch.events import Shutdown
from launch_testing.actions import GTest, ReadyToTest

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util
import launch_testing.markers
import launch_testing_ros
from launch_ros.substitutions import FindPackageShare

import pytest

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    ls = LaunchContext()

    param_path = os.path.join(
        FindPackageShare('fuse_variables').perform(ls),
        'test',
        'launch_tests',
        'test_load_device_id.yaml'
    )

    test_path = os.path.join(
        get_package_prefix('fuse_variables'),
        '..', '..', 'build', 'fuse_variables', 'test', 'test_load_device_id'
    )

    cmd = [test_path, '--ros-args', '--params-file', param_path]
    test_load_device_id_process = ExecuteProcess(cmd=cmd, shell=True, output='both')

    ld = LaunchDescription([
        test_load_device_id_process,
        ReadyToTest()
    ])

    return ld, {'test_load_device_id_process': test_load_device_id_process}


class TestNoFailedTests(unittest.TestCase):
    def test_no_failed_gtests(self, proc_output):
        proc_output.assertWaitFor('[  PASSED  ] 1 test.', timeout=10, stream='stdout')
