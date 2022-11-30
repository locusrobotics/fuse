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

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_pytest
from launch_pytest.actions import ReadyToTest
from launch_pytest.tools import process as process_tools

import pytest


@pytest.fixture
def test_proc():
    test_root = '.'

    param_path = os.path.join(test_root, 'launch_tests', 'test_parameters.yaml')
    test_path = os.path.join(test_root, 'test_parameters')

    cmd = [test_path, '--ros-args', '--params-file', param_path]
    return ExecuteProcess(cmd=cmd, shell=True, output='screen', cached_output=True)


@launch_pytest.fixture
def generate_test_description(test_proc):
    return LaunchDescription([test_proc, ReadyToTest()])


@pytest.mark.launch(fixture=generate_test_description)
async def test_no_failed_gtests(test_proc, launch_context):
    await process_tools.wait_for_exit(launch_context, test_proc, timeout=10)
    assert test_proc.return_code == 0, "GTests failed"
