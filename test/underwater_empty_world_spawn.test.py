# Copyright 2021 Open Rise Robotics
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

from pathlib import Path
import subprocess
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import pytest


@pytest.mark.launch_test
def generate_test_description():

    # Substitutions
    gzserver_launch_file = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'), 'launch/gzserver.launch.py'
    ])

    world_file_path = str(Path(__file__).parent / 'worlds/empty_underwater.world')

    # Launch Description
    return (
        LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gzserver_launch_file),
                launch_arguments={'server_required': 'True',
                                  'world': world_file_path}.items()
            ),
            TimerAction(period=5., actions=[ReadyToTest()])
        ])
    )


class TestUnderwaterEmptyWorldSpawn(unittest.TestCase):

    def test_node_names(self, proc_info):
        result = subprocess.run(['gz', 'topic', '-l'], capture_output=True)

        self.assertEqual(result.returncode, 0)

        topics = result.stdout.decode().split('\n')
        print(topics)

        self.assertIn('hydrodynamics/current_velocity', topics)
