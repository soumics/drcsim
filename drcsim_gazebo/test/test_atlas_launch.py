#!/usr/bin/env python3
#
# Copyright 2026 Open Source Robotics Foundation
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

"""
Prove atlas.launch.py wires real PID gains into AtlasPlugin, end to end.

This is the collapse-under-zero-gain symptom that motivated this whole
Tier 3 branch (see CLAUDE.md), now automated.

This deliberately does not check world-frame pelvis height (no
world->robot localization exists in this architecture -- robot_state_
publisher only knows the robot's own joint-driven TF tree, not where
VRCPlugin spawned it in the world). Instead: with no atlas_command
publisher running, AtlasPlugin's own default position target is 0.0 for
every joint (ZeroAtlasCommand()). If the real gains from
config/atlas_v5_gains.yaml made it through, every joint should hold near
that 0.0 target against gravity; if DRCSIM_ROS_PARAMS_FILE/
RosNodeOptionsFromEnv() were broken (gains still zero), heavily-loaded
joints (hips, knees) would droop well past this test's tolerance within
a few seconds.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@pytest.mark.launch_test
def generate_test_description():
    # atlas.launch.py's own generate_launch_description() already builds
    # everything (gz-sim, the params file, robot_state_publisher, the
    # clock bridge) -- include it directly rather than duplicating it.
    atlas_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drcsim_gazebo'), 'launch', 'atlas.launch.py')),
        # No display in a launch_testing/CI environment -- server-only.
        launch_arguments={'headless': 'true'}.items())

    return launch.LaunchDescription([
        atlas_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestAtlasStandsUnderRealGains(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_atlas_launch')
        self.joint_positions = {}

        def on_joint_states(msg: JointState):
            for name, position in zip(msg.name, msg.position):
                self.joint_positions[name] = position

        self.sub = self.node.create_subscription(
            JointState, '/atlas/joint_states', on_joint_states, 10)

    def tearDown(self):
        self.node.destroy_node()

    def test_joints_hold_near_zero_target_under_real_gains(self):
        # Generous timeout: gz-sim startup, ROS graph discovery, and
        # VRCPlugin's own startup-sequence state machine (pin -> bdi_stand
        # -> unpin) all have to complete first.
        end_time = self.node.get_clock().now().nanoseconds + 60 * 1_000_000_000
        max_deviation = 0.0
        saw_joint_states = False
        while self.node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.joint_positions:
                saw_joint_states = True
                max_deviation = max(
                    max_deviation, max(abs(p) for p in self.joint_positions.values()))

        self.assertTrue(saw_joint_states, 'never received /atlas/joint_states')
        # 0.5 rad (~29 deg) is well beyond PID tracking error under real
        # gains holding a 0.0 target, but far short of where a
        # zero-gain/gravity-collapsed joint would end up.
        self.assertLess(
            max_deviation, 0.5,
            f'a joint drifted {max_deviation:.3f} rad from its 0.0 target -- '
            'gains likely did not reach AtlasPlugin (DRCSIM_ROS_PARAMS_FILE?)')


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
