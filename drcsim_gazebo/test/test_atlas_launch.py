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

Earlier versions of this test tried to infer the gains were real by
watching /atlas/joint_states stay near a 0.0 target -- that's wrong:
VRCPlugin's own startup sequence (atlas.startup_mode=bdi_stand) actively
publishes atlas_command itself, driving the robot through a real
stand-up motion ("stand prep" -> "Nominal" -> "Dynamic Stand Behavior"),
not leaving it at a fixed target. Whether that motion looks "right" is
a much harder thing to assert against than the actual thing this test
needs to prove: did DRCSIM_ROS_PARAMS_FILE/RosNodeOptionsFromEnv()
actually deliver config/atlas_v5_gains.yaml's values into the
atlas_plugin node's parameters at all. That's checked directly here,
by querying a few of them over the node's own parameter service --
completely decoupled from simulation dynamics, timing, or what a
"correct" stand-up motion should look like.
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
from rcl_interfaces.srv import GetParameters
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


# A handful of joints spanning config/atlas_v5_gains.yaml's range of
# values (small and large P gains, legs/arms/back), so a mismatched or
# silently-empty params file is very unlikely to pass by coincidence.
EXPECTED_GAINS = {
    'atlas_controller.gains.l_leg_hpz.p': 5.0,
    'atlas_controller.gains.l_leg_aky.p': 2900.0,
    'atlas_controller.gains.back_bkz.p': 5000.0,
    'atlas_controller.gains.l_arm_wry2.d': 0.1,
    'atlas_controller.gains.r_arm_shx.d': 20.0,
}


class TestAtlasPluginReceivesRealGains(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_atlas_launch')

    def tearDown(self):
        self.node.destroy_node()

    def test_atlas_plugin_node_has_real_gains(self):
        client = self.node.create_client(GetParameters, '/atlas_plugin/get_parameters')
        # Generous timeout: gz-sim startup, the plugin's own node/executor
        # startup, and ROS graph discovery all have to complete first.
        self.assertTrue(
            client.wait_for_service(timeout_sec=60.0),
            '/atlas_plugin/get_parameters service never appeared')

        request = GetParameters.Request()
        request.names = list(EXPECTED_GAINS.keys())
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), 'get_parameters call timed out')

        actual = {
            name: value.double_value
            for name, value in zip(EXPECTED_GAINS.keys(), future.result().values)
        }
        self.assertEqual(
            actual, EXPECTED_GAINS,
            'atlas_plugin is not seeing the real gains from '
            'config/atlas_v5_gains.yaml -- DRCSIM_ROS_PARAMS_FILE/'
            'RosNodeOptionsFromEnv() likely did not deliver the params file')

    def test_joint_states_are_published(self):
        # Cheap interface smoke test: AtlasPlugin is actually running its
        # control loop and publishing, not just holding a loaded params
        # file it never uses.
        received = {}

        def on_joint_states(msg: JointState):
            received['msg'] = msg

        sub = self.node.create_subscription(
            JointState, '/atlas/joint_states', on_joint_states, 10)
        end_time = self.node.get_clock().now().nanoseconds + 30 * 1_000_000_000
        while 'msg' not in received and self.node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.node.destroy_subscription(sub)

        self.assertIn('msg', received, 'never received /atlas/joint_states')
        self.assertGreater(len(received['msg'].name), 0)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # gz sim (run through its own ruby wrapper script) doesn't always
        # react to SIGINT within launch_testing's 5s grace period, so it
        # gets escalated to SIGTERM (-15) -- that's shutdown latency of a
        # slow subprocess under a forced test-harness shutdown, not a
        # functional failure, so it's allowed here alongside a clean 0.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -15])
