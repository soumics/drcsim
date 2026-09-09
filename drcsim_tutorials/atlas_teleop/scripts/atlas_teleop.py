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
Joystick-driven whole-body pose blending for Atlas.

Blends up to 9 joystick sliders, each weighted toward a pose vector loaded
from a YAML preset file (see config/*.yaml), plus an origin pose (index 0)
and per-slider hand grasp blends. Ported from the original drcsim tutorial
of the same name; see the class-level comment below for what changed.
"""

import sys

from atlas_msgs.msg import AtlasCommand
from osrf_msgs.msg import JointCommands
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import yaml

# The exact 30-joint order AtlasPlugin::Configure() builds internally (see
# AtlasPlugin.cpp's jointNames.push_back() calls) -- AtlasCommand (unlike
# JointCommands) has no per-joint name field at all, so its position/
# k_effort/etc. arrays are matched to joints purely by index.
# config/*.yaml's per-index pose vectors are ordered to match.
ATLAS_JOINT_NAMES = [
    'back_bkz', 'back_bky', 'back_bkx', 'neck_ry',
    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
    'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx',
    'l_arm_wry2',
    'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx',
    'r_arm_wry2',
]
NUM_JOINTS = len(ATLAS_JOINT_NAMES)

# Grasp poses a slider can blend a hand toward; 'origin' is intentionally
# ignored (matching the original -- blending different grasps' origins was
# never implemented there either), only 'grasp' is used.
GRASPS = {
    'cyl': [0, 1.5, 1.7, 0, 1.5, 1.7, 0, 1.5, 1.7, -0.2, 0.8, 1.7],
    'sph': [-1.0, 1.4, 1.4, 0.0, 1.4, 1.4, 1.0, 1.4, 1.4, 0, 0.7, 0.7],
    'par': [0.0, 1.4, 1.2, 0, -1.4, -1.4, 0.0, -1.4, -1.4, 0.5, 0.7, 0.7],
}

HAND_JOINT_NAMES = [
    'f0_j0', 'f0_j1', 'f0_j2', 'f1_j0', 'f1_j1', 'f1_j2',
    'f2_j0', 'f2_j1', 'f2_j2', 'f3_j0', 'f3_j1', 'f3_j2',
]


def blend_command(pose_vectors, hand_grasps, axes):
    """
    Compute (whole_body_positions, left_hand, right_hand) for one Joy frame.

    Pure function, split out from the ROS callback so it's unit-testable
    without rclpy/a live joystick -- see test/test_blend_command.py.
    """
    position = list(pose_vectors[0])
    lh_position = [0.0] * 12
    rh_position = [0.0] * 12
    num_sliders = min(8, len(axes))
    for slider in range(1, num_sliders + 1):
        slider_pos = axes[slider - 1]
        for joint_idx in range(NUM_JOINTS):
            position[joint_idx] += slider_pos * pose_vectors[slider][joint_idx]
        lh_grasp_name, lh_scale = hand_grasps['l'][slider]
        rh_grasp_name, rh_scale = hand_grasps['r'][slider]
        for axis in range(12):
            lh_position[axis] += slider_pos * GRASPS[lh_grasp_name][axis] * lh_scale
            rh_position[axis] += slider_pos * GRASPS[rh_grasp_name][axis] * rh_scale
    return position, lh_position, rh_position


def load_pose_file(path):
    """Parse a config/*.yaml preset into (pose_vectors, hand_grasps, knobs)."""
    with open(path) as f:
        raw = yaml.safe_load(f)

    pose_vectors = [[0.0] * NUM_JOINTS for _ in range(9)]
    hand_grasps = {'l': [('cyl', 0.0)] * 9, 'r': [('cyl', 0.0)] * 9}
    knobs = raw.get('knobs', 'unknown')
    for key, value in raw.items():
        if key == 'knobs':
            continue
        idx = int(key)
        if idx < 0 or idx > 8:
            raise ValueError(f'pose index out of range 0-8: {idx}')
        tokens = value.split()
        pose_vectors[idx] = [float(x) for x in tokens[0:NUM_JOINTS]]
        hand_grasps['l'][idx] = (tokens[NUM_JOINTS], float(tokens[NUM_JOINTS + 1]))
        hand_grasps['r'][idx] = (tokens[NUM_JOINTS + 2], float(tokens[NUM_JOINTS + 3]))
    return pose_vectors, hand_grasps, knobs


class AtlasTeleop(Node):

    def __init__(self):
        super().__init__('atlas_teleop')
        pose_file = self.declare_parameter('pose_file', '').value
        if not pose_file:
            raise ValueError(
                "required parameter 'pose_file' not set -- pass "
                '--ros-args -p pose_file:=/path/to/preset.yaml')
        self.pose_vectors, self.hand_grasps, self.knobs = load_pose_file(pose_file)

        # Real per-joint gains already live in AtlasPlugin, loaded from
        # atlas_v5_gains.yaml at launch time -- AtlasCommand's kp/ki/kd/
        # effort-limit fields are only ever applied by AtlasPlugin when
        # their array length matches its own joint count (see
        # AtlasPlugin::SetAtlasCommand); leaving them empty here (unlike
        # the original, which hardcoded its own separate gain arrays) means
        # those real gains are simply left alone, not shadowed by
        # teleop-tool-local guesses.
        self.command = AtlasCommand()
        self.command.position = [0.0] * NUM_JOINTS
        self.command.k_effort = [255] * NUM_JOINTS

        self.lh_command = JointCommands()
        self.lh_command.name = list(HAND_JOINT_NAMES)
        self.rh_command = JointCommands()
        self.rh_command.name = list(HAND_JOINT_NAMES)

        self.ac_pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        # Live only if a SandiaHandPlugin-equipped variant is spawned; a
        # harmless publish-with-no-subscriber otherwise (the default
        # atlas_v5, no-hands setup this migration's launch file uses).
        self.lh_pub = self.create_publisher(
            JointCommands, 'sandia_hands/l_hand/joint_commands', 10)
        self.rh_pub = self.create_publisher(
            JointCommands, 'sandia_hands/r_hand/joint_commands', 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)

    def joy_cb(self, msg):
        position, lh_position, rh_position = blend_command(
            self.pose_vectors, self.hand_grasps, msg.axes)
        self.command.position = position
        self.lh_command.position = lh_position
        self.rh_command.position = rh_position
        self.lh_pub.publish(self.lh_command)
        self.rh_pub.publish(self.rh_command)
        self.ac_pub.publish(self.command)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AtlasTeleop()
    except ValueError as err:
        print(f'atlas_teleop: {err}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
