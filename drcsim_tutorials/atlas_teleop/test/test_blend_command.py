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

"""Unit tests for atlas_teleop.py's blend_command(), no rclpy node needed."""

import importlib.util
import pathlib

_SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'atlas_teleop.py')
_SPEC = importlib.util.spec_from_file_location('atlas_teleop_script', _SCRIPT_PATH)
atlas_teleop_script = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(atlas_teleop_script)


def test_zero_axes_returns_origin_pose():
    pose_vectors = [[0.0] * atlas_teleop_script.NUM_JOINTS for _ in range(9)]
    pose_vectors[0][3] = 1.5
    hand_grasps = {'l': [('cyl', 0.0)] * 9, 'r': [('cyl', 0.0)] * 9}

    position, lh, rh = atlas_teleop_script.blend_command(
        pose_vectors, hand_grasps, axes=[0.0] * 8)

    assert position == pose_vectors[0]
    assert position[3] == 1.5
    assert lh == [0.0] * 12
    assert rh == [0.0] * 12


def test_full_slider_adds_its_full_pose_vector():
    pose_vectors = [[0.0] * atlas_teleop_script.NUM_JOINTS for _ in range(9)]
    pose_vectors[1][5] = 2.0
    hand_grasps = {'l': [('cyl', 0.0)] * 9, 'r': [('cyl', 0.0)] * 9}

    position, _, _ = atlas_teleop_script.blend_command(
        pose_vectors, hand_grasps, axes=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    assert position[5] == 2.0


def test_hand_grasp_scaled_by_slider_position():
    pose_vectors = [[0.0] * atlas_teleop_script.NUM_JOINTS for _ in range(9)]
    hand_grasps = {'l': [('cyl', 0.0)] * 9, 'r': [('cyl', 0.0)] * 9}
    hand_grasps['r'][1] = ('sph', 1.0)

    _, lh, rh = atlas_teleop_script.blend_command(
        pose_vectors, hand_grasps, axes=[0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    assert lh == [0.0] * 12
    expected_rh0 = 0.5 * atlas_teleop_script.GRASPS['sph'][0]
    assert rh[0] == expected_rh0


def test_load_pose_file_round_trips_a_shipped_preset():
    config_dir = pathlib.Path(__file__).resolve().parent.parent / 'config'
    pose_vectors, hand_grasps, knobs = atlas_teleop_script.load_pose_file(
        str(config_dir / 'drive.yaml'))

    assert knobs == 'right_arm'
    assert len(pose_vectors) == 9
    assert all(len(v) == atlas_teleop_script.NUM_JOINTS for v in pose_vectors)
    # r_arm_wry2 (last joint) was the inserted-0.0 conversion value.
    assert pose_vectors[0][-1] == 0.0
