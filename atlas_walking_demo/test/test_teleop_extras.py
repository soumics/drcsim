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

"""Unit tests for teleop_extras.py (keys, grippers, kicks), no rclpy needed."""

import importlib.util
import pathlib

import pytest

_SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'teleop_extras.py')
_SPEC = importlib.util.spec_from_file_location('teleop_extras_script', _SCRIPT_PATH)
tx = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(tx)


def test_walk_keys_scale_by_speed_and_axis_limits():
    assert tx.walk_command('w', 0.5, 0.4, 0.3, 0.2, 0.4) == (0.2, 0.0, 0.0)
    assert tx.walk_command('s', 1.0, 0.4, 0.3, 0.2, 0.4) == (-0.3, 0.0, 0.0)
    assert tx.walk_command('a', 1.0, 0.4, 0.3, 0.2, 0.4) == (0.0, 0.2, 0.0)
    assert tx.walk_command('e', 1.0, 0.4, 0.3, 0.2, 0.4) == (0.0, 0.0, -0.4)
    assert tx.walk_command('g', 1.0, 0.4, 0.3, 0.2, 0.4) is None


def test_gripper_starts_relaxed_and_moves_between_poses_at_a_limited_rate():
    gripper = tx.Gripper()
    for hand in ('sandia', 'svh'):
        assert gripper.sample(0.0, hand) == pytest.approx(tx.HAND_POSES[hand]['relaxed'])
    gripper.toggle()
    assert gripper.pose == 'closed'
    first = gripper.sample(0.1, 'svh')
    relaxed, closed = tx.HAND_POSES['svh']['relaxed'], tx.HAND_POSES['svh']['closed']
    for value, start, goal in zip(first, relaxed, closed):
        assert abs(value - start) <= tx.HAND_SPEED * 0.1 + 1e-12
        assert min(start, goal) - 1e-12 <= value <= max(start, goal) + 1e-12
    assert gripper.sample(5.0, 'svh') == pytest.approx(closed)
    assert gripper.closure('svh') == pytest.approx(1.0)
    gripper.toggle()
    assert gripper.sample(5.0, 'svh') == pytest.approx(relaxed)
    gripper.pose = 'open'
    assert gripper.sample(5.0, 'sandia') == pytest.approx(tx.HAND_POSES['sandia']['open'])


def test_hand_poses_match_each_models_joint_list():
    for hand, names in (('sandia', tx.SANDIA_JOINT_NAMES), ('svh', tx.SVH_JOINT_NAMES)):
        for pose in ('open', 'relaxed', 'closed'):
            assert len(tx.HAND_POSES[hand][pose]) == len(names)


def test_kick_pushes_the_torso_then_clears_it():
    apply, clear = tx.kick_commands((0.0, 1.0), force=500.0)
    assert apply[:3] == ['gz', 'topic', '-t'] and apply[3].endswith('/wrench/persistent')
    assert 'force: {x: 0.0, y: 500.0, z: 0}' in apply[-1]
    assert 'atlas::utorso' in apply[-1] and 'atlas::utorso' in clear[-1]
    assert clear[3].endswith('/wrench/clear')
