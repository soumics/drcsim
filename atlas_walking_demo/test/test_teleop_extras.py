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


def test_gripper_closes_at_a_limited_rate_and_toggles_open():
    gripper = tx.Gripper()
    gripper.toggle()
    half = gripper.sample(tx.GRIP_TIME / 2.0)
    assert half == pytest.approx([c / 2.0 for c in tx.GRASP_CLOSED])
    assert gripper.sample(tx.GRIP_TIME) == pytest.approx(tx.GRASP_CLOSED)
    gripper.toggle()
    assert gripper.sample(tx.GRIP_TIME) == pytest.approx(tx.GRASP_OPEN)
    assert len(tx.GRASP_CLOSED) == len(tx.HAND_JOINT_NAMES) == 12


def test_kick_pushes_the_torso_then_clears_it():
    apply, clear = tx.kick_commands((0.0, 1.0), force=500.0)
    assert apply[:3] == ['gz', 'topic', '-t'] and apply[3].endswith('/wrench/persistent')
    assert 'force: {x: 0.0, y: 500.0, z: 0}' in apply[-1]
    assert 'atlas::utorso' in apply[-1] and 'atlas::utorso' in clear[-1]
    assert clear[3].endswith('/wrench/clear')
