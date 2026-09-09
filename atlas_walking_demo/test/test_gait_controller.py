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

"""Unit tests for gait_controller.py's state machine, no rclpy needed."""

import importlib.util
import pathlib

_SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'gait_controller.py')
_SPEC = importlib.util.spec_from_file_location('gait_controller_script', _SCRIPT_PATH)
gait_controller_script = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gait_controller_script)


def test_neutral_stand_has_all_30_joints():
    assert len(gait_controller_script.NEUTRAL_STAND) == 30
    assert set(gait_controller_script.NEUTRAL_STAND) == set(
        gait_controller_script.ATLAS_JOINT_NAMES)


def test_idle_controller_samples_neutral_stand():
    gait = gait_controller_script.GaitController()

    position = gait.sample(0.1)

    expected = [
        gait_controller_script.NEUTRAL_STAND[name]
        for name in gait_controller_script.ATLAS_JOINT_NAMES]
    assert position == expected
    assert gait.walking is False


def test_start_walking_begins_shift_left_and_leans_toward_left_foot():
    gait = gait_controller_script.GaitController()

    gait.start_walking()
    # A single small step should be partway through SHIFT_LEFT, not yet at
    # its target -- interpolating, not snapping (see the module docstring
    # on why AtlasPlugin has no rate limiter of its own).
    position = gait.sample(0.01)

    assert gait.walking is True
    hpx_index = gait_controller_script.ATLAS_JOINT_NAMES.index('l_leg_hpx')
    neutral_hpx = gait_controller_script.NEUTRAL_STAND['l_leg_hpx']
    assert position[hpx_index] > neutral_hpx  # leaning toward the support (left) foot
    assert position[hpx_index] < neutral_hpx + gait_controller_script.LEAN_HPX  # not there yet


def test_full_shift_left_reaches_lean_target_without_lifting():
    gait = gait_controller_script.GaitController()
    gait.start_walking()

    position = gait.sample(gait_controller_script.SHIFT_DURATION)

    hpx_index = gait_controller_script.ATLAS_JOINT_NAMES.index('l_leg_hpx')
    expected_hpx = gait_controller_script.NEUTRAL_STAND['l_leg_hpx'] + (
        gait_controller_script.LEAN_HPX)
    assert position[hpx_index] == expected_hpx
    r_kny_index = gait_controller_script.ATLAS_JOINT_NAMES.index('r_leg_kny')
    assert position[r_kny_index] == gait_controller_script.NEUTRAL_STAND['r_leg_kny']


def test_swing_right_lifts_and_flexes_only_the_right_leg():
    gait = gait_controller_script.GaitController()
    gait.start_walking()
    gait.sample(gait_controller_script.SHIFT_DURATION)  # -> LIFT_RIGHT begins
    gait.sample(gait_controller_script.LIFT_DURATION)  # -> SWING_RIGHT begins

    position = gait.sample(gait_controller_script.SWING_DURATION)  # SWING_RIGHT completes

    names = gait_controller_script.ATLAS_JOINT_NAMES
    stand = gait_controller_script.NEUTRAL_STAND
    assert position[names.index('r_leg_kny')] == stand['r_leg_kny'] + (
        gait_controller_script.LIFT_KNY)
    assert position[names.index('r_leg_hpy')] == stand['r_leg_hpy'] + (
        gait_controller_script.SWING_HPY)
    # Left (support) leg's own hip pitch is untouched by the right leg's swing.
    assert position[names.index('l_leg_hpy')] == stand['l_leg_hpy']


def test_stop_walking_returns_to_idle_only_at_a_shift_boundary():
    gait = gait_controller_script.GaitController()
    gait.start_walking()
    gait.sample(gait_controller_script.SHIFT_DURATION)  # completes SHIFT_LEFT
    gait.stop_walking()
    # Stop was requested mid-cycle, after SHIFT_LEFT already finished -- the
    # walk continues through the already-committed LIFT/SWING/PLANT_RIGHT
    # phases rather than freezing mid-air, only returning to idle at the
    # *next* SHIFT boundary (SHIFT_RIGHT's completion).
    gait.sample(gait_controller_script.LIFT_DURATION)
    gait.sample(gait_controller_script.SWING_DURATION)
    gait.sample(gait_controller_script.PLANT_DURATION)
    assert gait.walking is True  # still finishing the committed step

    gait.sample(gait_controller_script.SHIFT_DURATION)  # completes SHIFT_RIGHT

    assert gait.walking is False


def test_walk_cycle_is_a_valid_alternating_sequence():
    names = [phase.name for phase in gait_controller_script.WALK_CYCLE]
    assert names == [
        'SHIFT_LEFT', 'LIFT_RIGHT', 'SWING_RIGHT', 'PLANT_RIGHT',
        'SHIFT_RIGHT', 'LIFT_LEFT', 'SWING_LEFT', 'PLANT_LEFT',
    ]
    for phase in gait_controller_script.WALK_CYCLE:
        assert phase.support != phase.swing
