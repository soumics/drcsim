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


def _zero_pose():
    return {name: 0.0 for name in gait_controller_script.ATLAS_JOINT_NAMES}


def test_idle_controller_samples_neutral_pose_unchanged():
    neutral = _zero_pose()
    neutral['back_bky'] = 0.05  # an arbitrary real-looking starting value
    gait = gait_controller_script.GaitController(neutral)

    position = gait.sample(0.1)

    expected = [neutral[name] for name in gait_controller_script.ATLAS_JOINT_NAMES]
    assert position == expected
    assert gait.walking is False


def test_start_walking_begins_immediately_leaning_toward_left_foot():
    # Regression test: the controller must never ask Atlas to move away
    # from its real starting pose before a walk is even requested (the
    # second interactive failure -- see the module docstring) -- pressing
    # 'w' should lean *from* the neutral pose right away, not after some
    # separate settle into a different pose first.
    neutral = _zero_pose()
    gait = gait_controller_script.GaitController(neutral)

    gait.start_walking()
    # A small step should be partway through SHIFT_LEFT, not yet at its
    # target -- interpolating, not snapping (AtlasPlugin has no rate
    # limiter of its own).
    position = gait.sample(0.01)

    assert gait.walking is True
    hpx_index = gait_controller_script.ATLAS_JOINT_NAMES.index('l_leg_hpx')
    assert position[hpx_index] > neutral['l_leg_hpx']  # leaning toward the support (left) foot
    assert position[hpx_index] < neutral['l_leg_hpx'] + gait_controller_script.LEAN_HPX


def test_full_shift_left_reaches_lean_target_relative_to_neutral():
    neutral = _zero_pose()
    neutral['l_leg_hpx'] = 0.069  # a nonzero starting value, as real joint_states would give
    neutral['r_leg_kny'] = 0.2
    gait = gait_controller_script.GaitController(neutral)
    gait.start_walking()

    position = gait.sample(gait_controller_script.SHIFT_DURATION)

    hpx_index = gait_controller_script.ATLAS_JOINT_NAMES.index('l_leg_hpx')
    assert position[hpx_index] == neutral['l_leg_hpx'] + gait_controller_script.LEAN_HPX
    r_kny_index = gait_controller_script.ATLAS_JOINT_NAMES.index('r_leg_kny')
    assert position[r_kny_index] == neutral['r_leg_kny']  # untouched by a pure weight shift


def test_swing_right_lifts_and_flexes_only_the_right_leg():
    neutral = _zero_pose()
    gait = gait_controller_script.GaitController(neutral)
    gait.start_walking()
    gait.sample(gait_controller_script.SHIFT_DURATION)  # -> LIFT_RIGHT begins
    gait.sample(gait_controller_script.LIFT_DURATION)  # -> SWING_RIGHT begins

    position = gait.sample(gait_controller_script.SWING_DURATION)  # SWING_RIGHT completes

    names = gait_controller_script.ATLAS_JOINT_NAMES
    assert position[names.index('r_leg_kny')] == (
        neutral['r_leg_kny'] + gait_controller_script.LIFT_KNY)
    assert position[names.index('r_leg_hpy')] == (
        neutral['r_leg_hpy'] + gait_controller_script.SWING_HPY)
    # Left (support) leg's own hip pitch is untouched by the right leg's swing.
    assert position[names.index('l_leg_hpy')] == neutral['l_leg_hpy']


def test_stop_walking_returns_to_idle_only_at_a_shift_boundary():
    gait = gait_controller_script.GaitController(_zero_pose())
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


def test_harness_cycle_alternates_legs_with_heel_strike_and_toe_off():
    names = [phase.name for phase in gait_controller_script.HARNESS_CYCLE]
    assert names == [
        'LIFT_RIGHT', 'SWING_RIGHT', 'PLANT_RIGHT',
        'LIFT_LEFT', 'SWING_LEFT', 'PLANT_LEFT',
    ]
    poses = gait_controller_script.HARNESS_LEG_POSES
    for key in ('MID', 'LIFTED'):
        assert abs(sum(poses[key])) < 1e-9, key  # sole flat
    assert sum(poses['FRONT']) > 0.0  # toe up: heel strike
    assert sum(poses['BACK']) < 0.0  # toe down: push-off
    assert sum(poses['PUSH_OFF']) < 0.0


def test_harness_swing_right_moves_right_leg_forward_and_left_leg_back():
    names = gait_controller_script.ATLAS_JOINT_NAMES
    gait = gait_controller_script.GaitController(_zero_pose(), harness=True)
    gait.start_walking()
    gait.sample(gait_controller_script.HARNESS_LIFT_DURATION)  # -> SWING_RIGHT

    position = gait.sample(gait_controller_script.HARNESS_SWING_DURATION)

    reach = gait_controller_script.HARNESS_LEG_POSES['REACH']
    back = gait_controller_script.HARNESS_LEG_POSES['BACK']
    assert position[names.index('r_leg_hpy')] == reach[0]
    assert position[names.index('l_leg_hpy')] == back[0]
    # No lateral lean under the harness.
    assert position[names.index('l_leg_hpx')] == 0.0


def test_harness_lowers_arms_smoothly_then_swings_them_opposite_the_legs():
    names = gait_controller_script.ATLAS_JOINT_NAMES
    gait = gait_controller_script.GaitController(_zero_pose(), harness=True)
    rest = gait_controller_script.HARNESS_ARM_REST

    first = gait.sample(0.01)  # starts from the T-pose, no snap
    assert 0.0 > first[names.index('l_arm_shx')] > rest['l_arm_shx']
    lowered = gait.sample(gait_controller_script.IDLE_DURATION)
    assert lowered[names.index('l_arm_shx')] == rest['l_arm_shx']

    gait.start_walking()
    gait.sample(gait_controller_script.HARNESS_LIFT_DURATION)
    position = gait.sample(gait_controller_script.HARNESS_SWING_DURATION)  # right leg reaching
    # Right leg forward -> left arm forward (negative shz), right arm back.
    assert position[names.index('l_arm_shz')] < 0.0
    assert position[names.index('r_arm_shz')] < 0.0


def test_harness_stop_returns_to_idle_after_a_plant_phase():
    gait = gait_controller_script.GaitController(_zero_pose(), harness=True)
    gait.start_walking()
    gait.sample(gait_controller_script.HARNESS_LIFT_DURATION)
    gait.stop_walking()
    gait.sample(gait_controller_script.HARNESS_SWING_DURATION)
    assert gait.walking is True  # right foot still in the air

    gait.sample(gait_controller_script.HARNESS_PLANT_DURATION)  # PLANT_RIGHT done

    assert gait.walking is False
    assert gait.phase_name == 'IDLE'


def test_walk_cycle_is_a_valid_alternating_sequence():
    names = [phase.name for phase in gait_controller_script.WALK_CYCLE]
    assert names == [
        'SHIFT_LEFT', 'LIFT_RIGHT', 'SWING_RIGHT', 'PLANT_RIGHT',
        'SHIFT_RIGHT', 'LIFT_LEFT', 'SWING_LEFT', 'PLANT_LEFT',
    ]
    for phase in gait_controller_script.WALK_CYCLE:
        assert phase.support != phase.swing
