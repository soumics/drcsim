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

"""Unit tests for harness_gait.py's leg IK and gait, no rclpy needed."""

import importlib.util
import pathlib

import pytest

_SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'harness_gait.py')
_SPEC = importlib.util.spec_from_file_location('harness_gait_script', _SCRIPT_PATH)
hg = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(hg)

NAMES = hg.ATLAS_JOINT_NAMES
DT = 1.0 / 30.0


def _zero_pose():
    return {name: 0.0 for name in NAMES}


def _run(gait, seconds, dt=DT):
    """Sample for `seconds`; return the list of (positions, velocity)."""
    return [gait.sample(dt) for _ in range(int(round(seconds / dt)))]


@pytest.mark.parametrize('x, z', [(-0.05, -0.77), (0.1, -0.76), (-0.2, -0.75)])
def test_leg_ik_round_trips_through_fk_with_a_flat_sole(x, z):
    hpy, kny, aky = hg.leg_ik(x, z)
    fx, fz = hg.leg_fk(hpy, kny)
    assert fx == pytest.approx(x, abs=1e-9)
    assert fz == pytest.approx(z, abs=1e-9)
    assert kny > 0.0  # knee bends forward
    assert hpy + kny + aky == pytest.approx(0.0)


def test_crouch_starts_at_the_held_setpoint_and_lowers_the_harness():
    gait = hg.HarnessGait(_zero_pose())
    positions, (vx, vz) = gait.sample(0.001)
    assert max(abs(p) for p in positions) < 0.01  # no snap at takeover
    assert vx == 0.0 and vz < 0.0

    _run(gait, hg.CROUCH_DURATION)
    positions, (vx, vz) = gait.sample(DT)
    assert gait.phase_name == 'STAND'
    assert (vx, vz) == (0.0, 0.0)
    assert positions[NAMES.index('l_arm_shx')] == pytest.approx(hg.ARM_REST['l_arm_shx'])
    assert positions[NAMES.index('l_leg_kny')] > 0.3  # knees bent


def test_walk_is_continuous_and_stops_back_at_the_stance():
    gait = hg.HarnessGait(_zero_pose())
    gait.start_walking()
    samples = _run(gait, hg.CROUCH_DURATION + 6.0)
    gait.stop_walking()
    samples += _run(gait, 4.0 * hg.PERIOD)

    assert gait.phase_name == 'STAND' and gait.walking is False
    for (a, _), (b, _) in zip(samples, samples[1:]):
        # No jumps: under 9 rad/s at 30 Hz (a human knee peaks ~6 rad/s in
        # swing; Atlas's joint velocity limit is 12 rad/s).
        assert max(abs(p - q) for p, q in zip(a, b)) < 0.3
    travelled = sum(vel[0] for _, vel in samples) * DT
    assert travelled > 2.0
    final = samples[-1][0]
    for joint in ('hpy', 'kny', 'aky'):
        assert final[NAMES.index(f'l_leg_{joint}')] == pytest.approx(
            final[NAMES.index(f'r_leg_{joint}')], abs=1e-6)


def test_stance_foot_moves_back_exactly_as_fast_as_the_pelvis():
    gait = hg.HarnessGait(_zero_pose())
    gait.start_walking()
    _run(gait, hg.CROUCH_DURATION + 3.0 * hg.RAMP_TIME)  # full speed
    for _ in range(60):
        before = dict(gait._ankle_x)
        planted = {s: not gait._swinging[s] for s in ('l', 'r')}
        _, (vx, _) = gait.sample(DT)
        for side in ('l', 'r'):
            if planted[side] and not gait._swinging[side]:
                assert gait._ankle_x[side] - before[side] == pytest.approx(-vx * DT)


def test_arms_swing_opposite_the_legs():
    gait = hg.HarnessGait(_zero_pose())
    gait.start_walking()
    _run(gait, hg.CROUCH_DURATION + 3.0 * hg.PERIOD)
    for _ in range(40):
        positions, _ = gait.sample(DT)
        right_foot_ahead = gait._ankle_x['r'] - hg.ANKLE_X0
        left_foot_ahead = gait._ankle_x['l'] - hg.ANKLE_X0
        # Right foot forward -> left arm forward (negative l_arm_shz);
        # left foot forward -> right arm forward (positive r_arm_shz).
        assert positions[NAMES.index('l_arm_shz')] * right_foot_ahead <= 0.0
        assert positions[NAMES.index('r_arm_shz')] * left_foot_ahead >= 0.0
