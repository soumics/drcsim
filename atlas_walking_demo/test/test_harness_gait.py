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

"""Unit tests for harness_gait.py's leg IK and omnidirectional gait, no rclpy."""

import importlib.util
import math
import pathlib

import pytest

_SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'harness_gait.py')
_SPEC = importlib.util.spec_from_file_location('harness_gait_script', _SCRIPT_PATH)
hg = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(hg)

NAMES = hg.ATLAS_JOINT_NAMES
DT = 1.0 / 30.0
# URDF limits (atlas_v5_raw.urdf) for the leg joints the gait drives.
LIMITS = {
    'l_leg_hpz': (-0.174, 0.787), 'r_leg_hpz': (-0.787, 0.174),
    'hpx': (-0.524, 0.524), 'hpy': (-1.612, 0.658), 'kny': (0.0, 2.356),
    'aky': (-1.0, 0.7), 'akx': (-0.8, 0.8),
}
DIRECTIONS = {
    'forward': (hg.MAX_VX, 0.0, 0.0), 'backward': (-hg.MAX_VX_BACK, 0.0, 0.0),
    'left': (0.0, hg.MAX_VY, 0.0), 'right': (0.0, -hg.MAX_VY, 0.0),
    'turn_left': (0.0, 0.0, hg.MAX_WZ), 'turn_right': (0.0, 0.0, -hg.MAX_WZ),
    'curve_left': (hg.MAX_VX, 0.0, hg.MAX_WZ),
}


def _zero_pose():
    return {name: 0.0 for name in NAMES}


def _run(gait, seconds, dt=DT):
    """Sample for `seconds`; return the list of (positions, velocity)."""
    return [gait.sample(dt) for _ in range(int(round(seconds / dt)))]


def _crouched():
    gait = hg.HarnessGait(_zero_pose())
    _run(gait, hg.CROUCH_DURATION + 0.1)
    return gait


def _rotate_x(y, z, angle):
    c, s = math.cos(angle), math.sin(angle)
    return y * c - z * s, y * s + z * c


def _leg_fk_3d(side, hpz, hpx, hpy, kny):
    """Ankle position in the pelvis frame (independent of leg_ik_3d)."""
    sign = hg.SIDE_SIGN[side]
    sx, sz = hg.leg_fk(hpy, kny)
    x, y, z = hg.HPY_ORIGIN[0] + sx, sign * hg.HPY_ORIGIN[1], hg.HPY_ORIGIN[2] + sz
    y, z = _rotate_x(y, z, hpx)
    c, s = math.cos(hpz), math.sin(hpz)
    return x * c - y * s, sign * hg.HIP_Y + x * s + y * c, z


@pytest.mark.parametrize('x, z', [(-0.05, -0.77), (0.1, -0.76), (-0.2, -0.75)])
def test_leg_ik_round_trips_through_fk_with_a_flat_sole(x, z):
    hpy, kny, aky = hg.leg_ik(x, z)
    fx, fz = hg.leg_fk(hpy, kny)
    assert fx == pytest.approx(x, abs=1e-9)
    assert fz == pytest.approx(z, abs=1e-9)
    assert kny > 0.0  # knee bends forward
    assert hpy + kny + aky == pytest.approx(0.0)


@pytest.mark.parametrize('side', ['l', 'r'])
@pytest.mark.parametrize('dx, dy, dz, yaw', [
    (0.0, 0.0, 0.02, 0.0), (0.15, 0.0, 0.03, 0.0), (-0.12, 0.05, 0.05, 0.0),
    (0.05, -0.04, 0.03, 0.2), (-0.08, 0.06, 0.08, -0.15)])
def test_leg_ik_3d_round_trips_through_an_independent_fk(side, dx, dy, dz, yaw):
    sign = hg.SIDE_SIGN[side]
    target = (hg.FOOT_X0 + dx, sign * hg.FOOT_Y0 + dy, hg.FOOT_Z0 + dz)
    hpz, hpx, hpy, kny, aky, akx = hg.leg_ik_3d(side, *target, yaw)
    assert _leg_fk_3d(side, hpz, hpx, hpy, kny) == pytest.approx(target, abs=1e-9)
    assert hpz == yaw
    assert akx == -hpx and hpy + kny + aky == pytest.approx(0.0)  # sole level


def test_crouch_starts_at_the_held_setpoint_and_lowers_the_harness():
    gait = hg.HarnessGait(_zero_pose())
    positions, (vx, vy, vz, wz) = gait.sample(0.001)
    assert max(abs(p) for p in positions) < 0.01  # no snap at takeover
    assert (vx, vy, wz) == (0.0, 0.0, 0.0) and vz < 0.0

    _run(gait, hg.CROUCH_DURATION)
    positions, velocity = gait.sample(DT)
    assert gait.phase_name == 'STAND'
    assert velocity == (0.0, 0.0, 0.0, 0.0)
    assert positions[NAMES.index('l_arm_shx')] == pytest.approx(hg.ARM_REST['l_arm_shx'])
    assert positions[NAMES.index('l_leg_kny')] > 0.3  # knees bent


@pytest.mark.parametrize('direction', sorted(DIRECTIONS))
def test_every_direction_moves_the_harness_stays_in_limits_and_stops(direction):
    gait = _crouched()
    gait.set_velocity(*DIRECTIONS[direction])
    samples = _run(gait, 6.0)
    gait.stop()
    samples += _run(gait, 4.0 * hg.PERIOD)

    assert gait.phase_name == 'STAND' and not gait.walking
    for (a, _), (b, _) in zip(samples, samples[1:]):
        # Under 9 rad/s at 30 Hz (Atlas's joint velocity limit is 12).
        assert max(abs(p - q) for p, q in zip(a, b)) < 0.3
    for positions, _ in samples:
        for name, value in zip(NAMES, positions):
            joint = name if name in LIMITS else name.rsplit('_', 1)[-1]
            if '_leg_' in name and joint in LIMITS:
                lo, hi = LIMITS[joint]
                assert lo - 1e-6 <= value <= hi + 1e-6, name
    moved = [sum(v[i] for _, v in samples) * DT for i in (0, 1, 3)]
    for axis, commanded in zip(moved, DIRECTIONS[direction]):
        assert axis * commanded >= 0.0
        if commanded:
            assert abs(axis) > 0.5  # really went that way


def test_combined_walk_and_turn_is_scaled_to_the_legs_reach():
    gait = _crouched()
    gait.set_velocity(hg.MAX_VX, 0.0, hg.MAX_WZ)
    _run(gait, 4.0)
    vx, _, wz = gait.velocity
    assert abs(vx) + abs(wz) * hg.FOOT_Y0 == pytest.approx(hg.MAX_VX)


def test_stance_foot_moves_exactly_opposite_to_the_pelvis():
    gait = _crouched()
    gait.set_velocity(0.3, 0.1, 0.2)
    _run(gait, 3.0)  # up to speed
    for _ in range(60):
        before = {s: list(gait._foot[s]) for s in ('l', 'r')}
        planted = {s: not gait._swinging[s] for s in ('l', 'r')}
        _, (vx, vy, _, wz) = gait.sample(DT)
        for side in ('l', 'r'):
            if planted[side] and not gait._swinging[side]:
                x, y = before[side][0], before[side][1]
                assert gait._foot[side][0] - x == pytest.approx((-vx + wz * y) * DT)
                assert gait._foot[side][1] - y == pytest.approx((-vy - wz * x) * DT)


def test_arms_swing_opposite_the_legs():
    gait = _crouched()
    gait.start_walking()
    _run(gait, 3.0 * hg.PERIOD)
    for _ in range(40):
        positions, _ = gait.sample(DT)
        right_foot_ahead = gait._foot['r'][0] - hg.FOOT_X0
        left_foot_ahead = gait._foot['l'][0] - hg.FOOT_X0
        # Right foot forward -> left arm forward (negative l_arm_shz);
        # left foot forward -> right arm forward (positive r_arm_shz).
        assert positions[NAMES.index('l_arm_shz')] * right_foot_ahead <= 1e-12
        assert positions[NAMES.index('r_arm_shz')] * left_foot_ahead >= -1e-12
