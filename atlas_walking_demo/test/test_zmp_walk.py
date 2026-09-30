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

"""Unit tests for zmp_walk.py's online walker and feedforward, no rclpy."""

import math
import pathlib
import sys

# zmp_walk imports harness_gait by name, as the installed scripts do.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import harness_gait as hg  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
import zmp_walk as zw  # noqa: E402

# (time s, command): a velocity, or None for stop().
MIXED = [(1.0, (0.125, 0.0, 0.0)), (12.0, (0.0, 0.04, 0.0)), (20.0, (0.1, 0.0, 0.1)),
         (30.0, None)]


def _run(program, duration):
    walker = zw.ZmpWalker()
    samples = []
    program = list(program)
    for k in range(int(duration / zw.DT)):
        while program and k * zw.DT >= program[0][0]:
            command = program.pop(0)[1]
            if command is None:
                walker.stop()
            else:
                walker.set_velocity(*command)
        angles, share, info = walker.sample()
        samples.append((angles, share, info, {s: list(f) for s, f in walker.feet.items()},
                        dict(walker.feet_in_pelvis)))
    return walker, samples


@pytest.fixture(scope='module')
def mixed():
    return _run(MIXED, 42.0)


def test_starts_idle_and_stands_still_without_a_command():
    walker, samples = _run([], 5.0)
    assert walker.idle and not walker.steps
    assert samples[-1][2]['com'] == pytest.approx(samples[0][2]['com'], abs=1e-5)


def test_planned_zmp_tracks_the_reference(mixed):
    _, samples = mixed
    errors = [np.hypot(*np.subtract(i['zmp'], i['zmp_ref'])) for _, _, i, _, _ in samples]
    assert max(errors) < 0.02
    assert np.mean(errors) < 0.003


def test_feet_never_cross_or_collide(mixed):
    _, samples = mixed
    for _, _, _, feet, _ in samples:
        lx, ly, _, lyaw = feet['l']
        rx, ry, _, _ = feet['r']
        # The right ankle seen from the left foot's frame stays to its right.
        side = -(rx - lx) * math.sin(lyaw) + (ry - ly) * math.cos(lyaw)
        assert side < -0.15


def test_walks_forward_sideways_and_turns_then_stops_feet_together(mixed):
    walker, samples = mixed
    assert walker.idle
    (lx, ly, lz, lyaw), (rx, ry, rz, ryaw) = walker.feet['l'], walker.feet['r']
    assert lyaw == pytest.approx(ryaw) and lyaw > 0.5            # turned left
    assert math.hypot(lx - rx, ly - ry) == pytest.approx(2 * zw.FOOT_Y)  # side by side
    assert lz == rz == 0.0
    xs = [f['l'][0] for _, _, _, f, _ in samples]
    assert max(xs) > 1.0                                          # walked forward
    com = samples[-1][2]['com']
    assert com[0] == pytest.approx((lx + rx) / 2 + zw.ZMP_X_OFFSET * math.cos(lyaw), abs=0.02)


def test_commands_are_clamped_to_the_step_limits():
    walker = zw.ZmpWalker()
    walker.set_velocity(5.0, 5.0, 5.0)
    t_step = zw.SINGLE_SUPPORT + zw.DOUBLE_SUPPORT
    vx, vy, wz = walker.command
    assert vx * t_step == pytest.approx(zw.MAX_STEP)
    assert 2 * vy * t_step == pytest.approx(zw.MAX_SIDE_STEP)
    assert 2 * wz * t_step == pytest.approx(zw.MAX_TURN_STEP)
    assert zw.max_velocity() == pytest.approx((vx, zw.MAX_STEP_BACK / t_step, vy, wz))


def test_leg_targets_reproduce_the_planned_feet(mixed):
    _, samples = mixed
    for angles, _, _, _, feet_in_pelvis in samples[::50]:
        for side, rel in feet_in_pelvis.items():
            q = [angles[f'{side}_leg_{j}'] for j in ('hpz', 'hpx', 'hpy', 'kny')]
            assert hg.leg_fk_3d(side, *q) == pytest.approx(rel, abs=1e-9)


def test_load_share_sums_to_one_and_swing_leg_is_unloaded(mixed):
    _, samples = mixed
    for _, share, info, _, _ in samples:
        assert sum(share) == pytest.approx(1.0)
        if info['phase'] == 'swing_l':
            assert share == (0.0, 1.0)
        elif info['phase'] == 'swing_r':
            assert share == (1.0, 0.0)


def test_a_command_starts_stepping_after_the_preview_delay():
    walker, samples = _run([(0.0, (0.1, 0.0, 0.0))], 6.0)
    first_swing = next(i['t'] for _, _, i, _, _ in samples if i['phase'] != 'double')
    assert first_swing == pytest.approx(zw.START_DELAY + zw.FIRST_SHIFT, abs=2 * zw.DT)


def test_gravity_feedforward_balances_the_body_weight_about_the_hips():
    walker = zw.ZmpWalker()
    angles, share, _ = walker.sample()
    torques = zw.gravity_feedforward(angles, walker.feet_in_pelvis, share, (0.0, 0.0))
    # Standing symmetric: mirrored roll torques, equal pitch torques.
    assert torques['l_leg_hpx'] == pytest.approx(-torques['r_leg_hpx'], abs=1e-6)
    assert torques['l_leg_hpy'] == pytest.approx(torques['r_leg_hpy'], abs=1e-6)
    assert torques['l_leg_aky'] == 0.0 and torques['l_leg_akx'] == 0.0
    # A load ahead of the ankle loads the ankle pitch joint in proportion.
    torques = zw.gravity_feedforward(angles, walker.feet_in_pelvis, (1.0, 0.0), (0.05, 0.0))
    assert torques['l_leg_aky'] == pytest.approx(0.05 * zw.MASS * zw.G)
    assert torques['r_leg_hpy'] == 0.0
