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

"""Unit tests for zmp_walk.py's preview controller and feedforward, no rclpy."""

import pathlib
import sys

# zmp_walk imports harness_gait by name, as the installed scripts do.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import harness_gait as hg  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
import zmp_walk as zw  # noqa: E402


def _walk(n_steps=6, step_length=zw.STEP_LENGTH):
    walker = zw.ZmpWalker(n_steps, step_length)
    samples = []
    while not walker.done:
        angles, share, info = walker.sample()
        samples.append((angles, share, info, {s: list(f) for s, f in walker.feet.items()}))
    return walker, samples


def test_footstep_plan_alternates_and_closes_the_feet():
    plan, t_end = zw.footstep_plan(4, 0.2)
    assert [p[2] for p in plan] == ['r', 'l', 'r', 'l']
    assert plan[0][4][0] == pytest.approx(0.1)  # half first step
    assert plan[-1][4][0] == pytest.approx(plan[-2][4][0])  # feet together
    assert t_end > plan[-1][1]


def test_planned_zmp_tracks_the_reference():
    _, samples = _walk()
    errors = [np.hypot(*np.subtract(i['zmp'], i['zmp_ref'])) for _, _, i, _ in samples]
    assert max(errors) < 0.02
    assert np.mean(errors) < 0.003


def test_zmp_reference_stays_inside_the_support_feet():
    _, samples = _walk()
    for _, share, info, feet in samples:
        zx, zy = info['zmp_ref']
        stance = [s for s, frac in zip('lr', share) if frac > 0.0]
        # Within the sole of a loaded foot, or between the feet in double support.
        xs = [feet[s][0] for s in stance]
        ys = [feet[s][1] for s in stance]
        assert min(xs) - 0.1 <= zx <= max(xs) + 0.13
        assert min(ys) - 0.07 <= zy <= max(ys) + 0.07


def test_walk_ends_with_the_feet_together_and_the_com_between_them():
    walker, samples = _walk(6, 0.15)
    feet = walker.feet
    assert feet['l'][0] == pytest.approx(feet['r'][0])
    assert feet['l'][2] == pytest.approx(0.0) and feet['r'][2] == pytest.approx(0.0)
    com = samples[-1][2]['com']
    assert com[0] == pytest.approx(feet['l'][0] + zw.ZMP_X_OFFSET, abs=0.02)
    assert com[1] == pytest.approx(0.0, abs=0.01)


def test_leg_targets_reproduce_the_planned_feet():
    walker = zw.ZmpWalker(4)
    for _ in range(int(6.0 / zw.DT)):
        angles, _, _ = walker.sample()
    for side, rel in walker.feet_in_pelvis.items():
        q = [angles[f'{side}_leg_{j}'] for j in ('hpz', 'hpx', 'hpy', 'kny')]
        assert hg.leg_fk_3d(side, *q) == pytest.approx(rel, abs=1e-9)


def test_load_share_sums_to_one_and_swing_leg_is_unloaded():
    _, samples = _walk()
    for _, share, info, _ in samples:
        assert sum(share) == pytest.approx(1.0)
        if info['phase'] == 'swing_l':
            assert share == (0.0, 1.0)
        elif info['phase'] == 'swing_r':
            assert share == (1.0, 0.0)


def test_gravity_feedforward_balances_the_body_weight_about_the_hips():
    walker = zw.ZmpWalker(4)
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
