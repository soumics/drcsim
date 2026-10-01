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

"""Tests for qp_walk.py: the walking plan as the torque QP sees it."""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

pytest.importorskip('scipy')
qw = pytest.importorskip('qp_walk')
zw = pytest.importorskip('zmp_walk')


def _turned(yaw, offset):
    """Nominal soles, turned by yaw and moved by offset (an arbitrary QP frame)."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return {k: (rot @ p + offset, yaw) for k, (p, _) in qw.nominal_soles().items()}


def test_standing_still_the_plan_keeps_both_feet_down_at_the_com():
    soles = _turned(0.3, np.array([1.0, -2.0, 0.5]))
    mid = (soles['l'][0] + soles['r'][0]) / 2
    walk = qw.QpWalk(soles, mid, 0.0)
    walk.advance(1.0)
    c, v, a, p = walk.com_reference(1.0)
    assert walk.contacts('lr') == 'lr'
    assert walk.foot_targets('lr') == {}
    assert np.allclose(c, mid[:2], atol=0.03) and np.allclose(v, 0.0, atol=1e-3)
    assert walk.heading() == pytest.approx(0.3, abs=1e-6)


def test_a_swing_foot_leaves_the_contacts_and_lands_ahead_in_the_qp_frame():
    yaw = 0.5
    soles = _turned(yaw, np.array([1.0, -2.0, 0.5]))
    walk = qw.QpWalk(soles, (soles['l'][0] + soles['r'][0]) / 2, 0.0)
    walk.set_velocity(0.1, 0.0, 0.0)
    seen, t = set(), 0.0
    while t < 8.0:
        t += zw.DT
        walk.advance(t)
        contacts = walk.contacts('')
        for side, (pos, foot_yaw) in walk.foot_targets(contacts).items():
            seen.add(side)
            assert pos[2] >= 0.5 - 0.011 and foot_yaw == pytest.approx(yaw, abs=1e-6)
        assert contacts in ('lr', 'l', 'r')
    assert seen == {'l', 'r'}
    c, *_ = walk.com_reference(t)
    start = (soles['l'][0] + soles['r'][0]) / 2
    moved = c - start[:2]
    assert moved @ np.array([math.cos(yaw), math.sin(yaw)]) > 0.2  # forward, in its heading


def test_a_route_turns_and_ends_standing_where_predict_says():
    route = [(0.0, 0.0, -0.2, 6), (0.1, 0.0, 0.0, 4)]
    soles, duration = qw.predict(route)
    assert soles['l'][1] == pytest.approx(soles['r'][1], abs=1e-6)  # feet side by side
    assert soles['l'][1] < -0.5                                        # turned right
    gap = soles['l'][0] - soles['r'][0]
    assert np.linalg.norm(gap) == pytest.approx(2 * zw.FOOT_Y, abs=0.02)
    assert 10.0 < duration < 60.0
