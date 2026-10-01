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

"""Tests for policy_io.py: the observation/action conventions (no simulator)."""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import numpy as np  # noqa: E402
import policy_io as pio  # noqa: E402
import pytest  # noqa: E402


def test_gravity_points_down_when_level_and_backward_when_pitched_back():
    assert pio.gravity_in_body((1, 0, 0, 0)) == pytest.approx([0, 0, -1])
    # Lying on its back (-90 deg about y, body x up): gravity along body -x.
    half = math.radians(-90) / 2
    g = pio.gravity_in_body((math.cos(half), 0, math.sin(half), 0))
    assert g == pytest.approx([-1, 0, 0], abs=1e-9)


def test_observation_layout_and_scaling():
    q = np.arange(30) * 0.01
    qd = np.arange(30) * 1.0
    stance = np.zeros(30)
    last = np.full(15, 0.5)
    obs = pio.observation((1, 0, 0, 0), (4.0, 0, 0), q, qd, stance, last)
    assert obs.shape == (pio.OBS_SIZE,)
    assert obs[3] == pytest.approx(1.0)  # 0.25 x omega
    assert obs[6:21] == pytest.approx(q[pio.CONTROLLED_IDX])
    assert obs[21:36] == pytest.approx(0.1 * qd[pio.CONTROLLED_IDX])
    assert obs[36:] == pytest.approx(last)


def test_actions_move_only_legs_and_back_and_are_clipped():
    stance = np.linspace(-1, 1, 30)
    target = pio.joint_targets(np.full(15, 2.0), stance)  # clipped to 1
    moved = np.flatnonzero(~np.isclose(target, stance))
    assert set(moved) == set(pio.CONTROLLED_IDX)
    assert target[pio.CONTROLLED_IDX] - stance[pio.CONTROLLED_IDX] == pytest.approx(
        np.full(15, pio.ACTION_SCALE))
