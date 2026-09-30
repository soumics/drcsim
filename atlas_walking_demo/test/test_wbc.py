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

"""Tests for wbc.py's whole-body QP on the atlas_v5 model (needs Pinocchio, ProxQP)."""

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import harness_gait as hg  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
import zmp_walk as zw  # noqa: E402

pin = pytest.importorskip('pinocchio')
pytest.importorskip('proxsuite')
xacro = pytest.importorskip('xacro')
ament_index = pytest.importorskip('ament_index_python.packages')
wbc = pytest.importorskip('wbc')

N = hg.ATLAS_JOINT_NAMES


@pytest.fixture(scope='module')
def ctrl():
    try:
        share = ament_index.get_package_share_directory('atlas_description')
    except Exception:
        pytest.skip('atlas_description not installed')
    urdf = xacro.process_file(os.path.join(share, 'robots', 'atlas_v5.urdf.xacro')).toxml()
    return wbc.WholeBodyController(urdf, N)


def _stance(ctrl, contacts='lr'):
    angles, _, _ = zw.ZmpWalker().sample()
    pos = np.zeros(len(N))
    for name, a in angles.items():
        pos[N.index(name)] = a
    ctrl.sole_world = None
    return ctrl.state(pos, np.zeros(len(N)), [0, 0, 0, 1], np.zeros(3), contacts)


def test_standing_still_the_feet_carry_the_weight_within_torque_limits(ctrl):
    q, v = _stance(ctrl)
    tau = ctrl.solve(q, v, np.zeros(3))
    f = ctrl.last['f']
    assert f[2] + f[8] == pytest.approx(ctrl.mass * 9.81, rel=1e-3)
    assert f[2] == pytest.approx(f[8], rel=0.05)  # symmetric stance, even load
    assert np.all(np.abs(tau) <= ctrl.tau_max + 1e-6)


def test_a_lifted_foot_gets_no_wrench(ctrl):
    q, v = _stance(ctrl)
    ctrl.solve(q, v, np.zeros(3), contacts='l')
    f = ctrl.last['f']
    assert np.allclose(f[6:], 0.0, atol=1e-6)
    assert f[2] > 0.9 * ctrl.mass * 9.81  # standing on the left foot alone


def test_pushing_the_com_forward_moves_the_centre_of_pressure_forward(ctrl):
    q, v = _stance(ctrl)
    ctrl.solve(q, v, np.array([0.5, 0.0, 0.0]))
    f = ctrl.last['f']
    cop_x = [-f[4] / f[2], -f[10] / f[8]]  # CoP x = -m_y / f_z, sole frames
    assert all(c < -0.02 for c in cop_x)  # the ground pushes the CoM forward from behind
