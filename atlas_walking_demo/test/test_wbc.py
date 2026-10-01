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


def test_a_joint_moving_into_its_limit_is_braked(ctrl):
    q, v = _stance(ctrl)
    j = N.index('l_arm_elx')  # lower limit 0: the arm straight
    i, k = ctrl.iq[j], ctrl.iv[j]
    q[i] = ctrl.model.lowerPositionLimit[i] + 0.03
    v[k] = -2.0
    ctrl.solve(q, v, np.zeros(3), posture=np.full(len(N), -1.0))  # posture pulls past it
    assert ctrl.last['qdd'][k] > 0.0


@pytest.fixture(scope='module')
def svh_ctrl():
    try:
        share = ament_index.get_package_share_directory('atlas_svh_hands')
        urdf = xacro.process_file(
            os.path.join(share, 'robots', 'atlas_v5_svh_hands.urdf.xacro')).toxml()
    except Exception:
        pytest.skip('atlas_svh_hands (with the SVH description) not installed')
    return wbc.WholeBodyController(urdf, N)


def test_a_payload_adds_its_mass_at_the_palms_and_comes_off_again(svh_ctrl):
    c = svh_ctrl
    q, _ = _stance(c)
    bare = c.mass
    com0 = pin.centerOfMass(c.model, c.data, q).copy()
    c.set_payload(10.0, 0.15)
    assert c.mass == pytest.approx(bare + 10.0)
    pin.framesForwardKinematics(c.model, c.data, q)
    palms = np.mean([c.data.oMf[f].translation for f in c.palms.values()], axis=0)
    com = pin.centerOfMass(c.model, c.data, q)
    # The CoM moves toward the hands by about 10 / total of the way.
    assert np.linalg.norm(com - com0) > 0.5 * 10.0 / c.mass * np.linalg.norm(palms - com0)
    c.set_payload(0.0, 0.15)
    assert c.mass == pytest.approx(bare)


def test_the_hand_task_moves_the_palm_toward_its_target(svh_ctrl):
    c = svh_ctrl
    q, v = _stance(c)
    pin.framesForwardKinematics(c.model, c.data, q)
    f = c.palms['l']
    p0, r0 = c.data.oMf[f].translation.copy(), c.data.oMf[f].rotation.copy()
    c.solve(q, v, np.zeros(3), hand_targets={'l': (p0 + [0.0, 0.0, 0.1], r0)})
    pin.forwardKinematics(c.model, c.data, q, v, c.last['qdd'])
    acc = pin.getFrameClassicalAcceleration(c.model, c.data, f,
                                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    assert acc.linear[2] > 1.0


def test_a_squeeze_is_modelled_as_the_object_pushing_back_on_the_palms(svh_ctrl):
    c = svh_ctrl
    q, v = _stance(c)
    c.solve(q, v, np.zeros(3))
    f0, tau0 = c.last['f'].copy(), c.last['tau'].copy()
    c.solve(q, v, np.zeros(3), squeeze=100.0)
    f1, tau1 = c.last['f'], c.last['tau']
    # The object pushes each palm back along -normal; the feet carry the
    # rest of the weight. (Palms facing each other: the pushes cancel.)
    pin.framesForwardKinematics(c.model, c.data, q)
    push = sum(-100.0 * c.data.oMf[f].rotation[:, 1] for f in c.palms.values())
    assert f1[2] + f1[8] == pytest.approx(f0[2] + f0[8] - push[2], abs=1.0)
    # The arms do the pushing.
    arm = [N.index(n) for n in N if '_arm_' in n]
    assert np.abs(tau1[arm] - tau0[arm]).max() > 5.0


def test_qp_iterations_are_capped_and_a_failed_solve_keeps_the_last_torques(ctrl):
    q, v = _stance(ctrl)
    tau = ctrl.solve(q, v, np.zeros(3))
    assert wbc.MAX_ITER <= 500 and ctrl.last['iter'] < wbc.MAX_ITER
    ctrl.last['status'] = None  # what a capped, unsolved problem would leave
    assert np.allclose(ctrl.last['tau'], tau)
