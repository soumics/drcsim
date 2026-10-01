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

"""Round-trip test for retarget.py: Atlas's own poses as landmarks (needs Pinocchio)."""

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

pin = pytest.importorskip('pinocchio')
xacro = pytest.importorskip('xacro')
ament_index = pytest.importorskip('ament_index_python.packages')
rt = pytest.importorskip('retarget')


@pytest.fixture(scope='module')
def model():
    try:
        share = ament_index.get_package_share_directory('atlas_description')
    except Exception:
        pytest.skip('atlas_description not installed')
    path = os.path.join(share, 'robots', 'atlas_v5.urdf.xacro')
    return path, pin.buildModelFromXML(xacro.process_file(path).toxml())


def landmarks(model, q, camera):
    """Return MediaPipe-style points (33 x 3) of Atlas at q, seen by a rotated camera."""
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)

    def p(name):
        return data.oMf[model.getFrameId(name)].translation

    w = np.zeros((33, 3))
    w[rt.L_SH], w[rt.R_SH] = p('l_arm_shz'), p('r_arm_shz')
    w[rt.L_EL], w[rt.R_EL] = p('l_arm_elx'), p('r_arm_elx')
    w[rt.L_WR], w[rt.R_WR] = p('l_arm_wrx'), p('r_arm_wrx')
    w[rt.L_HIP], w[rt.R_HIP] = p('l_leg_hpx'), p('r_leg_hpx')
    w[rt.L_ANK], w[rt.R_ANK] = p('l_leg_akx'), p('r_leg_akx')
    head = p('neck_ry')
    w[rt.NOSE] = head + np.array([0.15, 0, 0.05])
    w[rt.L_EAR], w[rt.R_EAR] = head + [0, 0.07, 0.05], head + [0, -0.07, 0.05]
    return (camera @ w.T).T


def test_retargeted_arms_point_like_the_source(model, tmp_path):
    path, full = model
    rng = np.random.default_rng(0)
    frames, poses = [], []
    camera = pin.rpy.rpyToMatrix(0.3, -1.2, 2.0)  # arbitrary camera orientation
    for _ in range(12):
        q = pin.neutral(full)
        for side in 'lr':
            for j in rt.ARM:
                i = full.joints[full.getJointId(f'{side}_arm_{j}')].idx_q
                q[i] = rng.uniform(full.lowerPositionLimit[i], full.upperPositionLimit[i])
        poses.append(q)
        frames.append(landmarks(full, q, camera))
    np.savez(tmp_path / 'pose.npz', world=np.array(frames), fps=1.0)
    sys.argv = ['retarget.py', str(tmp_path / 'pose.npz'), str(tmp_path / 'out.npz'), path]
    rt.main()
    out = np.load(tmp_path / 'out.npz')
    names = list(out['names'])
    data = full.createData()
    errors = []
    for q_src, joints in zip(poses, out['joints']):
        q = pin.neutral(full)
        for side in 'lr':
            for j in rt.ARM:
                i = full.joints[full.getJointId(f'{side}_arm_{j}')].idx_q
                q[i] = joints[names.index(f'{side}_arm_{j}')]
        for side in 'lr':
            dirs = []
            for qq in (q_src, q):
                pin.framesForwardKinematics(full, data, qq)
                pts = [data.oMf[full.getFrameId(f'{side}_arm_{j}')].translation
                       for j in ('shz', 'elx', 'wrx')]
                dirs.append([(pts[1] - pts[0]) / np.linalg.norm(pts[1] - pts[0]),
                             (pts[2] - pts[1]) / np.linalg.norm(pts[2] - pts[1])])
            errors.append(max(np.degrees(np.arccos(np.clip(a @ b, -1, 1)))
                              for a, b in zip(*dirs)))
    print('direction errors (deg):', np.round(errors, 1))
    assert np.median(errors) < 5.0
    assert max(errors) < 15.0
