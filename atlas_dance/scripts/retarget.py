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

"""
Retarget a dancer's MediaPipe pose (extract_pose.py) onto Atlas's joints.

usage: retarget.py POSE.npz OUT.npz [URDF_XACRO]
Upper body only -- the legs are the walking/balance controller's job (they
carry 180 kg; a copied leg pose would topple Atlas):
- back_bkz/bky/bkx: the chest frame (shoulders) relative to the pelvis frame
  (hips), as Atlas's z-y-x back chain, within joint limits;
- arms (shz, shx, ely, elx per side; wrists held): per frame a small
  least-squares IK makes Atlas's upper arm and forearm point along the
  dancer's (directions in the chest frame, so body size doesn't matter),
  warm-started and smoothed toward the previous frame;
- neck_ry: the nose above/below the ears' midpoint.
Also writes signals for the leg controller: 'crouch' (hip height drop, m)
and 'sway' (hips sideways over the ankles, m). Output: t (T,), joints
(T, 30; NaN where not retargeted), crouch (T,), sway (T,), fps.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
import xacro

JOINTS = [
    'back_bkz', 'back_bky', 'back_bkx', 'neck_ry',
    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
    'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx',
    'l_arm_wry2',
    'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx',
    'r_arm_wry2',
]
# MediaPipe Pose landmark indices.
NOSE, L_EAR, R_EAR = 0, 7, 8
L_SH, R_SH, L_EL, R_EL, L_WR, R_WR = 11, 12, 13, 14, 15, 16
L_HIP, R_HIP, L_ANK, R_ANK = 23, 24, 27, 28
ARM = ('shz', 'shx', 'ely', 'elx')


def frame(left, right, up_from, up_to):
    """Return a rotation (columns x fwd, y left, z up) from a left-right pair and an up vector."""
    y = left - right
    y /= np.linalg.norm(y)
    z = up_to - up_from
    z -= y * (z @ y)
    z /= np.linalg.norm(z)
    return np.column_stack([np.cross(y, z), y, z])


def zyx(R):
    """Return (z, y, x) angles with R = Rz(z) Ry(y) Rx(x)."""
    return (np.arctan2(R[1, 0], R[0, 0]), np.arcsin(-np.clip(R[2, 0], -1, 1)),
            np.arctan2(R[2, 1], R[2, 2]))


class ArmIK:
    """Direction-matching IK for one Atlas arm on a fixed-torso Pinocchio model."""

    def __init__(self, model, side):
        self.model, self.data = model, model.createData()
        self.side = side
        self.idx = [model.joints[model.getJointId(f'{side}_arm_{j}')].idx_q for j in ARM]
        self.lower = np.array([model.lowerPositionLimit[i] for i in self.idx])
        self.upper = np.array([model.upperPositionLimit[i] for i in self.idx])
        # Joint frames (the wrist joints are locked in this model, but their
        # frames remain).
        self.points = [model.getFrameId(f'{side}_arm_{j}') for j in ('shz', 'elx', 'wrx')]
        self.torso = model.getFrameId('utorso')
        self.prev = np.clip(np.zeros(4), self.lower, self.upper)

    def directions(self, x):
        q = pin.neutral(self.model)
        q[self.idx] = x
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        R = self.data.oMf[self.torso].rotation
        p = [R.T @ self.data.oMf[f].translation for f in self.points]
        u, f = p[1] - p[0], p[2] - p[1]
        return u / np.linalg.norm(u), f / np.linalg.norm(f)

    def solve(self, upper_dir, fore_dir, smooth=0.1):
        def residual(x):
            u, f = self.directions(x)
            return np.concatenate([u - upper_dir, f - fore_dir, smooth * (x - self.prev)])
        x0 = np.clip(self.prev, self.lower + 1e-6, self.upper - 1e-6)
        self.prev = least_squares(residual, x0, bounds=(self.lower, self.upper),
                                  max_nfev=60).x
        return self.prev


def main():
    pose = np.load(sys.argv[1])
    out = sys.argv[2]
    xacro_path = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
        get_package_share_directory('atlas_description'), 'robots', 'atlas_v5.urdf.xacro')
    full = pin.buildModelFromXML(xacro.process_file(xacro_path).toxml())
    keep = [f'{s}_arm_{j}' for s in 'lr' for j in ARM]
    lock = [full.getJointId(n) for n in full.names[1:] if n not in keep]
    model = pin.buildReducedModel(full, lock, pin.neutral(full))
    limits = {n: (full.lowerPositionLimit[full.joints[full.getJointId(n)].idx_q],
                  full.upperPositionLimit[full.joints[full.getJointId(n)].idx_q])
              for n in ('back_bkz', 'back_bky', 'back_bkx', 'neck_ry')}
    ik = {s: ArmIK(model, s) for s in 'lr'}
    # Landmark frames vs Atlas's own: Atlas's shoulders aren't straight above
    # its hips, so the frame the landmarks give is tilted by a constant; take
    # it from Atlas's neutral pose (the dancer's is assumed alike).
    data = full.createData()
    pin.framesForwardKinematics(full, data, pin.neutral(full))

    def fk(name):
        return data.oMf[full.getFrameId(name)]

    hip_l, hip_r = fk('l_leg_hpx').translation, fk('r_leg_hpx').translation
    sh_l, sh_r = fk('l_arm_shz').translation, fk('r_arm_shz').translation
    hip_c, sh_c = (hip_l + hip_r) / 2, (sh_l + sh_r) / 2
    chest_cal = frame(sh_l, sh_r, hip_c, sh_c).T @ fk('utorso').rotation
    pelvis_cal = frame(hip_l, hip_r, hip_c, sh_c).T @ fk('pelvis').rotation

    world = pose['world'].astype(float)
    # Smooth landmark jitter (~0.2 s window) before retargeting.
    fps = float(pose['fps'])
    win = max(5, int(0.2 * fps) | 1)
    if fps >= 10 and len(world) > win:
        world = savgol_filter(world, win, 2, axis=0)
    T = len(world)
    joints = np.full((T, len(JOINTS)), np.nan)
    crouch, sway = np.zeros(T), np.zeros(T)
    for k, w in enumerate(world):
        pelvis = frame(w[L_HIP], w[R_HIP], (w[L_HIP] + w[R_HIP]) / 2,
                       (w[L_SH] + w[R_SH]) / 2) @ pelvis_cal
        chest = frame(w[L_SH], w[R_SH], (w[L_HIP] + w[R_HIP]) / 2,
                      (w[L_SH] + w[R_SH]) / 2) @ chest_cal
        bkz, bky, bkx = zyx(pelvis.T @ chest)
        for name, value in zip(('back_bkz', 'back_bky', 'back_bkx'), (bkz, bky, bkx)):
            joints[k, JOINTS.index(name)] = np.clip(value, *limits[name])
        head = w[NOSE] - (w[L_EAR] + w[R_EAR]) / 2
        head_c = chest.T @ head
        neck = -np.arctan2(head_c[2], max(1e-6, head_c[0])) - 0.3  # ~0 looking ahead
        joints[k, JOINTS.index('neck_ry')] = np.clip(neck, *limits['neck_ry'])
        for side, (sh, el, wr) in (('l', (L_SH, L_EL, L_WR)), ('r', (R_SH, R_EL, R_WR))):
            u = chest.T @ (w[el] - w[sh])
            f = chest.T @ (w[wr] - w[el])
            x = ik[side].solve(u / np.linalg.norm(u), f / np.linalg.norm(f))
            for j, value in zip(ARM, x):
                joints[k, JOINTS.index(f'{side}_arm_{j}')] = value
            for j in ('wry', 'wrx', 'wry2'):
                joints[k, JOINTS.index(f'{side}_arm_{j}')] = 0.0
        hip = (w[L_HIP] + w[R_HIP]) / 2
        ank = (w[L_ANK] + w[R_ANK]) / 2
        leg = pelvis.T @ (hip - ank)
        crouch[k] = leg[2]
        sway[k] = leg[1]
    crouch = crouch.max() - crouch  # drop from the tallest stance
    np.savez(out, t=np.arange(T) / fps, joints=joints, crouch=crouch, sway=sway, fps=fps,
             names=np.array(JOINTS))
    print(f'{out}: {T} frames ({T / fps:.1f} s); back range '
          f'{np.degrees(np.nanmin(joints[:, :3])):.0f}..{np.degrees(np.nanmax(joints[:, :3])):.0f}'
          f' deg; crouch up to {crouch.max():.2f} m')


if __name__ == '__main__':
    main()
