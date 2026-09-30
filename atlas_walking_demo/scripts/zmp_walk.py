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
Free-standing walking pattern generator for Atlas: ZMP preview control.

Pure Python (numpy/scipy, no rclpy). Classic position-controlled humanoid
walking (Kajita et al., "Biped walking pattern generation by using preview
control of zero-moment point", ICRA 2003):

* Atlas is modelled as a linear inverted pendulum: its centre of mass (CoM)
  at a constant height COM_HEIGHT above the soles.
* Footsteps give a ZMP (centre-of-pressure) reference: on the stance foot
  during single support, shifting to the next foot during double support.
* A preview controller looks PREVIEW seconds ahead along that reference and
  plans a CoM path whose ZMP tracks it -- so the ground reaction always
  stays inside the feet.
* Each tick the pelvis is placed at the CoM minus the body's CoM offset,
  swing feet follow smooth arcs, and harness_gait's 3D leg IK turns foot
  positions relative to the pelvis into joint angles.

Everything is in a fixed "walk frame" (x forward, y left, z up from the
soles) that starts centred between the feet; the node adds balance
feedback on top.
"""

import math

import harness_gait as hg
import numpy as np
from scipy.linalg import solve_discrete_are

G = 9.81
DT = 0.005               # control period (s): 200 Hz of simulation time
PREVIEW = 1.6            # preview horizon (s)
COM_HEIGHT = 1.12        # CoM above the soles in the crouched stance (m)
# CoM in the pelvis frame for the crouched stance with the arms down
# (computed from atlas_v5 + SVH masses: 179.7 kg).
COM_IN_PELVIS = (0.029, 0.0)
COM_HEIGHT_IN_PELVIS = 0.234
FOOT_Y = hg.FOOT_Y0      # ankle half-spacing (m)
ZMP_X_OFFSET = 0.02      # ZMP reference ahead of the ankle, toward mid-sole (m)
ZMP_Y_INSET = 0.04       # ZMP reference inside the ankle, toward the other foot (m)
ANKLE_Z = hg.FOOT_Z0 + hg.CROUCH_DROP  # ankle below the pelvis when standing

STEP_LENGTH = 0.15       # m per step (0.20 still falls free-standing)
SINGLE_SUPPORT = 0.8     # s
DOUBLE_SUPPORT = 0.4     # s
START_HOLD = PREVIEW     # s standing still first: the preview must not see a change at t=0
FIRST_SHIFT = 1.0        # s shifting onto the first stance foot
SWING_HEIGHT = 0.05      # m


def preview_gains(dt=DT, zc=COM_HEIGHT, horizon=PREVIEW, q_error=1.0, r=1e-6):
    """
    Return (Ke, Kx, Gd, A, B, C) for the LIPM preview servo (cart-table model).

    State x = [c, c_dot, c_ddot] per axis, input u = CoM jerk, output
    p = c - zc/g * c_ddot (the ZMP). Integral-augmented LQR plus preview
    gains Gd[j] for the reference j+1 steps ahead.
    """
    A = np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1]])
    B = np.array([[dt ** 3 / 6], [dt * dt / 2], [dt]])
    C = np.array([[1, 0, -zc / G]])
    At = np.block([[np.eye(1), C @ A], [np.zeros((3, 1)), A]])
    Bt = np.vstack([C @ B, B])
    It = np.array([[1], [0], [0], [0]])
    Q = np.diag([q_error, 0, 0, 0])
    P = solve_discrete_are(At, Bt, Q, np.array([[r]]))
    S = np.linalg.inv(r + Bt.T @ P @ Bt)
    K = S @ Bt.T @ P @ At
    Ke, Kx = K[0, 0], K[0, 1:]
    Ac = At - Bt @ K
    n = int(round(horizon / dt))
    Gd = np.zeros(n)
    X = -Ac.T @ P @ It
    for j in range(n):
        Gd[j] = (S @ Bt.T @ X)[0, 0]
        X = Ac.T @ X
    return Ke, Kx, Gd, A, B, C


def footstep_plan(n_steps, step_length=STEP_LENGTH, first='r'):
    """
    Return [(t_start, t_end, swing_side, from_xy, to_xy), ...] and the end time.

    Feet start at (0, +-FOOT_Y). Steps alternate starting with `first`;
    the last two steps bring the feet together again.
    """
    feet = {'l': [0.0, FOOT_Y], 'r': [0.0, -FOOT_Y]}
    t = START_HOLD + FIRST_SHIFT
    plan = []
    side = first
    for i in range(n_steps):
        other = 'l' if side == 'r' else 'r'
        last = i == n_steps - 1
        target_x = feet[other][0] if last else feet[other][0] + step_length
        if i == 0:
            target_x = feet[other][0] + step_length / 2.0
        start = list(feet[side])
        end = [target_x, feet[side][1]]
        plan.append((t, t + SINGLE_SUPPORT, side, start, end))
        feet[side] = end
        t += SINGLE_SUPPORT + DOUBLE_SUPPORT
        side = other
    return plan, t + FIRST_SHIFT


def zmp_reference(plan, t_end, dt=DT):
    """
    ZMP reference samples (x, y) for the plan, plus a tail for the preview.

    Waypoints, linearly interpolated: centred between the feet; ramping
    onto the stance foot during the double support before each step
    (FIRST_SHIFT for the first, after START_HOLD); on the stance foot through single
    support; centred again after the last step.
    """
    feet = {'l': [0.0, FOOT_Y], 'r': [0.0, -FOOT_Y]}

    def centre():
        return [(feet['l'][0] + feet['r'][0]) / 2 + ZMP_X_OFFSET,
                (feet['l'][1] + feet['r'][1]) / 2]

    events = [(0.0, centre())]
    for i, (t0, t1, swing, _, end) in enumerate(plan):
        stance = 'l' if swing == 'r' else 'r'
        inset = ZMP_Y_INSET if stance == 'r' else -ZMP_Y_INSET
        point = [feet[stance][0] + ZMP_X_OFFSET, feet[stance][1] + inset]
        ramp_start = t0 - (FIRST_SHIFT if i == 0 else DOUBLE_SUPPORT)
        if ramp_start > events[-1][0] + 1e-9:
            events.append((ramp_start, events[-1][1]))
        events += [(t0, point), (t1, point)]
        feet[swing] = list(end)
    events.append((events[-1][0] + DOUBLE_SUPPORT, centre()))
    events.append((t_end + PREVIEW + 1.0, centre()))
    n = int(round((t_end + PREVIEW) / dt)) + 1
    times = np.arange(n) * dt
    xp = [e[0] for e in events]
    return np.stack([np.interp(times, xp, [e[1][axis] for e in events])
                     for axis in range(2)], axis=1)


class ZmpWalker:
    """
    Step through a footstep plan at DT; sample() gives targets for one tick.

    sample() returns (leg_angles, load_share, info): leg_angles maps joint
    names to angles (both legs), load_share = (left, right) fractions of
    body weight for the gravity feedforward, info = dict(t, zmp_ref, com,
    phase).
    """

    def __init__(self, n_steps=6, step_length=STEP_LENGTH):
        self.plan, self.t_end = footstep_plan(n_steps, step_length)
        self.ref = zmp_reference(self.plan, self.t_end)
        self.Ke, self.Kx, self.Gd, self.A, self.B, self.C = preview_gains()
        start = self.ref[0]
        self.state = np.array([[start[0], start[1]], [0.0, 0.0], [0.0, 0.0]])  # 3x2
        self.err_sum = np.zeros(2)
        self.k = 0
        self.feet = {'l': [0.0, FOOT_Y, 0.0], 'r': [0.0, -FOOT_Y, 0.0]}
        # Extra pelvis displacement (walk frame) from the node's CoM feedback.
        self.pelvis_shift = (0.0, 0.0)
        self.cop_in_foot = {'l': (ZMP_X_OFFSET, -ZMP_Y_INSET), 'r': (ZMP_X_OFFSET, ZMP_Y_INSET)}

    @property
    def done(self):
        return self.k * DT >= self.t_end

    @property
    def t(self):
        return self.k * DT

    def _step_com(self):
        n = len(self.Gd)
        future = self.ref[self.k + 1:self.k + 1 + n]
        if len(future) < n:
            future = np.vstack([future, np.repeat(future[-1:], n - len(future), axis=0)])
        zmp = (self.C @ self.state)[0]
        self.err_sum += zmp - self.ref[self.k]
        u = -self.Ke * self.err_sum - self.Kx @ self.state - self.Gd @ future
        self.state = self.A @ self.state + self.B @ u.reshape(1, 2)
        return zmp

    def _feet_at(self, t):
        phase = 'double'
        for t0, t1, swing, start, end in self.plan:
            if t < t0:
                break
            if t >= t1:
                self.feet[swing][:2] = end
                continue
            s = (t - t0) / (t1 - t0)
            ease = 0.5 - 0.5 * math.cos(math.pi * s)
            self.feet[swing] = [start[0] + (end[0] - start[0]) * ease,
                                start[1] + (end[1] - start[1]) * ease,
                                SWING_HEIGHT * math.sin(math.pi * s) ** 2]
            phase = f'swing_{swing}'
            break
        return phase

    def sample(self):
        """Advance one tick; return (leg_angles, load_share, info)."""
        zmp = self._step_com()
        com = self.state[0]
        phase = self._feet_at(self.t)
        pelvis = (com[0] - COM_IN_PELVIS[0] + self.pelvis_shift[0],
                  com[1] - COM_IN_PELVIS[1] + self.pelvis_shift[1])
        angles = {}
        self.feet_in_pelvis = {}
        for side, (fx, fy, fz) in self.feet.items():
            rel = (fx - pelvis[0], fy - pelvis[1], ANKLE_Z + fz)
            self.feet_in_pelvis[side] = rel
            ik = hg.leg_ik_3d(side, *rel)
            for joint, value in zip(('hpz', 'hpx', 'hpy', 'kny', 'aky', 'akx'), ik):
                angles[f'{side}_leg_{joint}'] = value
        if phase == 'swing_l':
            share = (0.0, 1.0)
        elif phase == 'swing_r':
            share = (1.0, 0.0)
        else:
            # Split by where the ZMP reference sits between the feet.
            ly, ry = self.feet['l'][1], self.feet['r'][1]
            w = min(1.0, max(0.0, (self.ref[self.k][1] - ry) / (ly - ry)))
            share = (w, 1.0 - w)
        # Planned centre of pressure under each foot, relative to its ankle:
        # the ZMP reference when it is on that foot, else mid-sole / inset.
        ref = self.ref[self.k]
        self.cop_in_foot = {}
        for side in 'lr':
            inset = ZMP_Y_INSET if side == 'r' else -ZMP_Y_INSET
            fx, fy = self.feet[side][0], self.feet[side][1]
            if phase == f'swing_{"r" if side == "l" else "l"}':
                self.cop_in_foot[side] = (ref[0] - fx, ref[1] - fy)
            else:
                self.cop_in_foot[side] = (ZMP_X_OFFSET, inset)
        info = {'t': self.t, 'zmp_ref': tuple(ref), 'zmp': tuple(zmp),
                'com': tuple(com), 'phase': phase}
        self.k += 1
        return angles, share, info


MASS = 179.7  # kg, atlas_v5 + SVH hands (the same model as COM_IN_PELVIS)


def gravity_feedforward(angles, feet_in_pelvis, share, cop_offset=(ZMP_X_OFFSET, 0.0),
                        mass=MASS):
    """
    Return {joint: torque} holding each leg's planned share of the weight.

    Static, from the commanded angles: the ground pushes up with
    N = share * m * g at the planned centre of pressure, `cop_offset` ahead
    of / beside the ankle. A pitch joint holds +r_x * N (axis +y) and a roll
    joint -r_y * N (axis +x), r being that point relative to the joint
    (pelvis level) -- the ankle included: leaving the ankle without it let
    the ankle sag under a single-support load and Atlas tip over its toes.
    Model-based on purpose: feeding back measured foot loads made a load ->
    torque -> lift oscillation.
    """
    torques = {}
    for side, frac in zip('lr', share):
        cx, cy = cop_offset[side] if isinstance(cop_offset, dict) else cop_offset
        load = frac * mass * G
        hpy, kny = angles[f'{side}_leg_hpy'], angles[f'{side}_leg_kny']
        knee = hg._rotate(hg.THIGH, hpy)
        ankle = hg.leg_fk(hpy, kny)
        cop_x = ankle[0] + cx
        torques[f'{side}_leg_hpy'] = cop_x * load
        torques[f'{side}_leg_kny'] = (cop_x - knee[0]) * load
        torques[f'{side}_leg_aky'] = cx * load
        hip_y = hg.SIDE_SIGN[side] * hg.HIP_Y
        torques[f'{side}_leg_hpx'] = -(feet_in_pelvis[side][1] + cy - hip_y) * load
        torques[f'{side}_leg_akx'] = -cy * load
    return torques
