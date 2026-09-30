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

STEP_LENGTH = 0.15       # m, one foot ahead of the other (0.25 walks too; 0.30 falls)
SINGLE_SUPPORT = 0.8     # s
DOUBLE_SUPPORT = 0.4     # s
START_DELAY = PREVIEW    # s from a walk command to the weight shift: the preview must not
#                          see a change right away (a sudden change jerks the CoM)
FIRST_SHIFT = 1.0        # s shifting onto the first stance foot
SWING_HEIGHT = 0.05      # m
# Per-step limits: one foot ahead of the other, the leading foot's
# sideways step, its turn.
MAX_STEP = 0.25
MAX_STEP_BACK = 0.10
MAX_SIDE_STEP = 0.10
MAX_TURN_STEP = 0.25


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


def max_velocity():
    """Return (vx, vx_back, vy, wz) limits matching the per-step limits."""
    t_step = SINGLE_SUPPORT + DOUBLE_SUPPORT
    return (MAX_STEP / t_step, MAX_STEP_BACK / t_step,
            MAX_SIDE_STEP / t_step / 2, MAX_TURN_STEP / t_step / 2)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def _compose(pose, dx, dy, dyaw=0.0):
    """Return pose (x, y, yaw) moved by (dx, dy) in its own frame, then turned dyaw."""
    ox, oy = _rot(dx, dy, pose[2])
    return (pose[0] + ox, pose[1] + oy, pose[2] + dyaw)


def _inset(side):
    """ZMP reference offset from a foot's ankle, in the foot frame."""
    return (ZMP_X_OFFSET, -ZMP_Y_INSET if side == 'l' else ZMP_Y_INSET)


class ZmpWalker:
    """
    Velocity-driven walking pattern generator; call sample() every DT.

    set_velocity(vx, vy, wz) walks at that body velocity (walk frame,
    clamped to MAX_*); stop() finishes with the feet side by side. Footsteps
    are planned online, far enough ahead for the preview (a command takes
    effect after about PREVIEW + one step). Lateral steps and turns are
    taken by the leading foot only (the left one when going left), so the
    feet never cross.

    sample() returns (leg_angles, load_share, info): leg_angles maps joint
    names to angles (both legs), load_share = (left, right) fractions of
    body weight for the gravity feedforward, info = dict(t, zmp_ref, zmp,
    com, phase) with phase 'double', 'swing_l' or 'swing_r'.
    """

    def __init__(self):
        self.Ke, self.Kx, self.Gd, self.A, self.B, self.C = preview_gains()
        self.k = 0
        # Foot poses: [x, y, z, yaw] now; planned: (x, y, yaw) after all planned steps.
        self.feet = {'l': [0.0, FOOT_Y, 0.0, 0.0], 'r': [0.0, -FOOT_Y, 0.0, 0.0]}
        self.planned = {side: (f[0], f[1], f[3]) for side, f in self.feet.items()}
        self.body = (0.0, 0.0, 0.0)  # body pose after all planned steps
        self.steps = []              # [(t0, t1, swing, start, end)], start/end (x, y, yaw)
        self.events = [(0.0, self._centre())]
        self.command = (0.0, 0.0, 0.0)
        self.walking = False
        self.next_swing = None
        self.planned_count = 0       # steps planned so far, closing steps included
        start = self.events[0][1]
        self.state = np.array([[start[0], start[1]], [0.0, 0.0], [0.0, 0.0]])  # 3x2
        self.err_sum = np.zeros(2)
        # Extra pelvis displacement (pelvis frame) from the node's CoM feedback.
        self.pelvis_shift = (0.0, 0.0)
        self.pelvis = (0.0, 0.0, 0.0)
        self.feet_in_pelvis = {}
        self.cop_in_foot = {side: _inset(side) for side in 'lr'}
        self.phase = 'double'

    @property
    def t(self):
        return self.k * DT

    @property
    def idle(self):
        """Return True when standing still with nothing planned."""
        return not self.walking and (not self.steps or self.t > self.steps[-1][1] + DOUBLE_SUPPORT)

    def set_velocity(self, vx, vy, wz):
        """Walk at (vx, vy, wz): m/s forward/left, rad/s counterclockwise."""
        t_step = SINGLE_SUPPORT + DOUBLE_SUPPORT
        self.command = (max(-MAX_STEP_BACK, min(MAX_STEP, vx * t_step)) / t_step,
                        max(-MAX_SIDE_STEP, min(MAX_SIDE_STEP, 2 * vy * t_step)) / t_step / 2,
                        max(-MAX_TURN_STEP, min(MAX_TURN_STEP, 2 * wz * t_step)) / t_step / 2)
        if any(self.command):
            self.walking = True
        else:
            self.stop()

    def stop(self):
        """Finish walking: the next planned step brings the feet side by side."""
        if self.walking:
            self.walking = False
            self._plan_step(closing=True)
        self.command = (0.0, 0.0, 0.0)

    def _centre(self):
        points = [_compose(self.planned[s], *_inset(s))[:2] for s in 'lr']
        return ((points[0][0] + points[1][0]) / 2, (points[0][1] + points[1][1]) / 2)

    def _plan_step(self, closing=False):
        first = not self.steps or self.t > self.steps[-1][1] + DOUBLE_SUPPORT
        vx, vy, wz = self.command
        if first:
            if closing:
                return
            # Lead with the foot on the side it is heading (right if straight).
            self.next_swing = 'l' if (vy > 0.0 if vy else wz > 0.0) else 'r'
            t0 = max(self.t, self.events[-1][0]) + START_DELAY + FIRST_SHIFT
            ramp = FIRST_SHIFT
            self.body = self._mid_pose()
        else:
            t0 = self.steps[-1][1] + DOUBLE_SUPPORT
            ramp = DOUBLE_SUPPORT
        swing = self.next_swing
        stance = 'r' if swing == 'l' else 'l'
        t_step = SINGLE_SUPPORT + DOUBLE_SUPPORT
        if not closing:
            dy = 2 * vy * t_step if (vy > 0.0) == (swing == 'l') else 0.0
            dyaw = 2 * wz * t_step if (wz > 0.0) == (swing == 'l') else 0.0
            self.body = _compose(self.body, vx * t_step, dy, dyaw)
        sign = 1.0 if swing == 'l' else -1.0
        end = _compose(self.body, 0.0, sign * FOOT_Y)
        start = self.planned[swing]
        t1 = t0 + SINGLE_SUPPORT
        point = _compose(self.planned[stance], *_inset(stance))[:2]
        if t0 - ramp > self.events[-1][0] + 1e-9:
            self.events.append((t0 - ramp, self.events[-1][1]))
        self.events += [(t0, point), (t1, point)]
        self.steps.append((t0, t1, swing, start, end))
        self.planned[swing] = end
        self.next_swing = stance
        self.planned_count += 1
        if closing:
            self.events.append((t1 + DOUBLE_SUPPORT, self._centre()))

    def _mid_pose(self):
        (lx, ly, lyaw), (rx, ry, ryaw) = self.planned['l'], self.planned['r']
        return ((lx + rx) / 2, (ly + ry) / 2, ryaw + _wrap(lyaw - ryaw) / 2)

    def _reference(self, times):
        xp = [e[0] for e in self.events]
        return np.stack([np.interp(times, xp, [e[1][axis] for e in self.events])
                         for axis in range(2)], axis=1)

    def _step_com(self):
        n = len(self.Gd)
        times = self.t + DT * np.arange(n + 1)
        ref = self._reference(times)
        zmp = (self.C @ self.state)[0]
        self.err_sum += zmp - ref[0]
        u = -self.Ke * self.err_sum - self.Kx @ self.state - self.Gd @ ref[1:]
        self.state = self.A @ self.state + self.B @ u.reshape(1, 2)
        return zmp, ref[0]

    def _update_feet(self):
        phase = 'double'
        for t0, t1, swing, start, end in self.steps:
            if self.t < t0:
                break
            if self.t >= t1:
                self.feet[swing] = [end[0], end[1], 0.0, end[2]]
                continue
            s = (self.t - t0) / (t1 - t0)
            ease = 0.5 - 0.5 * math.cos(math.pi * s)
            self.feet[swing] = [start[0] + (end[0] - start[0]) * ease,
                                start[1] + (end[1] - start[1]) * ease,
                                SWING_HEIGHT * math.sin(math.pi * s) ** 2,
                                start[2] + _wrap(end[2] - start[2]) * ease]
            phase = f'swing_{swing}'
            break
        return phase

    def _prune(self):
        # Keep the step in progress and the events needed to interpolate from now.
        while self.steps and self.steps[0][1] < self.t - 1.0 and len(self.steps) > 1:
            self.steps.pop(0)
        while len(self.events) > 2 and self.events[1][0] < self.t - 1.0:
            self.events.pop(0)

    def sample(self):
        """Advance one tick; return (leg_angles, load_share, info)."""
        horizon = self.t + PREVIEW + SINGLE_SUPPORT + DOUBLE_SUPPORT
        while self.walking and (not self.steps or self.steps[-1][1] < horizon
                                or self.t > self.steps[-1][1] + DOUBLE_SUPPORT):
            self._plan_step()
        if not self.walking and self.steps and self.t > self.steps[-1][1] + DOUBLE_SUPPORT:
            self.steps.clear()
        self._prune()
        zmp, ref = self._step_com()
        com = self.state[0]
        phase = self.phase = self._update_feet()
        (lx, ly, _, lyaw), (rx, ry, _, ryaw) = self.feet['l'], self.feet['r']
        yaw = ryaw + _wrap(lyaw - ryaw) / 2
        off = _rot(COM_IN_PELVIS[0] - self.pelvis_shift[0],
                   COM_IN_PELVIS[1] - self.pelvis_shift[1], yaw)
        self.pelvis = (com[0] - off[0], com[1] - off[1], yaw)
        angles = {}
        for side, (fx, fy, fz, fyaw) in self.feet.items():
            rx_, ry_ = _rot(fx - self.pelvis[0], fy - self.pelvis[1], -yaw)
            rel = (rx_, ry_, ANKLE_Z + fz)
            self.feet_in_pelvis[side] = rel
            ik = hg.leg_ik_3d(side, *rel, _wrap(fyaw - yaw))
            for joint, value in zip(('hpz', 'hpx', 'hpy', 'kny', 'aky', 'akx'), ik):
                angles[f'{side}_leg_{joint}'] = value
        if phase == 'swing_l':
            share = (0.0, 1.0)
        elif phase == 'swing_r':
            share = (1.0, 0.0)
        else:
            # Split by where the ZMP reference sits between the feet.
            dx, dy = lx - rx, ly - ry
            w = ((ref[0] - rx) * dx + (ref[1] - ry) * dy) / (dx * dx + dy * dy)
            w = min(1.0, max(0.0, w))
            share = (w, 1.0 - w)
        # Planned centre of pressure under each foot, relative to its ankle
        # in the pelvis frame: the ZMP reference when it is on that foot
        # alone, else the inset point.
        for side in 'lr':
            fx, fy, _, fyaw = self.feet[side]
            if share == ((1.0, 0.0) if side == 'l' else (0.0, 1.0)):
                self.cop_in_foot[side] = _rot(ref[0] - fx, ref[1] - fy, -yaw)
            else:
                self.cop_in_foot[side] = _rot(*_inset(side), fyaw - yaw)
        info = {'t': self.t, 'zmp_ref': tuple(ref), 'zmp': tuple(zmp),
                'com': tuple(com), 'phase': phase}
        self.k += 1
        return angles, share, info

    def com_from_foot(self, side):
        """Return the planned CoM relative to a foot's ankle, in the pelvis yaw frame."""
        foot = self.feet[side]
        return _rot(self.state[0][0] - foot[0], self.state[0][1] - foot[1], -self.pelvis[2])


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
