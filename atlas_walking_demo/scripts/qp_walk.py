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
Walking under torque control: zmp_walk's plan, tracked by the whole-body QP.

Pure Python (no rclpy, no Pinocchio). QpWalk wraps a zmp_walk.ZmpWalker --
the same footstep planner and ZMP preview control as the position-mode
free walk -- and turns its plan into what TorqueBalance feeds the QP:
- a CoM reference (position, velocity, acceleration) and the planned ZMP,
  tracked with DCM feedback: CMP = p_ref + (1 + k) (xi - xi_ref);
- the planned contacts: the swing foot leaves the contact set for its
  swing and gets a sole target (position, heading) instead;
- the pelvis heading.
No leg IK: the QP finds the joint motion. That keeps the hands free for
their own tasks (carrying a box while walking).

Frames: the walker plans in its "walk frame" (origin between the feet,
x forward). QpWalk anchors that frame to the QP's world frame at the
soles' midpoint and heading when it is created, and starts the walker
from the real feet and CoM.
"""

import math

import numpy as np
import zmp_walk as zw

SOLE_X = 0.023      # sole centre ahead of the ankle (wbc.SOLE)
LAND_WAIT = 0.06    # s after a planned touchdown the foot counts as down anyway


def _rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


class QpWalk:
    """
    A walking plan for the torque QP, anchored at the current feet.

    soles: {side: (sole position (3,), yaw)} in the QP world frame;
    com: the current CoM (3,) in that frame. Drive it like ZmpWalker
    (set_velocity, stop); call advance(t) with the simulation time.
    """

    def __init__(self, soles, com, t):
        (pl, yl), (pr, yr) = soles['l'], soles['r']
        self.origin = (np.asarray(pl[:2]) + np.asarray(pr[:2])) / 2
        self.yaw0 = yr + zw._wrap(yl - yr) / 2
        self.ground = min(pl[2], pr[2])
        self.walker = w = zw.ZmpWalker()
        for side, (pos, yaw) in soles.items():
            x, y = self._to_walk(pos[:2])
            dyaw = zw._wrap(yaw - self.yaw0)
            ax, ay = _rot(-SOLE_X, 0.0, dyaw)  # sole -> ankle
            w.feet[side] = [x + ax, y + ay, 0.0, dyaw]
            w.planned[side] = (x + ax, y + ay, dyaw)
        # Start at rest: the ZMP reference where the CoM is now. Starting it
        # between the feet while a held box had pulled the CoM forward made
        # the preview's first jerk a huge acceleration; the QP failed and
        # Atlas fell within 10 ms (measured).
        here = self._to_walk(com[:2])
        w.events = [(0.0, here)]
        w.state = np.array([list(here), [0.0, 0.0], [0.0, 0.0]])
        self.t0 = t
        self.walk_t = t       # sim time of the walker's current sample
        self.landed = {}      # side -> time its swing ended
        self.phase = 'double'
        self.prev_phase = 'double'
        # Optional swing-foot style: f(s) -> (sole height above the ground,
        # pitch) for swing progress s in [0, 1], instead of the walker's arc.
        self.swing_style = None

    # --- frames --------------------------------------------------------

    def _to_walk(self, xy):
        d = np.asarray(xy) - self.origin
        return _rot(d[0], d[1], -self.yaw0)

    def _to_world(self, xy):
        x, y = _rot(xy[0], xy[1], self.yaw0)
        return self.origin + np.array([x, y])

    # --- commands ------------------------------------------------------

    def set_velocity(self, vx, vy, wz):
        """Walk at (vx, vy, wz) in the robot's frame (m/s, rad/s)."""
        self.walker.set_velocity(vx, vy, wz)

    def stop(self):
        """Finish the planned steps and bring the feet side by side."""
        self.walker.stop()

    @property
    def idle(self):
        return self.walker.idle

    @property
    def steps_planned(self):
        return self.walker.planned_count

    # --- per tick ------------------------------------------------------

    def advance(self, t):
        """Step the walker up to simulation time t (it runs at zw.DT)."""
        while self.walk_t + zw.DT <= t + 1e-9:
            self.walker.sample()
            self.walk_t += zw.DT
            phase = self.walker.phase
            if self.prev_phase.startswith('swing') and phase != self.prev_phase:
                self.landed[self.prev_phase[-1]] = self.walk_t
            self.prev_phase = phase
        self.phase = self.walker.phase

    def com_reference(self, t):
        """Return (c_ref, v_ref, a_ref, p_ref) in the world frame (x, y), at time t."""
        w = self.walker
        c, v, a = w.state[0], w.state[1], w.state[2]
        dt = max(0.0, t - self.walk_t)
        c = c + v * dt
        p = c - zw.COM_HEIGHT / zw.G * a
        rv = np.array(_rot(v[0], v[1], self.yaw0))
        ra = np.array(_rot(a[0], a[1], self.yaw0))
        return self._to_world(c), rv, ra, self._to_world(p)

    def contacts(self, loaded):
        """
        Return the feet the QP may push on: the planned stance.

        A foot whose swing just ended joins once its load cell sees it
        (`loaded`, from TorqueBalance's hysteresis) or LAND_WAIT later.
        """
        swing = self.phase[-1] if self.phase.startswith('swing') else None
        out = ''
        for side in 'lr':
            if side == swing:
                continue
            landed = self.landed.get(side)
            if landed is not None and self.walk_t - landed < LAND_WAIT and side not in loaded:
                continue
            out += side
        return out or 'lr'

    def swing_progress(self):
        """Return how far the current swing is, 0..1 (None in double support)."""
        w = self.walker
        for t0, t1, *_ in w.steps:
            if t0 <= w.t < t1:
                return (w.t - t0) / (t1 - t0)
        return None

    def foot_targets(self, contacts):
        """Return {side: (sole position (3,), yaw, pitch)} for the feet not in contacts."""
        out = {}
        for side in 'lr':
            if side in contacts:
                continue
            x, y, z, yaw = self.walker.feet[side]
            sx, sy = _rot(SOLE_X, 0.0, yaw)
            pos = self._to_world((x + sx, y + sy))
            pitch = 0.0
            if self.phase == f'swing_{side}' and self.swing_style is not None:
                # The walker's clock is one tick ahead of its phase: on the
                # swing's last tick no step contains it any more.
                s = self.swing_progress()
                z, pitch = self.swing_style(1.0 if s is None else s)
            # Not yet loaded after the planned touchdown: press a little.
            down = -0.01 if not self.phase.startswith('swing') else 0.0
            out[side] = (np.array([pos[0], pos[1], self.ground + z + down]),
                         zw._wrap(yaw + self.yaw0), pitch)
        return out

    def heading(self):
        """Return the planned pelvis yaw in the world frame."""
        return zw._wrap(self.walker.pelvis[2] + self.yaw0)


class Route:
    """
    A scripted walk: segments of (vx, vy, wz, steps), then stop.

    Call update(walk) after every walk.advance(); it switches segments as
    the planner commits to steps and returns True once Atlas stands again.
    The walker is open-loop on footsteps, so a route's end pose is known in
    advance (predict()).
    """

    def __init__(self, segments):
        self.segments = list(segments)
        self.index = -1
        self.base = 0
        self.stopped = False
        self.done = False

    def update(self, walk):
        if self.done:
            return True
        if not self.stopped and (self.index < 0 or walk.steps_planned - self.base >=
                                 self.segments[self.index][3]):
            self.index += 1
            if self.index < len(self.segments):
                vx, vy, wz, _ = self.segments[self.index]
                walk.set_velocity(vx, vy, wz)
                self.base = walk.steps_planned
            else:
                walk.stop()
                self.stopped = True
        if self.stopped and walk.idle:
            self.done = True
        return self.done


def nominal_soles():
    """Return the standing soles {side: (position, yaw)}, origin between them."""
    return {side: (np.array([SOLE_X, sign * zw.FOOT_Y, 0.0]), 0.0)
            for side, sign in (('l', 1.0), ('r', -1.0))}


def predict(segments, soles=None, max_time=200.0):
    """
    Return (soles, duration): where a route leaves the feet, and how long it takes.

    soles: {side: (sole position (3,), yaw)}, by default nominal_soles();
    the result is in the same frame.
    """
    soles = soles or nominal_soles()
    mid = (soles['l'][0] + soles['r'][0]) / 2
    walk = QpWalk(soles, mid, 0.0)
    route = Route(segments)
    t = 0.0
    while not route.update(walk):
        t += zw.DT
        walk.advance(t)
        if t > max_time:
            raise RuntimeError('route did not finish')
    out = {}
    for side in 'lr':
        x, y, _, yaw = walk.walker.feet[side]
        sx, sy = _rot(SOLE_X, 0.0, yaw)
        pos = walk._to_world((x + sx, y + sy))
        out[side] = (np.array([pos[0], pos[1], walk.ground]), zw._wrap(yaw + walk.yaw0))
    return out, t
