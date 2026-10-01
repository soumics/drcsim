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
Torque-controlled standing balance for Atlas (whole-body QP), no rclpy.

TorqueBalance.update(state) turns one atlas_msgs/AtlasState into joint
torques via wbc.WholeBodyController, with:
- a DCM (capture point) balance law: xi = c + cdot / omega, desired CMP
  r = xi + K_DCM (xi - xi_target), CoM acceleration omega^2 (c - r);
- contact detection from the foot load cells, with hysteresis (a foot lands
  above CONTACT_ON, lifts below CONTACT_OFF) -- a flag flipping at every
  bounce rocked Atlas from foot to foot;
- a task putting a lifted foot back down, level, where it left the ground.

Measured with precise pushes on the torso (0.2 s, through ros_gz_bridge;
docker_ws/push_compare.sh): survives 106-121 N s sideways, 75 N s forward
and backward; position (PD) control survives 90 sideways, 75 forward and
falls at 75 backward.
"""

import math

import numpy as np
import pinocchio as pin
import qp_walk
import wbc
import zmp_walk as zw

OMEGA = math.sqrt(9.81 / 1.12)  # LIPM natural frequency (1/s)
CONTACT_ON = 60.0               # N on a foot's load cell: in contact
CONTACT_OFF = 20.0              # N: lifted
DEFAULTS = {
    'k_dcm': 2.0,          # DCM feedback gain
    'kp_height': 30.0, 'kd_height': 11.0,
    'w_com': 1000.0, 'w_rot': 10.0, 'w_post': 1.0,
    'w_foot': 5000.0, 'kp_foot': 1600.0, 'kd_foot': 80.0,
    'w_hand': 200.0,       # palm tasks (manipulation)
    'kd_joint': 1.0,       # AtlasPlugin joint damping in torque mode (N m s/rad)
    'com_ahead': 0.02,     # CoM target ahead of the soles' midpoint (m)
    'sync_period_ms': 0,   # >0: ask AtlasPlugin to hold physics for fresh commands (lockstep)
    'trace': False,
    'stepping': True,      # take a recovery step when the capture point leaves the feet
    'step_margin': 0.0,    # how far outside the support polygon before stepping (m)
    'step_time': 0.35,     # s swing
    'step_height': 0.06,   # m
    'step_beyond': 0.15,   # m past the predicted capture point
    'step_settle': 0.4,        # s after touchdown before another step (unless far out)
    'step_margin_after': 0.05,  # m extra margin for 1 s after a landing
    'retarget_speed': 0.1,     # m/s the balance target walks to mid-stance after a step
    'step_persist': 0.015,     # s the capture point must stay outside before stepping
    'vcom_tau': 0.03,          # s low-pass on the CoM velocity estimate
    'plant_time': 0.5,     # s both feet count as planted after a landing
    'step_width': 0.18,    # m min sideways distance from the stance foot
    'step_max_x': (-0.35, 0.50),  # m swing sole from the stance sole, stance frame
    'step_max_y': 0.30,           # m max sideways
}
SOLE_CORNERS = [(dx, dy) for dx in (-0.09, 0.136) for dy in (-0.067, 0.067)]  # sole frame


class TorqueBalance:
    """
    Stand in torque mode; construct once the robot stands (position mode).

    The posture and the CoM height to hold are the ones at construction;
    the CoM target is over the middle of the soles (com_ahead forward).
    """

    def __init__(self, urdf, joint_names, state, params=None):
        self.p = dict(DEFAULTS, **(params or {}))
        self.wbc = wbc.WholeBodyController(urdf, joint_names)
        self.in_contact = {'l': True, 'r': True}
        q, _ = self._state(state)
        soles = self.wbc.sole_world
        self.posture = np.array(state.position)
        self.com_target = np.array([
            (soles['l'][0] + soles['r'][0]) / 2 + self.p['com_ahead'],
            (soles['l'][1] + soles['r'][1]) / 2,
            pin.centerOfMass(self.wbc.model, self.wbc.data, q)[2]])
        o = state.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y ** 2 + o.z ** 2))
        self.rot_target = pin.rpy.rpyToMatrix(0.0, 0.0, yaw)
        self.com = self.com_target.copy()
        self.vcom = np.zeros(3)
        self.step = None      # dict(side, start, end, t0) while stepping
        self.hand_targets = {}  # side -> (palm position, palm rotation), world frame
        self.walk = None        # qp_walk.QpWalk while walking
        self.squeeze = 0.0      # N each palm pushes along its normal (holding an object)
        self.landed_at = -1e9
        self.outside_since = None  # time the capture point left the feet
        self.vcom_f = np.zeros(3)  # low-passed CoM velocity
        self.t = 0.0
        self.messages = []    # events for the node to log

    def contacts(self, state):
        """Update and return the feet in contact ('lr', 'l', 'r'), with hysteresis."""
        for side, wrench in (('l', state.l_foot), ('r', state.r_foot)):
            fz = -wrench.force.z
            if fz > CONTACT_ON:
                self.in_contact[side] = True
            elif fz < CONTACT_OFF:
                self.in_contact[side] = False
        return ''.join(s for s in 'lr' if self.in_contact[s]) or 'lr'

    def _state(self, state, contacts='lr'):
        o, w = state.orientation, state.angular_velocity
        return self.wbc.state(np.array(state.position), np.array(state.velocity),
                              [o.x, o.y, o.z, o.w], [w.x, w.y, w.z], contacts)

    def start_walk(self, state, t):
        """Start walking (plan anchored at the current feet); drive it via self.walk."""
        q, _ = self._state(state)
        soles = self._soles(q)
        com = pin.centerOfMass(self.wbc.model, self.wbc.data, q)
        self.step = None
        self.walk = qp_walk.QpWalk(soles, com, t)
        # Standing, the balance holds the CoM off its target by a steady
        # offset where the model and the robot disagree (8 cm behind with a
        # 10 kg box held, measured): that offset is what keeps it balanced.
        # Starting the walk with the reference at the CoM dropped it, and
        # Atlas fell backward within a second (measured). Keep it, in the
        # body frame.
        bias = self.com_target[:2] - com[:2]
        bias *= min(1.0, 0.12 / max(np.linalg.norm(bias), 1e-9))
        yaw = self.walk.yaw0
        c, s = math.cos(-yaw), math.sin(-yaw)
        self.walk_bias = (c * bias[0] - s * bias[1], s * bias[0] + c * bias[1])
        return self.walk

    def end_walk(self):
        """Stop following the walking plan; balance over the feet where they are."""
        yaw = self.walk.heading()
        self.walk = None
        self.rot_target = pin.rpy.rpyToMatrix(0.0, 0.0, yaw)
        # The standing target, as at construction: between the soles,
        # com_ahead forward (the steady offset comes back by itself).
        soles = self.wbc.sole_world
        mid = (soles['l'][:2] + soles['r'][:2]) / 2
        self.com_target[:2] = mid + self.p['com_ahead'] * np.array([math.cos(yaw), math.sin(yaw)])
        self.mid_target = None

    def update(self, state, t):
        """Return the joint torques (AtlasCommand order) for this state at time t (s)."""
        self.t = t
        contacts = self.contacts(state)
        if self.walk is not None:
            self.walk.advance(t)
            contacts = self.walk.contacts(contacts)
            self.in_contact.update({s: s in contacts for s in 'lr'})
        elif self.step is None and t < getattr(self, 'planted_until', -1.0):
            # Just landed: both feet count as planted, so the QP brakes on the
            # new foot; the landed foot unloaded again and Atlas ran on over
            # the stance foot (measured).
            contacts = 'lr'
            self.in_contact.update({'l': True, 'r': True})
        if self.step is not None and self.walk is None:
            contacts = self._stepping_contacts(contacts)
        q, v = self._state(state, contacts)
        self.q = q
        m, d = self.wbc.model, self.wbc.data
        if getattr(self, '_retarget', False):
            self._retarget = False
            soles = self.wbc.sole_world
            self.mid_target = np.array([(soles['l'][0] + soles['r'][0]) / 2,
                                        (soles['l'][1] + soles['r'][1]) / 2])
            # Balance where the capture point already is (kept inside the new
            # support), then walk the target to the midpoint: snapping it
            # there at once yanked the body back past it (measured).
            corners = []
            for side in 'lr':
                pos = soles[side]
                corners += [(pos[0] + dx, pos[1] + dy) for dx, dy in SOLE_CORNERS]
            hull = zw._hull(corners)
            xi_now = self.com[:2] + self.vcom_f[:2] / OMEGA
            inner = zw._closest_on_hull((float(xi_now[0]), float(xi_now[1])), hull)
            self.com_target[:2] = inner
            self.posture = np.array(state.position)  # hold the new stance
            # A split stance can't reach the standing height; holding it
            # made the legs fight each other after a step.
            self.com_target[2] = self.com[2] if hasattr(self, 'com') else self.com_target[2]
        if getattr(self, 'mid_target', None) is not None and self.step is None:
            gap = self.mid_target - self.com_target[:2]
            dist = np.linalg.norm(gap)
            move = min(dist, self.p['retarget_speed'] *
                       max(0.0, min(0.01, t - getattr(self, '_t_retarget', t))))
            if dist > 1e-6:
                self.com_target[:2] += gap / dist * move
        self._t_retarget = t
        self.com = pin.centerOfMass(m, d, q, v).copy()
        self.vcom = d.vcom[0].copy()
        p = self.p
        # The kinematic velocity estimate jumps at contact changes (touchdown,
        # load moving between feet: -1.6 m/s glitches, measured); low-pass it.
        dt = max(1e-4, min(0.01, t - getattr(self, '_t_prev', t)))
        self._t_prev = t
        a = dt / (p['vcom_tau'] + dt)
        self.vcom_f = (1 - a) * self.vcom_f + a * self.vcom
        xi = self.com[:2] + self.vcom_f[:2] / OMEGA
        cmp = xi + p['k_dcm'] * (xi - self.com_target[:2])
        acc = np.zeros(3)
        acc[:2] = OMEGA ** 2 * (self.com[:2] - cmp)
        acc[2] = (p['kp_height'] * (self.com_target[2] - self.com[2]) -
                  p['kd_height'] * self.vcom[2])
        targets = None
        if p.get('trace') and round(t * 1000) % 50 == 0 and t - self.landed_at < 1.5:
            self.messages.append(
                f'trace t={t:.2f} c={contacts} step={self.step is not None} '
                f'com=({self.com[0]:+.3f},{self.com[1]:+.3f},{self.com[2]:+.3f}) '
                f'vcom=({self.vcom[0]:+.2f},{self.vcom[1]:+.2f}) xi=({xi[0]:+.3f},{xi[1]:+.3f}) '
                f'target=({self.com_target[0]:+.3f},{self.com_target[1]:+.3f}) '
                f'acc=({acc[0]:+.1f},{acc[1]:+.1f})')
        if self.walk is not None:
            # Track the walking plan: DCM feedback around the planned ZMP.
            c_ref, v_ref, _, p_ref = self.walk.com_reference(t)
            xi_ref = c_ref + v_ref / OMEGA
            yaw = self.walk.heading()
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = self.walk_bias
            bias = np.array([c * bx - s * by, s * bx + c * by])
            cmp = p_ref + (1 + p['k_dcm']) * (xi - xi_ref) - p['k_dcm'] * bias
            acc[:2] = OMEGA ** 2 * (self.com[:2] - cmp)
            self.rot_target = pin.rpy.rpyToMatrix(0.0, 0.0, self.walk.heading())
            targets = self.walk.foot_targets(contacts)
        elif self.step is None and p['stepping']:
            self._check_step(q, xi, contacts)
        if self.step is not None and self.walk is None:
            # Replan the landing spot from the current capture point until
            # 70% of the swing: the push may still be acting when the step
            # starts, and a target fixed then landed far short (measured).
            st = self.step
            elapsed = t - st['t0']
            if elapsed < 0.7 * p['step_time']:
                soles = self._soles(q)
                remaining = p['step_time'] + 0.1 - elapsed
                st['end'] = self._step_target(soles, st['side'], (float(xi[0]), float(xi[1])),
                                              remaining)
            targets = {st['side']: self._swing_point()}
        self.last_acc = acc
        tau = self.wbc.solve(q, v, acc, self.rot_target, self.posture, contacts,
                             w_com=p['w_com'], w_rot=p['w_rot'], w_post=p['w_post'],
                             w_foot=p['w_foot'], kp_foot=p['kp_foot'], kd_foot=p['kd_foot'],
                             foot_targets=targets, hand_targets=self.hand_targets or None,
                             w_hand=p['w_hand'], squeeze=self.squeeze)
        return tau

    # --- stepping ----------------------------------------------------

    def _soles(self, q):
        """Return {side: (sole centre, yaw)} in the world."""
        m, d = self.wbc.model, self.wbc.data
        pin.framesForwardKinematics(m, d, q)
        out = {}
        for side, f in self.wbc.feet.items():
            R = d.oMf[f].rotation
            out[side] = (d.oMf[f].translation.copy(), math.atan2(R[1, 0], R[0, 0]))
        return out

    def _check_step(self, q, xi, contacts):
        """Start a recovery step if the capture point xi has left the loaded feet."""
        soles = self._soles(q)
        # Support: every foot on the ground, loaded or not -- with the
        # loaded feet only, a capture point between the feet looked
        # "outside" after each touchdown and the other foot stepped back to it.
        ground_z = min(pos[2] for pos, _ in soles.values())
        on_ground = [c for c in 'lr' if soles[c][0][2] < ground_z + 0.02 and
                     (c in contacts or self.step is None)]
        corners = []
        for pos, yaw in (soles[c] for c in on_ground):
            c, s = math.cos(yaw), math.sin(yaw)
            corners += [(pos[0] + c * dx - s * dy, pos[1] + s * dx + c * dy)
                        for dx, dy in SOLE_CORNERS]
        hull = zw._hull(corners)
        xi = (float(xi[0]), float(xi[1]))
        if False:
            self.messages.append(
                f'trace t={self.t:.2f} contacts={contacts} xi=({xi[0]:+.3f},{xi[1]:+.3f}) '
                f'com=({self.com[0]:+.3f},{self.com[1]:+.3f}) vcom=({self.vcom[0]:+.2f},'
                f'{self.vcom[1]:+.2f}) inside={zw._inside(xi, hull)} hull_x=('
                f'{min(c[0] for c in hull):+.2f},{max(c[0] for c in hull):+.2f}) hull_y=('
                f'{min(c[1] for c in hull):+.2f},{max(c[1] for c in hull):+.2f})')
        # Just after a landing the body settles onto a split stance whose
        # support polygon is thin; small excursions there chained needless
        # second steps (measured), so ask for more before stepping again.
        margin = self.p['step_margin'] + (self.p['step_margin_after']
                                          if self.t - self.landed_at < 1.0 else 0.0)
        if zw._inside(xi, hull, -margin):
            self.outside_since = None
            return
        p = self.p
        if self.outside_since is None:
            self.outside_since = self.t
        if self.t - self.outside_since < p['step_persist']:
            return
        # Right after a touchdown the body is still settling onto the new
        # stance; stepping again at once chained steps (measured).
        if (self.t - self.landed_at < p['step_settle'] and
                zw._inside(xi, hull, -0.10)):
            return
        # Swing the foot farther from xi (the unloaded one); the stance
        # foot's edge is where the ZMP sits, xi diverges from there.
        far = {s: math.hypot(xi[0] - pos[0], xi[1] - pos[1]) for s, (pos, _) in soles.items()}
        side = max(far, key=far.get)
        end = self._step_target(soles, side, xi, p['step_time'] + 0.1)
        start = soles[side][0]
        self.step = {'side': side, 'start': start, 'end': end, 't0': self.t}
        self.outside_since = None
        self.messages.append(
            f'Capture point ({xi[0]:+.2f}, {xi[1]:+.2f}) left the feet: stepping '
            f'{"left" if side == "l" else "right"} foot to ({end[0]:+.2f}, {end[1]:+.2f}).')

    def _step_target(self, soles, side, xi, horizon):
        """Return the swing sole target for capture point xi, `horizon` s ahead."""
        p = self.p
        stance = 'r' if side == 'l' else 'l'
        # During the swing Atlas stands on the stance foot alone: xi diverges
        # from that foot's edge -- sideways too. Predicting from both feet
        # missed the sideways drift and the foot landed too narrow (measured).
        spos0, syaw0 = soles[stance]
        c0, s0 = math.cos(syaw0), math.sin(syaw0)
        stance_hull = zw._hull([(spos0[0] + c0 * dx - s0 * dy, spos0[1] + s0 * dx + c0 * dy)
                                for dx, dy in SOLE_CORNERS])
        edge = zw._closest_on_hull(xi, stance_hull)
        grow = math.exp(OMEGA * horizon)
        v = np.array(xi) - np.array(edge)
        n = np.linalg.norm(v) or 1.0
        # Past the prediction along the CoM's travel: along (xi - edge) the
        # margin pointed at the stance foot's corner -- sideways on a
        # forward push -- and the step landed short (measured).
        heading = self.vcom_f[:2]
        hn = np.linalg.norm(heading)
        heading = heading / hn if hn > 0.05 else v / n
        target = np.array(edge) + v * grow + p['step_beyond'] * heading
        spos, syaw = soles[stance]
        # Keep it within reach of the stance foot, and not onto it.
        rel = target - spos[:2]
        c, s = math.cos(-syaw), math.sin(-syaw)
        lx, ly = c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]
        sign = 1.0 if side == 'l' else -1.0
        # Mostly fore/aft travel: normal stance width, the prediction only
        # sets the step length. (The push hits the torso high up; its first
        # instants spike the sideways estimate, and a forward push became a
        # crossover step -- measured.)
        vel = self.vcom_f[:2]
        if abs(vel[1]) < 0.5 * abs(vel[0]):
            ly = sign * 2 * zw.FOOT_Y
        # Limit each direction on its own: a combined reach limit shortened
        # forward steps when the sideways prediction was large (measured).
        lx = min(p['step_max_x'][1], max(p['step_max_x'][0], lx))
        if sign * ly >= 0.0:  # its own side of the stance foot, 0.18-0.30 m
            ly = sign * min(p['step_max_y'], max(p['step_width'], sign * ly))
        else:  # across it (pushed toward the stance foot): cross in front
            lx = max(lx, 0.25)
            ly = sign * max(-0.15, sign * ly)
        c, s = math.cos(syaw), math.sin(syaw)
        end = spos.copy()
        end[0] += c * lx - s * ly
        end[1] += s * lx + c * ly
        return end

    def _swing_point(self):
        """Swing-foot sole target now: ease across, arc up, land slightly low."""
        st = self.step
        s = min(1.0, (self.t - st['t0']) / self.p['step_time'])
        # Forward first, sideways later: a crossover clears the stance foot.
        ex = 0.5 - 0.5 * math.cos(math.pi * min(1.0, 1.5 * s))
        ey = 0.5 - 0.5 * math.cos(math.pi * min(1.0, max(0.0, (s - 0.2) / 0.7)))
        point = st['start'].copy()
        point[0] += (st['end'][0] - st['start'][0]) * ex
        point[1] += (st['end'][1] - st['start'][1]) * ey
        # Down by 85% of the swing (the foot trails its target), then press.
        sz = min(1.0, s / 0.85)
        point[2] = st['start'][2] + self.p['step_height'] * math.sin(math.pi * sz) ** 2
        if sz >= 1.0:
            point[2] = st['end'][2] - 0.05  # press down until the load cell sees contact
        return point

    def _stepping_contacts(self, measured):
        """Contacts during a step: the swing foot is free until it lands."""
        st = self.step
        side = st['side']
        stance = 'r' if side == 'l' else 'l'
        elapsed = self.t - st['t0']
        # Only once the planned path is down: a mid-swing scuff counted as a
        # (short) landing otherwise (measured).
        landed = elapsed > 0.85 * self.p['step_time'] and self.in_contact[side]
        if not landed and elapsed < 3 * self.p['step_time']:
            self.in_contact[side] = False
            return stance
        # Landed: new stance, new balance target between the feet.
        self.step = None
        self.landed_at = self.t
        self.messages.append(f'Step landed after {elapsed:.2f} s.')
        self.planted_until = self.t + self.p['plant_time']
        self._retarget = True
        return measured
