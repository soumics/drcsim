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
import wbc

OMEGA = math.sqrt(9.81 / 1.12)  # LIPM natural frequency (1/s)
CONTACT_ON = 60.0               # N on a foot's load cell: in contact
CONTACT_OFF = 20.0              # N: lifted
DEFAULTS = {
    'k_dcm': 2.0,          # DCM feedback gain
    'kp_height': 30.0, 'kd_height': 11.0,
    'w_com': 1000.0, 'w_rot': 10.0, 'w_post': 1.0,
    'w_foot': 5000.0, 'kp_foot': 400.0, 'kd_foot': 40.0,
    'kd_joint': 1.0,       # AtlasPlugin joint damping in torque mode (N m s/rad)
    'com_ahead': 0.02,     # CoM target ahead of the soles' midpoint (m)
}


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

    def update(self, state):
        """Return the joint torques (AtlasCommand order) for this state."""
        contacts = self.contacts(state)
        q, v = self._state(state, contacts)
        m, d = self.wbc.model, self.wbc.data
        self.com = pin.centerOfMass(m, d, q, v).copy()
        self.vcom = d.vcom[0].copy()
        p = self.p
        xi = self.com[:2] + self.vcom[:2] / OMEGA
        cmp = xi + p['k_dcm'] * (xi - self.com_target[:2])
        acc = np.zeros(3)
        acc[:2] = OMEGA ** 2 * (self.com[:2] - cmp)
        acc[2] = (p['kp_height'] * (self.com_target[2] - self.com[2]) -
                  p['kd_height'] * self.vcom[2])
        return self.wbc.solve(q, v, acc, self.rot_target, self.posture, contacts,
                              w_com=p['w_com'], w_rot=p['w_rot'], w_post=p['w_post'],
                              w_foot=p['w_foot'], kp_foot=p['kp_foot'], kd_foot=p['kd_foot'])
