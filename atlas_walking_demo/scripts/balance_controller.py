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
Free-standing (no harness) walking controller for Atlas, without rclpy.

FreeWalkController.update(state, t) turns one atlas_msgs/AtlasState into
one AtlasCommand's worth of arrays, at zmp_walk.DT of simulation time:

- takeover: AtlasPlugin's setpoint, reconstructed as position + effort/kp
  (see walk_keyboard.py), blended into the crouched stance over CROUCH_TIME;
- joint targets from zmp_walk.ZmpWalker (ZMP preview control + leg IK),
  driven by set_velocity()/stop();
- gravity feedforward in AtlasCommand.effort (planned load split);
- ankle stabilizer on the loaded legs from the pelvis IMU:
  aky += KP * pitch + KD * pitch_rate (roll likewise on akx and, scaled by
  load, on hpx) -- leaning back (pitch < 0) needs negative aky, measured;
- CoM feedback: the CoM measured from leg kinematics + IMU, relative to the
  loaded feet, against the plan; the pelvis target shifts to cancel the
  error. Laterally this is what makes it walk: without it a sideways sway
  grew step by step until Atlas fell after 2-4 steps.

Past FALL_TILT it stops commanding (status 'fallen'). recover() gets it back
up with VRCPlugin's "recover" mode: the returned Output asks the node to
publish that mode (upright spring harness at the spawn height), the legs
blend into the stance, the harness lowers onto the feet (Output.harness_vz
on atlas/cmd_vel) and "nominal" lets go.
"""

from dataclasses import dataclass
import math

import harness_gait as hg
import zmp_walk as zw

N = hg.ATLAS_JOINT_NAMES
CROUCH_TIME = 3.0
SETTLE_TIME = 2.0
FALL_TILT = math.radians(35)
GAIN_OVERRIDES = {'hpz': (1000.0, 10.0), 'hpx': (2500.0, 10.0), 'akx': (1000.0, 3.0)}
# Recovery timeline (s after recover()): blend the legs into the stance
# in the air, lower the harness until the legs carry the weight, hold, let
# go.
RECOVER_BLEND = 1.5
RECOVER_LOWER = 1.5
RECOVER_RELEASE = 4.0
RECOVER_Z = 0.90 + 0.15   # VRCPlugin's "recover" height: atlas.launch.py's default z + lift
SOLE_BELOW_ANKLE = 0.074  # m (atlas_v5 URDF)
STAND_Z = SOLE_BELOW_ANKLE - zw.ANKLE_Z  # pelvis height in the stance: 0.911 m, measured
RECOVER_PRELOAD = 0.005   # harness this far below standing height before letting go
DEFAULT_PARAMS = {
    'stabilizer_kp': 0.3, 'stabilizer_kd': 0.02,
    'stabilizer_kp_roll': 0.3, 'stabilizer_kd_roll': 0.02,
    'hip_roll_kp': 0.5, 'hip_roll_kd': 0.03,
    'com_kp': 0.0, 'com_kd': 0.05, 'com_kp_y': 1.0, 'com_kd_y': 0.1,
}


@dataclass
class Output:
    """One tick's command: AtlasCommand arrays plus requests for VRCPlugin."""

    position: list
    effort: list
    kp: list
    kd: list
    mode: str = None         # publish on atlas/mode (once) if set
    harness_vz: float = 0.0  # publish on atlas/cmd_vel linear.z if nonzero


def rpy(q):
    """Return (roll, pitch) of a quaternion."""
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))


class FreeWalkController:
    """
    Balance and walk Atlas free-standing; see the module docstring.

    Construct from the first full AtlasState; call update(state, t) every
    zmp_walk.DT with t in seconds of simulation time. status is one of
    'crouch', 'settle', 'ready' (walk commands accepted), 'fallen',
    'recovering'.
    """

    def __init__(self, state, params=None):
        self.params = dict(DEFAULT_PARAMS, **(params or {}))
        self.base = [p + e / k if k > 0 else p
                     for p, e, k in zip(state.position, state.effort, state.kp_position)]
        self.gains_p = list(state.kp_position)
        self.gains_d = list(state.kd_position)
        for i, name in enumerate(N):
            joint = name.rsplit('_', 1)[1]
            if '_leg_' in name and joint in GAIN_OVERRIDES:
                self.gains_p[i], self.gains_d[i] = GAIN_OVERRIDES[joint]
        self.status = 'crouch'
        self.t_start = None
        self.fall_info = ''
        self._velocity = (0.0, 0.0, 0.0)
        self._reset_walker(blend_from=list(self.base))

    def _reset_walker(self, blend_from, blend_time=CROUCH_TIME):
        self.walker = zw.ZmpWalker()
        self.com_err = None
        self.com_rate = (0.0, 0.0)
        self.blend_from = blend_from
        self.blend_time = blend_time
        self.angles, self.share, self.info = self.walker.sample()

    # --- commands -------------------------------------------------------

    def set_velocity(self, vx, vy, wz):
        """Walk at (vx, vy, wz) once ready; remembered until then."""
        self._velocity = (vx, vy, wz)
        if self.status == 'ready':
            self.walker.set_velocity(vx, vy, wz)

    def stop(self):
        """Stop walking (finishes with the feet together)."""
        self._velocity = (0.0, 0.0, 0.0)
        self.walker.stop()

    def recover(self):
        """Start getting up after a fall (no-op unless fallen)."""
        if self.status == 'fallen':
            self.status = 'recovering'
            self.t_recover = None

    @property
    def walking(self):
        return not self.walker.idle

    # --- control --------------------------------------------------------

    def _target(self, angles):
        """Return the full 30-joint target: legs from `angles`, arms at rest."""
        cmd = list(self.base)
        for name, value in angles.items():
            cmd[N.index(name)] = value
        for name, delta in hg.ARM_REST.items():
            cmd[N.index(name)] = self.base[N.index(name)] + delta
        return cmd

    def update(self, state, t):
        """Return this tick's Output, or None to stop commanding (fallen)."""
        if self.t_start is None:
            self.t_start = t
        roll, pitch = rpy(state.orientation)
        if self.status == 'recovering':
            return self._recover_tick(state, t)
        if self.status == 'fallen':
            return None
        if abs(roll) > FALL_TILT or abs(pitch) > FALL_TILT:
            self.status = 'fallen'
            self.fall_info = (f'pitch {math.degrees(pitch):+.0f}, roll '
                              f'{math.degrees(roll):+.0f} deg, phase {self.walker.phase}')
            return None
        elapsed = t - self.t_start
        if self.status in ('crouch', 'settle'):
            if elapsed >= self.blend_time:
                self.status = 'settle'
            if elapsed >= self.blend_time + SETTLE_TIME:
                self.status = 'ready'
                if any(self._velocity):
                    self.walker.set_velocity(*self._velocity)
        balancing = self.status != 'crouch'
        if balancing:
            self._com_feedback(state, roll, pitch)
        self.angles, self.share, self.info = self.walker.sample()
        blend = min(1.0, elapsed / self.blend_time)
        target = self._target(self.angles)
        cmd = [a + blend * (b - a) for a, b in zip(self.blend_from, target)]
        effort = self._feedforward(blend)
        p = self.params
        rate_p, rate_r = state.angular_velocity.y, state.angular_velocity.x
        for side, frac in zip('lr', self.share):
            if frac > 0.2:
                cmd[N.index(f'{side}_leg_aky')] += (
                    p['stabilizer_kp'] * pitch + p['stabilizer_kd'] * rate_p)
                cmd[N.index(f'{side}_leg_akx')] += (
                    p['stabilizer_kp_roll'] * roll + p['stabilizer_kd_roll'] * rate_r)
                # Hip strategy for roll: the torso droops toward the swing
                # side through the stance hip, so correct it there too.
                cmd[N.index(f'{side}_leg_hpx')] += frac * (
                    p['hip_roll_kp'] * roll + p['hip_roll_kd'] * rate_r)
        return Output(cmd, effort, self.gains_p, self.gains_d)

    def _feedforward(self, scale):
        effort = [0.0] * len(N)
        for name, torque in zw.gravity_feedforward(
                self.angles, self.walker.feet_in_pelvis, self.share,
                self.walker.cop_in_foot).items():
            effort[N.index(name)] = scale * torque
        return effort

    def _com_feedback(self, state, roll, pitch):
        """
        Shift the pelvis command to cancel the CoM's drift from the plan.

        Measured: the CoM relative to each loaded ankle, from the measured
        leg angles and the IMU tilt, blended by load share -- switching the
        reference foot outright turned each landing's placement error into
        a step in the feedback. Planned: the walker's CoM relative to the
        same feet. Both in the pelvis yaw frame.
        """
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        err = [0.0, 0.0]
        for side, frac in zip('lr', self.share):
            if frac <= 0.0:
                continue
            q = [state.position[N.index(f'{side}_leg_{j}')] for j in ('hpz', 'hpx', 'hpy', 'kny')]
            ankle = hg.leg_fk_3d(side, *q)
            v = [zw.COM_IN_PELVIS[0] - ankle[0], zw.COM_IN_PELVIS[1] - ankle[1],
                 zw.COM_HEIGHT_IN_PELVIS - ankle[2]]
            v = [cp * v[0] + sp * sr * v[1] + sp * cr * v[2], cr * v[1] - sr * v[2]]
            ref = self.walker.com_from_foot(side)
            err = [e + frac * (m - r) for e, m, r in zip(err, v, ref)]
        if self.com_err is not None:
            alpha = 0.1
            self.com_rate = tuple((1 - alpha) * r + alpha * (e - q) / zw.DT
                                  for r, e, q in zip(self.com_rate, err, self.com_err))
        self.com_err = tuple(err)
        p = self.params
        gains = ((p['com_kp'], p['com_kd']), (p['com_kp_y'], p['com_kd_y']))
        self.walker.pelvis_shift = tuple(
            max(-0.05, min(0.05, -(kp * e + kd * r)))
            for (kp, kd), e, r in zip(gains, self.com_err, self.com_rate))

    def _recover_tick(self, state, t):
        mode = None
        if self.t_recover is None:
            self.t_recover = t
            self.recover_from = list(state.position)
            self._reset_walker(blend_from=list(state.position))
            mode = 'recover'
        tr = t - self.t_recover
        target = self._target(self.angles)
        blend = min(1.0, tr / RECOVER_BLEND)
        cmd = [a + blend * (b - a) for a, b in zip(self.recover_from, target)]
        # Down to just below standing height: a harness too low presses
        # Atlas into the floor (measured: 2140 N on the feet for 1760 N of
        # weight) and it fell back on release.
        lower = RECOVER_BLEND <= tr < RECOVER_BLEND + RECOVER_LOWER
        vz = (STAND_Z - RECOVER_PRELOAD - RECOVER_Z) / RECOVER_LOWER if lower else 0.0
        effort = self._feedforward(min(1.0, max(0.0, (tr - RECOVER_BLEND) / RECOVER_LOWER)))
        if tr >= RECOVER_RELEASE:
            mode = 'nominal'
            self.status = 'settle'
            self._reset_walker(blend_from=target, blend_time=1e-3)
            self.t_start = t
        return Output(cmd, effort, self.gains_p, self.gains_d, mode, vz)
