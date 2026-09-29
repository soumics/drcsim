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
Foot-trajectory walking gait for Atlas in VRCPlugin's harness (no rclpy).

The pelvis is held by the harness ("pinned_with_gravity") and carried
forward through atlas/cmd_vel, so this gait never has to balance. Instead
of blending joint-space keyframes, it plans where each *ankle* should be
relative to the hip every tick and solves the leg's inverse kinematics:

* stance: the ankle moves backward at exactly the pelvis speed, so the foot
  stays planted on the floor without skating;
* swing: the ankle lifts off, follows a raised arc forward and lands half a
  step ahead of the hip;
* the harness is lowered CROUCH_DROP first, so the knees stay slightly
  bent through the whole cycle, as in human walking;
* each arm swings with the opposite leg.

Starting and stopping ramp the step length, so there are no jumps. Stopping
ends with one step in place that brings both feet back under the hips.
"""

import math

# The exact 30-joint order AtlasPlugin::Configure() builds -- see
# gait_controller.py.
ATLAS_JOINT_NAMES = [
    'back_bkz', 'back_bky', 'back_bkx', 'neck_ry',
    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
    'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx',
    'l_arm_wry2',
    'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx',
    'r_arm_wry2',
]

# Sagittal leg geometry from atlas_v5_raw.urdf, in the hip-pitch (hpy) joint
# frame, x forward / z up: l_leg_kny origin (-0.05, -0.374) on the thigh,
# l_leg_aky origin (0, -0.422) on the shin. At zero joint angles the ankle
# is at (-0.05, -0.796), directly under the pelvis origin (hpy sits 0.05 m
# ahead of it).
THIGH = (-0.05, -0.374)
SHIN = (0.0, -0.422)
ANKLE_X0 = THIGH[0] + SHIN[0]
LEG_Z0 = THIGH[1] + SHIN[1]

# Gait parameters (m, s, rad).
CROUCH_DROP = 0.025      # harness lowered this much: ~30 deg knee bend mid-stance,
#                          nearly straight at push-off (a 5 cm drop gave 49 deg)
CROUCH_DURATION = 3.0    # lowering the harness and the arms on takeover
STEP_LENGTH = 0.30       # ankle travel per stance, fore to aft
PERIOD = 1.2             # one full stride (left + right step)
DUTY = 0.6               # fraction of a stride each foot is on the floor
SWING_HEIGHT = 0.08      # ankle lift at mid-swing
RAMP_TIME = PERIOD       # step length ramps 0 <-> full over one stride
WALK_SPEED = STEP_LENGTH / (DUTY * PERIOD)  # pelvis speed at full step

# Arms: lowered from Atlas's zero T-pose, elbows slightly bent; shz swings
# them. Signs measured live (TF pelvis->hand): l_arm_shx -1.2 / r_arm_shx
# +1.2 lower the arms; negative l_arm_shz / positive r_arm_shz move that
# hand forward; ely 1.57 turns elx into a forward elbow bend.
ARM_REST = {
    'l_arm_shx': -1.3, 'r_arm_shx': 1.3,
    'l_arm_ely': 1.57, 'r_arm_ely': 1.57,
    'l_arm_elx': 0.5, 'r_arm_elx': -0.5,
}
ARM_SWING = 0.35  # shz amplitude at full step length

# Stance/swing phase offset per leg: the right leg is half a stride behind.
LEG_PHASE_OFFSET = {'l': 0.0, 'r': 0.5}


def _rotate(vec, angle):
    """Rotate (x, z) about the joint's +y axis, as the URDF joints do."""
    x, z = vec
    c, s = math.cos(angle), math.sin(angle)
    return (x * c + z * s, -x * s + z * c)


def leg_fk(hpy, kny):
    """Return the ankle (x, z) in the hip-pitch frame for the given angles."""
    thigh = _rotate(THIGH, hpy)
    shin = _rotate(SHIN, hpy + kny)
    return (thigh[0] + shin[0], thigh[1] + shin[1])


# Law of cosines for the knee: |thigh + R(kny) shin|^2 is
# |thigh|^2 + |shin|^2 + 2 * _KNEE_C * cos(kny - _KNEE_DELTA).
_THIGH_SQ = THIGH[0] ** 2 + THIGH[1] ** 2
_SHIN_SQ = SHIN[0] ** 2 + SHIN[1] ** 2
_KNEE_C = math.hypot(THIGH[0] * SHIN[1], THIGH[1] * SHIN[1])
_KNEE_DELTA = math.atan2(THIGH[0] * SHIN[1], THIGH[1] * SHIN[1])


def leg_ik(x, z, sole_pitch=0.0):
    """
    Return (hpy, kny, aky) putting the ankle at (x, z) in the hip-pitch frame.

    The knee always bends forward (kny >= its straight-leg value), and aky
    keeps the sole at `sole_pitch` (0 = parallel to the floor, positive =
    toe up). Targets beyond full leg extension are clamped to it.
    """
    reach_sq = x * x + z * z
    cos_arg = (reach_sq - _THIGH_SQ - _SHIN_SQ) / (2.0 * _KNEE_C)
    kny = _KNEE_DELTA + math.acos(max(-1.0, min(1.0, cos_arg)))
    # Ankle position with hpy = 0, then the hip angle that rotates it onto
    # the target direction (angle measured as atan2(x, z) about +y).
    ax, az = leg_fk(0.0, kny)
    hpy = math.atan2(x, z) - math.atan2(ax, az)
    hpy = math.atan2(math.sin(hpy), math.cos(hpy))
    aky = sole_pitch - hpy - kny
    return hpy, kny, aky


def _hermite(s, p0, m0, p1, m1):
    """Cubic Hermite from p0 (slope m0) to p1 (slope m1) over s in [0, 1]."""
    s2, s3 = s * s, s * s * s
    return ((2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0 +
            (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * m1)


class HarnessGait:
    """
    Tick-driven foot-trajectory gait; call sample(dt) at a steady rate.

    sample() returns (positions, (vx, vz)): the 30 joint targets in
    ATLAS_JOINT_NAMES order, and the pelvis velocity to send on atlas/cmd_vel
    (forward, up) so the harness moves in step with the feet.
    """

    def __init__(self, neutral_pose):
        self.neutral_pose = dict(neutral_pose)
        self.walking = False
        self._stop_requested = False
        self._time = 0.0
        self._state = 'CROUCH'
        self._phase = 0.0            # stride phase, 0..1
        self._amplitude = 0.0        # step length scale, 0..1
        self._settle = 0.0           # time spent stepping in place to stop
        self._ankle_x = {'l': ANKLE_X0, 'r': ANKLE_X0}
        self._lift_x = {'l': ANKLE_X0, 'r': ANKLE_X0}
        self._swinging = {'l': False, 'r': False}
        self._ankle_z = {'l': LEG_Z0 + CROUCH_DROP, 'r': LEG_Z0 + CROUCH_DROP}

    @property
    def phase_name(self):
        """Return 'CROUCH', 'STAND', 'WALK' or 'STOPPING'."""
        return self._state

    def start_walking(self):
        self._stop_requested = False
        self.walking = True
        if self._state == 'STAND':
            self._state = 'WALK'
        elif self._state == 'STOPPING':
            self._state = 'WALK'
            self._settle = 0.0

    def stop_walking(self):
        self._stop_requested = True
        if self._state == 'WALK':
            self._state = 'STOPPING'
        elif self._state == 'CROUCH':
            self.walking = False

    def sample(self, dt):
        self._time += dt
        vx, vz = 0.0, 0.0
        blend = 1.0
        if self._state == 'CROUCH':
            # Joint-space blend from the held setpoint to the crouched IK
            # stance: joint zero (kny = 0) lies on the other IK branch than
            # a forward-bent knee (the thigh's 5 cm offset makes kny = 0 and
            # kny ~= 0.27 the same leg length), so IK alone would snap.
            blend = min(1.0, self._time / CROUCH_DURATION)
            if blend < 1.0:
                vz = -CROUCH_DROP / CROUCH_DURATION
            else:
                self._state = 'WALK' if self.walking else 'STAND'
        elif self._state in ('WALK', 'STOPPING'):
            vx = self._step(dt)
        return self._pose(blend), (vx, vz)

    def _step(self, dt):
        """Advance the stride by dt; return the pelvis forward speed."""
        target = 0.0 if self._state == 'STOPPING' else 1.0
        rate = dt / RAMP_TIME
        self._amplitude += max(-rate, min(rate, target - self._amplitude))
        speed = self._amplitude * WALK_SPEED
        self._phase = (self._phase + dt / PERIOD) % 1.0
        stand_z = LEG_Z0 + CROUCH_DROP
        landing_x = ANKLE_X0 + self._amplitude * STEP_LENGTH / 2.0
        for side in ('l', 'r'):
            leg_phase = (self._phase + LEG_PHASE_OFFSET[side]) % 1.0
            if leg_phase < DUTY:
                # Planted: slides back relative to the hip exactly as fast
                # as the harness carries the hip forward.
                self._swinging[side] = False
                self._ankle_x[side] -= speed * dt
                self._ankle_z[side] = stand_z
            else:
                if not self._swinging[side]:
                    self._swinging[side] = True
                    self._lift_x[side] = self._ankle_x[side]
                s = (leg_phase - DUTY) / (1.0 - DUTY)
                # Leave and meet the floor at ground speed (-speed relative
                # to the hip), not at rest relative to the hip: easing to
                # zero hip-relative velocity made every touchdown skid
                # forward at walking speed (measured 6-10 cm per stance).
                ground = -speed * (1.0 - DUTY) * PERIOD
                self._ankle_x[side] = _hermite(
                    s, self._lift_x[side], ground, landing_x, ground)
                self._ankle_z[side] = stand_z + SWING_HEIGHT * math.sin(math.pi * s) ** 2
        if self._state == 'STOPPING' and self._amplitude == 0.0:
            # One more stride in place lands both feet back under the hips.
            self._settle += dt
            if self._settle >= PERIOD and not any(self._swinging.values()):
                self._state = 'STAND'
                self._settle = 0.0
                self.walking = False
                for side in ('l', 'r'):
                    self._ankle_x[side] = ANKLE_X0
        return speed

    def _pose(self, blend):
        """Build the 30 targets; `blend` < 1 only while crouching in."""
        pose = dict(self.neutral_pose)
        for side in ('l', 'r'):
            angles = leg_ik(self._ankle_x[side], self._ankle_z[side])
            for joint, angle in zip(('hpy', 'kny', 'aky'), angles):
                name = f'{side}_leg_{joint}'
                pose[name] += blend * (angle - pose[name])
        for name, delta in ARM_REST.items():
            pose[name] += blend * delta
        # Each arm forward with the opposite foot.
        half_step = STEP_LENGTH / 2.0
        pose['l_arm_shz'] -= ARM_SWING * (self._ankle_x['r'] - ANKLE_X0) / half_step
        pose['r_arm_shz'] += ARM_SWING * (self._ankle_x['l'] - ANKLE_X0) / half_step
        return [pose[name] for name in ATLAS_JOINT_NAMES]
