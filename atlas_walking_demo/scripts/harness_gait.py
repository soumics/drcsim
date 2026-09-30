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
Omnidirectional foot-trajectory walking gait for Atlas in VRCPlugin's harness.

Pure Python (no rclpy). The pelvis is held by the harness
("pinned_with_gravity") and moved through atlas/cmd_vel, so this gait never
has to balance. It is driven by a body velocity command (vx forward, vy
left, wz turning) and plans where each *ankle* goes relative to the pelvis
every tick, then solves the whole leg's inverse kinematics (hip yaw, hip
roll, hip pitch, knee, ankle pitch and roll; soles kept flat):

* stance: the ankle moves exactly opposite to the pelvis motion (including
  rotation), so the planted foot stays still on the floor;
* swing: the foot lifts off at ground speed, arcs 8 cm high and lands half
  a step ahead in the direction of travel, again at ground speed;
* the harness is lowered CROUCH_DROP first, so the knees stay bent ~30 deg,
  as in human walking; each arm swings with the opposite leg.

The commanded velocity is rate-limited, so starting, stopping and changing
direction never jump. Stopping ends with one stride in place that brings
both feet back under the hips.
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

# Hip geometry from atlas_v5_raw.urdf (left leg; y mirrored on the right):
# l_leg_hpz/hpx at (0, 0.089, 0) in the pelvis frame, l_leg_hpy at
# (0.05, 0.0225, -0.066) from there. Joint order down the leg: hpz (yaw),
# hpx (roll), hpy/kny/aky (pitch), akx (roll).
HIP_Y = 0.089
HPY_ORIGIN = (0.05, 0.0225, -0.066)
# Ankle position in the pelvis frame at zero joint angles.
FOOT_X0 = HPY_ORIGIN[0] + THIGH[0] + SHIN[0]
FOOT_Y0 = HIP_Y + HPY_ORIGIN[1]
FOOT_Z0 = HPY_ORIGIN[2] + THIGH[1] + SHIN[1]

# Gait parameters (m, s, rad).
CROUCH_DROP = 0.025      # harness lowered this much: ~30 deg knee bend mid-stance,
#                          nearly straight at push-off (a 5 cm drop gave 49 deg)
CROUCH_DURATION = 3.0    # lowering the harness and the arms on takeover
PERIOD = 1.2             # one full stride (left + right step)
DUTY = 0.6               # fraction of a stride each foot is on the floor
SWING_HEIGHT = 0.08      # ankle lift at mid-swing
STANCE_TIME = DUTY * PERIOD
SWING_TIME = PERIOD - STANCE_TIME

# Velocity limits and rate limits. Hip-yaw joint limits (-0.17 rad inward)
# bound turning: a stance foot rotates wz * STANCE_TIME / 2 each way.
MAX_VX = 0.42            # forward; about a 0.30 m ankle stroke per stance
MAX_VX_BACK = 0.30
MAX_VY = 0.20
MAX_WZ = 0.40
LIN_ACCEL = 0.35         # m/s^2
ANG_ACCEL = 0.6          # rad/s^2
WALK_SPEED = MAX_VX

# Arms: lowered from Atlas's zero T-pose, elbows slightly bent; shz swings
# them. Signs measured live (TF pelvis->hand): l_arm_shx -1.2 / r_arm_shx
# +1.2 lower the arms; negative l_arm_shz / positive r_arm_shz move that
# hand forward; ely 1.57 turns elx into a forward elbow bend.
ARM_REST = {
    'l_arm_shx': -1.3, 'r_arm_shx': 1.3,
    'l_arm_ely': 1.57, 'r_arm_ely': 1.57,
    'l_arm_elx': 0.5, 'r_arm_elx': -0.5,
}
ARM_SWING = 0.35         # shz amplitude for a 0.15 m foot excursion
ARM_SWING_REF = 0.15

# Stance/swing phase offset per leg: the right leg is half a stride behind.
LEG_PHASE_OFFSET = {'l': 0.0, 'r': 0.5}
SIDE_SIGN = {'l': 1.0, 'r': -1.0}


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


def leg_ik_3d(side, x, y, z, yaw=0.0):
    """
    Return (hpz, hpx, hpy, kny, aky, akx) for an ankle at (x, y, z), foot yaw.

    (x, y, z) is the ankle in the pelvis frame and `yaw` the foot's heading
    relative to the pelvis; the sole is kept level (akx cancels the hip
    roll, aky the hip and knee pitch).
    """
    sign = SIDE_SIGN[side]
    dx, dy = x, y - sign * HIP_Y
    c, s = math.cos(yaw), math.sin(yaw)
    # Into the hip-yaw frame.
    x1, y1, z1 = c * dx + s * dy, -s * dx + c * dy, z
    # Hip roll: in the frontal plane the ankle sits at (hpy lateral offset,
    # z_leg) rotated by hpx; an angle measured as atan2(y, -z) adds up.
    offset_y = sign * HPY_ORIGIN[1]
    z_leg = -math.sqrt(max(y1 * y1 + z1 * z1 - offset_y * offset_y, 1e-9))
    hpx = math.atan2(y1, -z1) - math.atan2(offset_y, -z_leg)
    hpy, kny, aky = leg_ik(x1 - HPY_ORIGIN[0], z_leg - HPY_ORIGIN[2])
    return yaw, hpx, hpy, kny, aky, -hpx


def leg_fk_3d(side, hpz, hpx, hpy, kny):
    """Return the ankle (x, y, z) in the pelvis frame: the inverse of leg_ik_3d."""
    sign = SIDE_SIGN[side]
    sx, sz = leg_fk(hpy, kny)
    x, y, z = HPY_ORIGIN[0] + sx, sign * HPY_ORIGIN[1], HPY_ORIGIN[2] + sz
    c, s = math.cos(hpx), math.sin(hpx)
    y, z = y * c - z * s, y * s + z * c
    c, s = math.cos(hpz), math.sin(hpz)
    return x * c - y * s, sign * HIP_Y + x * s + y * c, z


class HarnessGait:
    """
    Tick-driven omnidirectional gait; call sample(dt) at a steady rate.

    set_velocity(vx, vy, wz) sets the target body velocity (clamped and
    rate-limited); stop() brings Atlas to a stand. sample() returns
    (positions, (vx, vy, vz, wz)): the 30 joint targets in
    ATLAS_JOINT_NAMES order, and the pelvis velocity to send on
    atlas/cmd_vel (pelvis frame) so the harness moves in step with the feet.
    """

    def __init__(self, neutral_pose):
        self.neutral_pose = dict(neutral_pose)
        self._time = 0.0
        self._state = 'CROUCH'
        self._phase = 0.0                 # stride phase, 0..1
        self._target = (0.0, 0.0, 0.0)    # commanded (vx, vy, wz)
        self._velocity = [0.0, 0.0, 0.0]  # rate-limited (vx, vy, wz)
        self._settle = 0.0                # time stepping in place to stop
        stand_z = FOOT_Z0 + CROUCH_DROP
        self._foot = {side: [FOOT_X0, SIDE_SIGN[side] * FOOT_Y0, stand_z, 0.0]
                      for side in ('l', 'r')}  # x, y, z, yaw (pelvis frame)
        self._lift = {side: list(self._foot[side]) for side in ('l', 'r')}
        self._swinging = {'l': False, 'r': False}

    # -- commands -----------------------------------------------------------

    @property
    def phase_name(self):
        """Return 'CROUCH', 'STAND', 'WALK' or 'STOPPING'."""
        return self._state

    @property
    def walking(self):
        return self._state in ('WALK', 'STOPPING') or (
            self._state == 'CROUCH' and any(self._target))

    @property
    def velocity(self):
        """Return the current (rate-limited) (vx, vy, wz)."""
        return tuple(self._velocity)

    def set_velocity(self, vx, vy, wz):
        """Set the target body velocity; all zeros is the same as stop()."""
        vx = max(-MAX_VX_BACK, min(MAX_VX, vx))
        vy = max(-MAX_VY, min(MAX_VY, vy))
        wz = max(-MAX_WZ, min(MAX_WZ, wz))
        # Turning adds wz * FOOT_Y0 to the outer foot's fore/aft speed;
        # scale the command so no stance stroke outreaches straight-ahead
        # full speed (else the leg hits full extension, then the knee
        # snaps ~11 rad/s at lift-off).
        fore_aft = abs(vx) + abs(wz) * FOOT_Y0
        limit = MAX_VX if vx >= 0.0 else MAX_VX_BACK
        scale = min(1.0, limit / fore_aft) if fore_aft > 0.0 else 1.0
        target = (vx * scale, vy, wz * scale)
        self._target = target
        if not any(target):
            self.stop()
            return
        if self._state in ('STAND', 'STOPPING'):
            self._state = 'WALK'
            self._settle = 0.0

    def stop(self):
        self._target = (0.0, 0.0, 0.0)
        if self._state == 'WALK':
            self._state = 'STOPPING'

    def start_walking(self):
        """Walk straight ahead at full speed (the original 'w' behaviour)."""
        self.set_velocity(WALK_SPEED, 0.0, 0.0)

    def stop_walking(self):
        self.stop()

    # -- stepping -----------------------------------------------------------

    def sample(self, dt):
        self._time += dt
        blend, vz = 1.0, 0.0
        if self._state == 'CROUCH':
            # Joint-space blend from the held setpoint to the crouched IK
            # stance: joint zero (kny = 0) lies on the other IK branch than
            # a forward-bent knee (the thigh's 5 cm offset makes kny = 0 and
            # kny ~= 0.27 the same leg length), so IK alone would snap.
            blend = min(1.0, self._time / CROUCH_DURATION)
            if blend < 1.0:
                vz = -CROUCH_DROP / CROUCH_DURATION
            else:
                self._state = 'WALK' if any(self._target) else 'STAND'
        elif self._state in ('WALK', 'STOPPING'):
            self._step(dt)
        vx, vy, wz = self._velocity
        return self._pose(blend), (vx, vy, vz, wz)

    def _ramp_velocity(self, dt):
        limits = (LIN_ACCEL, LIN_ACCEL, ANG_ACCEL)
        for i, (target, accel) in enumerate(zip(self._target, limits)):
            delta = target - self._velocity[i]
            self._velocity[i] += max(-accel * dt, min(accel * dt, delta))

    def _ground_velocity(self, x, y):
        """Pelvis-frame velocity of a point fixed on the floor at (x, y)."""
        vx, vy, wz = self._velocity
        return (-vx + wz * y, -vy - wz * x, -wz)

    def _landing(self, side):
        """Where the swing foot lands: half a stance stroke ahead of home."""
        home_x, home_y = FOOT_X0, SIDE_SIGN[side] * FOOT_Y0
        gx, gy, gyaw = self._ground_velocity(home_x, home_y)
        half = STANCE_TIME / 2.0
        return (home_x - gx * half, home_y - gy * half, -gyaw * half)

    def _step(self, dt):
        self._ramp_velocity(dt)
        self._phase = (self._phase + dt / PERIOD) % 1.0
        stand_z = FOOT_Z0 + CROUCH_DROP
        for side in ('l', 'r'):
            foot = self._foot[side]
            leg_phase = (self._phase + LEG_PHASE_OFFSET[side]) % 1.0
            if leg_phase < DUTY:
                # Planted: moves exactly opposite to the pelvis.
                self._swinging[side] = False
                gx, gy, gyaw = self._ground_velocity(foot[0], foot[1])
                foot[0] += gx * dt
                foot[1] += gy * dt
                foot[3] += gyaw * dt
                foot[2] = stand_z
                continue
            if not self._swinging[side]:
                self._swinging[side] = True
                self._lift[side] = list(foot)
            s = (leg_phase - DUTY) / (1.0 - DUTY)
            lift = self._lift[side]
            land = self._landing(side)
            # Leave and meet the floor at ground speed, not at rest relative
            # to the hip (that made every touchdown skid).
            g_lift = self._ground_velocity(lift[0], lift[1])
            g_land = self._ground_velocity(land[0], land[1])
            for i, axis in enumerate((0, 1, 3)):
                foot[axis] = _hermite(
                    s, lift[axis], g_lift[i] * SWING_TIME,
                    land[i], g_land[i] * SWING_TIME)
            foot[2] = stand_z + SWING_HEIGHT * math.sin(math.pi * s) ** 2
        if self._state == 'STOPPING' and not any(self._velocity):
            # One more stride in place lands both feet back home.
            self._settle += dt
            if self._settle >= PERIOD and not any(self._swinging.values()):
                self._state = 'STAND'
                self._settle = 0.0
                for side in ('l', 'r'):
                    self._foot[side] = [FOOT_X0, SIDE_SIGN[side] * FOOT_Y0, stand_z, 0.0]

    def _pose(self, blend):
        """Build the 30 targets; `blend` < 1 only while crouching in."""
        pose = dict(self.neutral_pose)
        for side in ('l', 'r'):
            x, y, z, yaw = self._foot[side]
            angles = leg_ik_3d(side, x, y, z, yaw)
            for joint, angle in zip(('hpz', 'hpx', 'hpy', 'kny', 'aky', 'akx'), angles):
                name = f'{side}_leg_{joint}'
                pose[name] += blend * (angle - pose[name])
        for name, delta in ARM_REST.items():
            pose[name] += blend * delta
        # Each arm forward with the opposite foot.
        pose['l_arm_shz'] -= ARM_SWING * (self._foot['r'][0] - FOOT_X0) / ARM_SWING_REF
        pose['r_arm_shz'] += ARM_SWING * (self._foot['l'][0] - FOOT_X0) / ARM_SWING_REF
        return [pose[name] for name in ATLAS_JOINT_NAMES]
