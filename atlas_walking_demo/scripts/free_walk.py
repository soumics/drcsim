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
Free-standing (no harness) walking for Atlas: ZMP preview control + stabilizer.

usage: ros2 run atlas_walking_demo free_walk.py --ros-args -p steps:=6
Takes over AtlasPlugin's PID setpoint (bumpless, see walk_keyboard.py),
crouches over CROUCH_TIME, settles, then walks the zmp_walk.ZmpWalker plan
at 200 Hz of simulation time:
- joint targets from the walker's CoM plan and leg IK;
- gravity feedforward in AtlasCommand.effort (planned load split);
- ankle stabilizer on the loaded legs from the pelvis IMU:
  aky += KP * pitch + KD * pitch_rate (roll likewise on akx and, scaled by
  load, on hpx) -- leaning back (pitch < 0) needs negative aky, measured;
- CoM feedback: the CoM measured from leg kinematics + IMU, relative to the
  loaded feet, against the plan; the pelvis target shifts to cancel the
  error. Laterally this is what makes it walk: without it a sideways sway
  grew step by step until Atlas fell after 2-4 steps.
Stops commanding (holding the last pose) if Atlas tilts past FALL_TILT.
"""

import math

from atlas_msgs.msg import AtlasCommand, AtlasState
import harness_gait as hg
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
import zmp_walk as zw

N = hg.ATLAS_JOINT_NAMES
CROUCH_TIME = 3.0
SETTLE_TIME = 2.0
FALL_TILT = math.radians(35)
GAIN_OVERRIDES = {'hpz': (1000.0, 10.0), 'hpx': (2500.0, 10.0), 'akx': (1000.0, 3.0)}


def _rpy(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))


class FreeWalkNode(Node):

    def __init__(self):
        super().__init__('atlas_free_walk',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.steps = self.declare_parameter('steps', 6).value
        self.step_length = self.declare_parameter('step_length', zw.STEP_LENGTH).value
        self.kp = self.declare_parameter('stabilizer_kp', 0.3).value
        self.kd = self.declare_parameter('stabilizer_kd', 0.02).value
        self.debug = self.declare_parameter('debug', False).value
        self.kp_roll = self.declare_parameter('stabilizer_kp_roll', 0.3).value
        self.kd_roll = self.declare_parameter('stabilizer_kd_roll', 0.02).value
        self.hip_roll_kp = self.declare_parameter('hip_roll_kp', 0.5).value
        self.hip_roll_kd = self.declare_parameter('hip_roll_kd', 0.03).value
        self.com_kp = self.declare_parameter('com_kp', 0.0).value
        self.com_kp_y = self.declare_parameter('com_kp_y', 1.0).value
        self.com_kd_y = self.declare_parameter('com_kd_y', 0.1).value
        self.com_kd = self.declare_parameter('com_kd', 0.05).value
        self.com_err = None
        self.com_rate = (0.0, 0.0)
        zw.SINGLE_SUPPORT = self.declare_parameter('single_support', zw.SINGLE_SUPPORT).value
        zw.DOUBLE_SUPPORT = self.declare_parameter('double_support', zw.DOUBLE_SUPPORT).value
        zw.ZMP_Y_INSET = self.declare_parameter('zmp_y_inset', zw.ZMP_Y_INSET).value
        self.state = None
        self.start_time = None
        self.setpoint = None
        self.fallen = False
        self.phase = 'takeover'
        self.done = False
        self.last_log = -1.0
        self.walker = zw.ZmpWalker(self.steps, self.step_length)
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 10)
        self.create_timer(zw.DT, self._tick)

    def _on_state(self, msg):
        self.state = msg
        if self.setpoint is None and len(msg.position) == len(N):
            self.setpoint = [p + e / k if k > 0 else p
                             for p, e, k in zip(msg.position, msg.effort, msg.kp_position)]
            self.gains_p = list(msg.kp_position)
            self.gains_d = list(msg.kd_position)
            for i, name in enumerate(N):
                joint = name.rsplit('_', 1)[1]
                if '_leg_' in name and joint in GAIN_OVERRIDES:
                    self.gains_p[i], self.gains_d[i] = GAIN_OVERRIDES[joint]
            self.get_logger().info(
                f'Took over; crouching, then {self.steps} steps of {self.step_length} m.')

    def _tick(self):
        if self.setpoint is None or self.fallen:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time
        roll, pitch = _rpy(self.state.orientation)
        if abs(roll) > FALL_TILT or abs(pitch) > FALL_TILT:
            self.fallen = True
            self.get_logger().error(
                f'Fell at t={t:.1f}s (pitch {math.degrees(pitch):+.0f}, '
                f'roll {math.degrees(roll):+.0f} deg, phase {self.phase}); stopped.')
            return
        walking = t >= CROUCH_TIME + SETTLE_TIME and not self.walker.done
        if walking and hasattr(self, 'share'):
            self._com_feedback(roll, pitch)
        if walking or not hasattr(self, 'angles'):
            self.angles, self.share, self.info = self.walker.sample() if walking else (
                self._stance())
        self.phase = self.info['phase'] if walking else (
            'crouch' if t < CROUCH_TIME else 'stand')
        blend = min(1.0, t / CROUCH_TIME)
        cmd = list(self.setpoint)
        for name, value in self.angles.items():
            i = N.index(name)
            cmd[i] += blend * (value - cmd[i])
        for name, delta in hg.ARM_REST.items():
            cmd[N.index(name)] += blend * delta
        effort = [0.0] * len(N)
        for name, torque in zw.gravity_feedforward(
                self.angles, self.walker.feet_in_pelvis, self.share,
                self.walker.cop_in_foot).items():
            effort[N.index(name)] = blend * torque
        rate_p, rate_r = self.state.angular_velocity.y, self.state.angular_velocity.x
        for side, frac in zip('lr', self.share):
            if frac > 0.2:
                cmd[N.index(f'{side}_leg_aky')] += self.kp * pitch + self.kd * rate_p
                cmd[N.index(f'{side}_leg_akx')] += self.kp_roll * roll + self.kd_roll * rate_r
                # Hip strategy for roll: the torso droops toward the swing
                # side through the stance hip, so correct it there too.
                cmd[N.index(f'{side}_leg_hpx')] += frac * (
                    self.hip_roll_kp * roll + self.hip_roll_kd * rate_r)
        msg = AtlasCommand()
        msg.position = cmd
        msg.effort = effort
        msg.k_effort = [255] * len(N)
        msg.kp_position, msg.kd_position = self.gains_p, self.gains_d
        self.pub.publish(msg)
        if self.debug and self.phase.startswith('swing') and t - self.last_log >= 0.1:
            stance = 'l' if self.phase == 'swing_r' else 'r'
            s = self.state
            foot = s.l_foot if stance == 'l' else s.r_foot
            load = max(1.0, -foot.force.z)
            idx = [N.index(f'{stance}_leg_{j}') for j in ('hpx', 'hpy', 'kny', 'aky', 'akx')]
            errs = ' '.join(f'{N[i][6:]}={math.degrees(cmd[i] - s.position[i]):+5.1f}'
                            for i in idx)
            self.get_logger().info(
                f'[dbg t={t:5.2f}] stance {stance}: cmd-meas deg {errs}  '
                f'CoP x={100 * foot.torque.y / load:+5.1f} '
                f'y={-100 * foot.torque.x / load:+5.1f} cm '
                f'load={load:5.0f} pitch={math.degrees(pitch):+5.1f}')
        if t - self.last_log >= 0.5:
            self.last_log = t
            s = self.state
            self.get_logger().info(
                f't={t:5.1f} {self.phase:8s} pitch={math.degrees(pitch):+5.1f} '
                f'roll={math.degrees(roll):+5.1f} Fz L/R={-s.l_foot.force.z:5.0f}/'
                f'{-s.r_foot.force.z:5.0f} com=({self.info["com"][0]:+.2f},'
                f'{self.info["com"][1]:+.2f})'
                + (f' comerr=({100 * self.com_err[0]:+.1f},{100 * self.com_err[1]:+.1f})cm '
                   f'shift=({100 * self.walker.pelvis_shift[0]:+.1f},'
                   f'{100 * self.walker.pelvis_shift[1]:+.1f})' if self.com_err else ''))
        if self.walker.done and not self.done:
            self.done = True
            self.get_logger().info(f'Walk finished at t={t:.1f}s; standing.')

    def _com_feedback(self, roll, pitch):
        """
        Shift the pelvis command to cancel the CoM's drift from the plan.

        Measured: the CoM relative to the stance ankle, from the measured leg
        angles and the IMU tilt. Planned: the walker's CoM relative to the
        same foot. The walker plans open-loop in position; without this,
        small errors accumulated step after step until Atlas fell (4 steps).
        """
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        err = [0.0, 0.0]
        # Blend both feet's estimates by their load share: switching the
        # reference foot outright turned each landing's placement error into
        # a step in the feedback.
        for side, frac in zip('lr', self.share):
            if frac <= 0.0:
                continue
            q = [self.state.position[N.index(f'{side}_leg_{j}')]
                 for j in ('hpz', 'hpx', 'hpy', 'kny')]
            ankle = hg.leg_fk_3d(side, *q)
            v = [zw.COM_IN_PELVIS[0] - ankle[0], zw.COM_IN_PELVIS[1] - ankle[1],
                 zw.COM_HEIGHT_IN_PELVIS - ankle[2]]
            v = [cp * v[0] + sp * sr * v[1] + sp * cr * v[2], cr * v[1] - sr * v[2]]
            foot = self.walker.feet[side]
            ref = [self.info['com'][0] - foot[0], self.info['com'][1] - foot[1]]
            err = [e + frac * (m - r) for e, m, r in zip(err, v, ref)]
        err = tuple(err)
        if self.com_err is not None:
            alpha = 0.1
            self.com_rate = tuple((1 - alpha) * r + alpha * (e - p) / zw.DT
                                  for r, e, p in zip(self.com_rate, err, self.com_err))
        self.com_err = err
        gains = ((self.com_kp, self.com_kd), (self.com_kp_y, self.com_kd_y))
        self.walker.pelvis_shift = tuple(
            max(-0.05, min(0.05, -(kp * e + kd * r)))
            for (kp, kd), e, r in zip(gains, err, self.com_rate))

    def _stance(self):
        """Return the initial stance: the walker's first sample, not advancing it."""
        saved = (self.walker.k, self.walker.state.copy(), self.walker.err_sum.copy())
        result = self.walker.sample()
        self.walker.k, self.walker.state, self.walker.err_sum = saved
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FreeWalkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
