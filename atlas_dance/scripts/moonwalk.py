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
A scripted moonwalk routine for Atlas under whole-body torque control.

usage: ros2 run atlas_dance moonwalk.py [--ros-args -p glide_start:=8.5 ...]
Not a copy of a dancer's pose: a choreography written for Atlas.
- groove: knees bounce on the beat, elbows pump, the torso twists;
- glide: long backward steps (qp_walk), each swinging foot sliding back on
  its toes -- heel up, toes skimming the floor -- then setting down flat,
  body leaning forward, arms loose: the moonwalk look;
- finale: the right hand comes up to the brim of an imaginary hat.
Times (s after 'dance start') are parameters, so the routine can be lined up
with a reference clip (drcsim_record_dance). Runs in lockstep: start the sim
with sync_max_per_window:=5.0 sync_max_per_step:=0.05.
"""

import math
import os
import sys

from ament_index_python.packages import get_package_prefix

sys.path.insert(0, os.path.join(get_package_prefix('atlas_walking_demo'), 'lib',
                                'atlas_walking_demo'))

from atlas_msgs.msg import AtlasCommand, AtlasState  # noqa: E402
import balance_controller as bc  # noqa: E402
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
import qp_walk  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
import torque_balance as tb  # noqa: E402

N = bc.N
BEAT = 1.2                 # s per beat (the step period while gliding)
# rad the gliding foot pitches toe-down (heel up): 0.15 danced the whole
# routine reliably; 0.2-0.25 sometimes, 0.35 rocked the torso until Atlas fell
# (measured).
TOE_DOWN = 0.15
TOE_CLEAR = 0.02           # m the toes skim above the floor
TOE_AHEAD = 0.13           # m from the sole centre to the toe (wbc.SOLE_FRONT)
# zmp_walk timing for the glide (set only while gliding: the crouch shares
# the constants): one step per beat, long backward steps.
GLIDE_TIMING = {'SINGLE_SUPPORT': 0.8, 'DOUBLE_SUPPORT': 0.4, 'MAX_STEP_BACK': 0.20,
                'ZMP_Y_INSET': 0.02}
# Hat tip, body frame (x forward, y left, z up from the ground between the soles).
# Reachable: up at the head (1.62 m) the arm ran into its limits and tipped
# Atlas over sideways (measured).
HAT = np.array([0.30, -0.15, 1.45])
R_HAT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # palm down


def smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def toe_slide(s):
    """
    Swing style of a moonwalk step: (sole height, pitch) at swing progress s.

    The heel comes up in the first 25% of the swing and is down again by 75%,
    the foot down by 85% (landing late or still rotating jolted Atlas,
    measured); meanwhile the toes stay TOE_CLEAR above the floor, so the foot seems
    to slide back on its toes. The clearance ramps in and out too: a swing
    that ended 3 cm up never landed, the QP counted the foot as planted
    anyway and Atlas fell (measured).
    """
    pitch = TOE_DOWN * min(smooth(s / 0.25), 1.0 - smooth((s - 0.5) / 0.25))
    clear = TOE_CLEAR * min(smooth(s / 0.1), 1.0 - smooth((s - 0.65) / 0.2))
    return clear + TOE_AHEAD * math.sin(pitch), pitch


class MoonwalkNode(Node):
    """Stand under torque control, then dance the routine."""

    def __init__(self):
        super().__init__('atlas_moonwalk',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        p = self.declare_parameter
        self.start_delay = float(p('start_delay', 2.0).value)
        self.glide_start = float(p('glide_start', 8.5).value)    # first glide step
        self.glide_steps = int(p('glide_steps', 5).value)
        self.glide_speed = float(p('glide_speed', 0.12).value)   # m/s backward
        self.finale = float(p('finale', 14.0).value)
        self.length = float(p('length', 19.5).value)
        self.bounce = float(p('bounce', 0.03).value)             # m knee bounce
        self.style = float(p('style', 1.0).value)                # groove size, 0..1+
        self.w_rot = float(p('w_rot', 10.0).value)  # 100 starved the swing-foot task
        global TOE_DOWN, TOE_CLEAR
        TOE_DOWN = float(p('toe_down', TOE_DOWN).value)
        TOE_CLEAR = float(p('toe_clear', TOE_CLEAR).value)
        self.urdf = None
        self.position = None
        self.torque = None
        self.t0 = None
        self.last_tick = -1.0
        self.done = False
        self.fallen = False
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.create_subscription(
            String, '/robot_description', lambda m: setattr(self, 'urdf', m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 1)
        self.get_logger().info(f'Moonwalk routine: {self.length:.1f} s.')
        self.last_log = -1.0

    # --- choreography ------------------------------------------------------

    def _joint(self, name, value):
        self.torque.posture[N.index(name)] = self.rest[N.index(name)] + value

    def _upper_body(self, t):
        """Set the posture targets of the back, neck and arms for dance time t."""
        beat = 2 * math.pi * t / BEAT
        groove = smooth(t / 1.0) * (1.0 - smooth((t - (self.glide_start - 1.0)) / 1.0))
        glide = smooth((t - (self.glide_start - 1.5)) / 1.0) * \
            (1.0 - smooth((t - self.finale) / 1.0))
        g = self.style
        # Torso: a twist on the beat while grooving; leaning forward to glide.
        self._joint('back_bkz', 0.3 * g * groove * math.sin(beat / 2))
        # (No forward lean while gliding: it pitched Atlas over the toes at the
        # end of a backward step, measured.)
        self._joint('back_bky', 0.08 * g * groove * max(0.0, math.sin(beat)))
        self._joint('neck_ry', 0.15 * g * groove * math.sin(beat))
        # Elbows: pumping in turn on the beat, then bent and loose to glide.
        pump = 0.9 * g * groove * (0.5 + 0.5 * math.sin(beat))
        pump_r = 0.9 * g * groove * (0.5 - 0.5 * math.sin(beat))
        self._joint('l_arm_elx', 0.4 * glide + pump)
        self._joint('r_arm_elx', -(0.4 * glide + pump_r))
        # Shoulders: swinging with the pumps, then a loose sway while gliding.
        swing = 0.25 * g * groove * math.sin(beat) + 0.12 * glide * math.sin(beat / 2)
        self._joint('l_arm_shz', swing)
        self._joint('r_arm_shz', swing)

    def _knees(self, t):
        """Bounce the CoM height on the beat while grooving."""
        groove = smooth(t / 1.0) * (1.0 - smooth((t - (self.glide_start - 2.0)) / 1.0))
        self.torque.com_target[2] = self.height - self.bounce * groove * \
            (0.5 - 0.5 * math.cos(2 * math.pi * t / BEAT))

    def _glide(self, s, t):
        tq = self.torque
        command_at = self.glide_start - qp_walk.zw.START_DELAY - qp_walk.zw.FIRST_SHIFT
        if tq.walk is None and t >= command_at and not getattr(self, 'glided', False):
            self.glided = True
            self.saved = {k: getattr(qp_walk.zw, k) for k in GLIDE_TIMING}
            for k, v in GLIDE_TIMING.items():
                setattr(qp_walk.zw, k, v)
            walk = tq.start_walk(s, self.now)
            walk.swing_style = toe_slide
            self.route = qp_walk.Route([(-self.glide_speed, 0.0, 0.0, self.glide_steps)])
        if tq.walk is not None and self.route.update(tq.walk):
            tq.end_walk()
            for k, v in self.saved.items():
                setattr(qp_walk.zw, k, v)
            self.get_logger().info(f'glide finished at {t:.2f} s')

    def _hat(self, t):
        tq = self.torque
        if t < self.finale or tq.walk is not None:
            tq.hand_targets = {}
            return
        # Ramp from when the hand actually starts: the glide's closing step can
        # run past `finale`, and a hand that jumped straight to the hat threw
        # Atlas over sideways (measured).
        if not hasattr(self, 'hat_t0'):
            self.hat_t0 = t
        a = smooth((t - self.hat_t0) / 2.0) * (1.0 - smooth((t - (self.length - 2.5)) / 2.0))
        if not hasattr(self, 'hat_from'):
            w = tq.wbc
            pin.framesForwardKinematics(w.model, w.data, tq.q)
            f = w.data.oMf[w.palms['r']]
            self.hat_from = (f.translation.copy(), f.rotation.copy())
            soles = tq._soles(tq.q)
            mid = (soles['l'][0] + soles['r'][0]) / 2
            mid[2] = min(soles['l'][0][2], soles['r'][0][2])
            yaw = soles['r'][1]
            rz = pin.rpy.rpyToMatrix(0.0, 0.0, yaw)
            self.hat_to = (mid + rz @ HAT, rz @ R_HAT)
        (p0, r0), (p1, r1) = self.hat_from, self.hat_to
        tq.hand_targets = {'r': (p0 + a * (p1 - p0), r0 @ pin.exp3(a * pin.log3(r0.T @ r1)))}

    def _swing_info(self):
        tq = self.torque
        if tq.walk is None or not tq.walk.phase.startswith('swing'):
            return '-'
        side = tq.walk.phase[-1]
        target = tq.walk.foot_targets(''.join(c for c in 'lr' if c != side))[side]
        pin.framesForwardKinematics(tq.wbc.model, tq.wbc.data, tq.q)
        f = tq.wbc.data.oMf[tq.wbc.feet[side]]
        pitch = math.atan2(-f.rotation[2, 0], math.hypot(f.rotation[0, 0], f.rotation[1, 0]))
        ground = tq.walk.ground
        return (f'{side} z={f.translation[2] - ground:.3f}/{target[0][2] - ground:.3f}'
                f' pitch={pitch:+.2f}/{target[2]:+.2f}')

    def _at_limits(self, s):
        m, w = self.torque.wbc.model, self.torque.wbc
        out = []
        for j, x in enumerate(s.position):
            i = w.iq[j]
            lo, hi = m.lowerPositionLimit[i], m.upperPositionLimit[i]
            if x < lo + 0.05 or x > hi - 0.05:
                out.append(f'{N[j]}={x:+.2f}[{lo:+.2f},{hi:+.2f}]')
        return ' '.join(out)

    # --- control loop ------------------------------------------------------

    def _on_state(self, s):
        if self.urdf is None or len(s.position) != len(N) or self.fallen:
            return
        t = self.now = self.get_clock().now().nanoseconds / 1e9
        if self.position is None:
            self.position = bc.FreeWalkController(s)
            self.get_logger().info('Took over; crouching.')
        roll, pitch = bc.rpy(s.orientation)
        if max(abs(roll), abs(pitch)) > bc.FALL_TILT:
            self.fallen = True
            self.get_logger().error(f'Fell (roll {math.degrees(roll):+.0f}, '
                                    f'pitch {math.degrees(pitch):+.0f}); stopped.')
            return
        cmd = AtlasCommand()
        cmd.k_effort = [255] * len(N)
        cmd.header.stamp = s.header.stamp
        if self.torque is None:
            if t - self.last_tick < qp_walk.zw.DT - 1e-4:
                return
            self.last_tick = t
            out = self.position.update(s, t)
            cmd.position, cmd.effort = out.position, out.effort
            cmd.kp_position, cmd.kd_position = out.kp, out.kd
            if self.position.status == 'ready':
                self.torque = tb.TorqueBalance(self.urdf, N, s, {
                    'stepping': False, 'w_rot': self.w_rot})
                self.rest = self.torque.posture.copy()
                self.height = self.torque.com_target[2]
                self.t0 = t + self.start_delay
                self.get_logger().info('Standing; torque control.')
        else:
            dt = t - self.t0
            if dt >= 0.0 and not self.done:
                if not getattr(self, 'announced', False):
                    self.announced = True
                    self.get_logger().info(f'dance start at sim time {t:.3f}')
                self._upper_body(dt)
                self._knees(dt)
                self._glide(s, dt)
                self._hat(dt)
                if dt >= self.length:
                    self.done = True
                    self.torque.hand_targets = {}
                    self.get_logger().info('dance finished')
            tau = self.torque.update(s, t)
            if t - self.last_log >= (0.1 if self.torque.walk else 1.0):
                self.last_log = t
                walk = f' {self.torque.walk.phase}' if self.torque.walk else ''
                self.get_logger().info(
                    f'[{dt:5.1f}] roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f}'
                    f'{walk} qp_failures={self.torque.wbc.last.get("failures", 0)} '
                    f'load=({-s.l_foot.force.z:.0f},{-s.r_foot.force.z:.0f}) '
                    f'swing={self._swing_info()} '
                    f'at_limit={self._at_limits(s)}')
            cmd.position = [0.0] * len(N)
            cmd.effort = [float(x) for x in tau]
            cmd.kp_position = [0.0] * len(N)
            cmd.kd_position = [1.0] * len(N)
            cmd.desired_controller_period_ms = 2
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoonwalkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
