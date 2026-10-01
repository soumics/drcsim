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
Play a retargeted dance on Atlas in Gazebo, balancing free-standing.

usage: ros2 run atlas_dance dance_player.py --ros-args -p moves:=moonwalk.npz
       [-p legs:=torque_moonwalk|moonwalk|stand|torque] [-p glide_speed:=0.06]
       [-p skim_height:=0.05] [-p start_delay:=2.0]
'torque': the whole-body torque QP balances (torque_balance.py) and the dance
sets its posture targets -- survives full-amplitude moves. 'torque_moonwalk':
the same, walking backward through the dance (qp_walk), the swing foot
skimming the floor; start the sim with sync_max_per_window:=5.0. Otherwise the legs
are atlas_walking_demo's free-standing controller (ZMP walking, balance
feedback): 'stand' keeps the stance, 'moonwalk' walks backward with
the swing foot skimming the floor, so Atlas glides back while its legs look
like they walk. The back, arms and neck follow the dance (retarget.py),
blended in over BLEND s; the controller's CoM feedback absorbs the swinging
arms. Logs 'dance start' (sim time) so a recording can be synced to the clip.
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
import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
import zmp_walk as zw  # noqa: E402

BLEND = 2.0  # s to blend the dance in (and out)
UPPER = ['back_bkz', 'back_bky', 'back_bkx', 'neck_ry'] + [
    f'{s}_arm_{j}' for s in 'lr' for j in ('shz', 'shx', 'ely', 'elx', 'wry', 'wrx', 'wry2')]


class DancePlayerNode(Node):
    """Stand, then play the dance (upper body) with the chosen leg mode."""

    def __init__(self):
        super().__init__('atlas_dance_player',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        path = self.declare_parameter('moves', '').value
        self.legs = self.declare_parameter('legs', 'moonwalk').value
        self.glide = self.declare_parameter('glide_speed', 0.06).value
        self.skim = self.declare_parameter('skim_height', 0.05).value
        # Off by default: pointing the swinging toe (heel up) tipped the
        # already marginal backward glide over sideways (measured, 0.2-0.4 rad).
        self.toe = self.declare_parameter('toe_point', 0.0).value
        self.amplitude = self.declare_parameter('amplitude', 0.6).value
        self.max_rate = self.declare_parameter('max_joint_speed', 2.0).value
        self.start_delay = self.declare_parameter('start_delay', 2.0).value
        moves = np.load(path)
        self.t_moves = moves['t']
        names = list(moves['names'])
        self.moves = {n: moves['joints'][:, names.index(n)] for n in UPPER}
        self.duration = float(self.t_moves[-1])
        self.controller = None
        self.dance_t0 = None
        self.done = False
        self.last_tick = -1.0
        self.torque = None   # legs:=torque: torque_balance.TorqueBalance once standing
        self.urdf = None
        if self.legs in ('torque', 'torque_moonwalk'):
            self.create_subscription(
                String, '/robot_description', lambda m: setattr(self, 'urdf', m.data),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 1)
        self.get_logger().info(f'Loaded {path}: {self.duration:.1f} s of moves, legs: '
                               f'{self.legs}.')

    def _pose_at(self, t):
        """Return {joint: angle} of the dance at dance time t (s), interpolated."""
        return {n: float(np.interp(t, self.t_moves, v)) for n, v in self.moves.items()}

    def _upper_targets(self, t, current):
        """Advance the dance: blended, scaled, rate-limited upper-body targets."""
        dt = t - self.dance_t0
        if dt < 0.0 or self.done:
            return current
        if not getattr(self, 'announced', False):
            self.announced = True
            self.get_logger().info(f'dance start at sim time {t:.3f}')
        w = self.amplitude * min(1.0, dt / BLEND, max(0.0, (self.duration - dt) / BLEND))
        pose = self._pose_at(min(dt, self.duration))
        out = dict(current)
        for n in UPPER:
            if math.isnan(pose[n]):
                continue
            want = (1 - w) * self.stance_upper[n] + w * pose[n]
            now = current.get(n, self.stance_upper[n])
            step = self.max_rate * max(0.0, min(0.01, t - getattr(self, '_t_up', t)))
            out[n] = now + max(-step, min(step, want - now))
        self._t_up = t
        if dt >= self.duration and not self.done:
            self.done = True
            self.get_logger().info('dance finished')
        return out

    def _torque_tick(self, s, t):
        """
        legs:=torque(_moonwalk) -- whole-body QP; the dance sets the posture targets.

        torque_moonwalk also walks backward (qp_walk) through the dance, the
        swing foot skimming the floor: the QP balances the swinging arms and
        the steps together. Position-mode moonwalking with the arms dancing
        tipped forward within 4 s (measured).
        """
        tq = self.torque
        current = {n: tq.posture[bc.N.index(n)] for n in UPPER}
        for n, v in self._upper_targets(t, current).items():
            tq.posture[bc.N.index(n)] = v
        if self.legs == 'torque_moonwalk' and self.dance_t0 is not None:
            if tq.walk is None and t >= self.dance_t0 and not self.done and \
                    not getattr(self, 'walked', False):
                self.walked = True
                zw.SWING_HEIGHT = self.skim
                tq.start_walk(s, t).set_velocity(-self.glide, 0.0, 0.0)
            elif tq.walk is not None and self.done:
                if tq.walk.walker.walking:
                    tq.walk.stop()
                elif tq.walk.idle:
                    tq.end_walk()
                    self.get_logger().info('glide finished; standing')
        roll, pitch = bc.rpy(s.orientation)
        if max(abs(roll), abs(pitch)) > bc.FALL_TILT:
            if not getattr(self, 'fell_logged', False):
                self.fell_logged = True
                self.get_logger().error(f'Fell (roll {math.degrees(roll):+.0f}, '
                                        f'pitch {math.degrees(pitch):+.0f}); stopped.')
            return
        tau = tq.update(s, t)
        cmd = AtlasCommand()
        cmd.header.stamp = s.header.stamp
        cmd.position = [0.0] * len(bc.N)
        cmd.effort = [float(x) for x in tau]
        cmd.k_effort = [255] * len(bc.N)
        cmd.kp_position = [0.0] * len(bc.N)
        cmd.kd_position = [1.0] * len(bc.N)
        cmd.desired_controller_period_ms = 2
        self.pub.publish(cmd)

    def _on_state(self, s):
        if len(s.position) != len(bc.N):
            return
        t = self.get_clock().now().nanoseconds / 1e9
        if self.torque is not None:
            self._torque_tick(s, t)
            return
        if t - self.last_tick < zw.DT - 1e-4:  # the controller runs at 200 Hz
            return
        self.last_tick = t
        c = self.controller
        if c is None:
            c = self.controller = bc.FreeWalkController(s)
            self.get_logger().info('Took over; crouching.')
        if c.status == 'ready' and self.legs.startswith('torque') and self.urdf is not None:
            import torque_balance as tb
            self.torque = tb.TorqueBalance(self.urdf, bc.N, s, {'stepping': False})
            self.stance_upper = {n: self.torque.posture[bc.N.index(n)] for n in UPPER}
            self.dance_t0 = t + self.start_delay
            self.get_logger().info('Standing; torque control holds the balance.')
            return
        if c.status == 'ready' and self.dance_t0 is None and not self.legs.startswith('torque'):
            self.dance_t0 = t + self.start_delay
            self.stance_upper = {n: c._target({})[bc.N.index(n)] for n in UPPER}
        if self.dance_t0 is not None and not self.done:
            dt = t - self.dance_t0
            if 0.0 <= dt and not getattr(self, 'announced', False):
                self.announced = True
                self.get_logger().info(f'dance start at sim time {t:.3f}')
                if self.legs == 'moonwalk':
                    zw.SWING_HEIGHT = self.skim  # the swing foot skims the floor
                    c.set_velocity(-self.glide, 0.0, 0.0)
            if dt >= 0.0:
                # Blend in/out, scale the moves around the stance (amplitude),
                # and rate-limit each joint: big fast arm and torso swings
                # throw the CoM around faster than the legs can follow.
                w = self.amplitude * min(1.0, dt / BLEND,
                                         max(0.0, (self.duration - dt) / BLEND))
                pose = self._pose_at(min(dt, self.duration))
                step = self.max_rate * zw.DT
                for n in UPPER:
                    if math.isnan(pose[n]):
                        continue
                    want = (1 - w) * self.stance_upper[n] + w * pose[n]
                    now = c.upper_body.get(n, self.stance_upper[n])
                    c.upper_body[n] = now + max(-step, min(step, want - now))
            if dt >= self.duration:
                self.done = True
                c.upper_body = {}
                c.stop()
                self.get_logger().info('dance finished')
        out = c.update(s, t)
        if out is None:
            if c.status == 'fallen' and not getattr(self, 'fell_logged', False):
                self.fell_logged = True
                self.get_logger().error(f'Fell ({c.fall_info}); stopped commanding.')
            return
        if self.dance_t0 is not None and t - getattr(self, 'last_log', -1.0) >= 0.25:
            self.last_log = t
            roll, pitch = bc.rpy(s.orientation)
            self.get_logger().info(
                f'[{t - self.dance_t0:5.2f}] {c.walker.phase:8s} walking={c.walking} '
                f'roll={math.degrees(roll):+5.1f} pitch={math.degrees(pitch):+5.1f} '
                f'cmd_v={c.walker.command[0]:+.2f}')
        position = list(out.position)
        if self.legs == 'moonwalk' and self.dance_t0 is not None and not self.done:
            # Moonwalk look: the swinging foot points its toe down (heel up)
            # as it lifts, so it seems to slide back on its toes.
            for side in 'lr':
                lift = max(0.0, c.walker.feet[side][2]) / max(1e-3, zw.SWING_HEIGHT)
                position[bc.N.index(f'{side}_leg_aky')] += self.toe * lift
        cmd = AtlasCommand()
        cmd.position, cmd.effort = position, out.effort
        cmd.k_effort = [255] * len(bc.N)
        cmd.kp_position, cmd.kd_position = out.kp, out.kd
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DancePlayerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
