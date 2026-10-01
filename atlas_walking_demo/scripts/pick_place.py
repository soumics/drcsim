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
Pick up a heavy box with both hands, carry it sideways, set it down.

usage: ros2 run atlas_walking_demo pick_place.py [--ros-args -p box_mass:=10.0]
Spawns a table and a box in front of Atlas (ros_gz_sim create), stands up
like torque_stand.py (position-mode crouch, then the whole-body torque QP),
then moves both palms through WAYPOINTS with smoothly interpolated 6D hand
tasks while the QP keeps Atlas balanced: the box's weight shifts the
centre of mass forward and the CoM task leans the body back to carry it.
The grip is a two-handed squeeze (SVH palms 2 cm inside the box's sides):
finger grasps alone don't hold this much weight. While the box is held the
QP's model carries its mass at the palms (wbc.set_payload). Needs SVH hands
(atlas.launch.py's default) and Pinocchio/ProxQP; runs in lockstep, so
start the sim with sync_max_per_window:=5.0.
"""

import math
import subprocess

from atlas_msgs.msg import AtlasCommand, AtlasState
import balance_controller as bc
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import torque_balance as tb

N = bc.N
BOX_SIZE = 0.30                       # m, cube
# Task geometry is relative to the midpoint between the soles, on the ground:
# the QP's world frame has its origin at the starting pelvis, Gazebo's at
# the spawn point (soles 3.2 cm ahead of it), so neither frame can be used
# directly -- sending the hands to Gazebo coordinates in the QP's frame put
# them a metre too high and toppled Atlas (measured).
BOX_CENTER = np.array([0.50, 0.0, 1.0])
SOLES_IN_GAZEBO = np.array([0.032, 0.0, 0.0])  # Atlas spawned at the origin, facing +x
TABLE_TOP = BOX_CENTER[2] - BOX_SIZE / 2
CARRY_Y = -0.25                       # m: where the box goes, sideways
TABLE_W = BOX_SIZE - CARRY_Y + 0.05   # m: just wide enough for both box spots
GRIP = 0.02                           # m the palms press in past the box's sides
# Palm orientations (world): fingers forward (+x), palms facing the box.
R_LEFT = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
R_RIGHT = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])


def palms_at(center, gap):
    """Return {side: (palm position, rotation)} with the palms gap m from the box's sides."""
    half = BOX_SIZE / 2 + gap
    return {'l': (center + [0.0, half, 0.0], R_LEFT),
            'r': (center + [0.0, -half, 0.0], R_RIGHT)}


LIFT = np.array([0.0, 0.0, 0.15])
CARRIED = BOX_CENTER + [0.0, CARRY_Y, 0.0]
# (name, seconds to get there, palm targets)
# The hands rest low at the sides: going straight for the box ran them into
# the side of the table (measured), so they come up over the top first.
WAYPOINTS = [
    ('raise', 2.5, palms_at(np.array([0.25, 0.0, BOX_CENTER[2] + 0.15]), 0.25)),
    ('reach', 2.5, palms_at(BOX_CENTER + [0.0, 0.0, 0.08], 0.15)),
    ('pre-grasp', 2.0, palms_at(BOX_CENTER, 0.15)),
    ('approach', 2.0, palms_at(BOX_CENTER, 0.02)),
    ('squeeze', 1.5, palms_at(BOX_CENTER, -GRIP)),
    ('lift', 2.5, palms_at(BOX_CENTER + LIFT, -GRIP)),
    ('carry', 3.0, palms_at(CARRIED + LIFT, -GRIP)),
    ('lower', 2.5, palms_at(CARRIED + [0.0, 0.0, 0.005], -GRIP)),
    ('release', 1.5, palms_at(CARRIED, 0.12)),
    ('retreat', 2.5, palms_at(CARRIED + [0.0, 0.0, 0.15], 0.15)),
    ('clear', 2.0, palms_at(np.array([0.25, 0.0, BOX_CENTER[2] + 0.15]), 0.25)),
    # None: back to where the palms started. Dropping the hand task with the
    # arms up let the posture task swing them down at once (fell, measured).
    ('home', 2.5, None),
]


def scene_sdf(box_mass):
    """
    Return [(name, SDF, (x, y, z)), ...] for the table and the box (Gazebo frame).

    ros_gz_sim create ignores the SDF's own <pose> (it uses its -x/-y/-z,
    default 0): both models landed on Atlas until the pose was passed there.
    """
    tx, ty = SOLES_IN_GAZEBO[0] + BOX_CENTER[0], CARRY_Y / 2
    table = f"""<sdf version="1.9"><model name="table"><static>true</static>
      <link name="link">
      <collision name="c"><geometry><box><size>0.4 {TABLE_W} {TABLE_TOP}</size></box></geometry>
      </collision><visual name="v"><geometry><box><size>0.4 {TABLE_W} {TABLE_TOP}</size></box>
      </geometry><material><diffuse>0.45 0.32 0.22 1</diffuse></material></visual>
      </link></model></sdf>"""
    inertia = box_mass * BOX_SIZE ** 2 / 6
    box = f"""<sdf version="1.9"><model name="box">
      <link name="link">
      <inertial><mass>{box_mass}</mass><inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy>
      <izz>{inertia}</izz></inertia></inertial>
      <collision name="c"><geometry><box><size>{BOX_SIZE} {BOX_SIZE} {BOX_SIZE}</size></box>
      </geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
      </surface></collision>
      <visual name="v"><geometry><box><size>{BOX_SIZE} {BOX_SIZE} {BOX_SIZE}</size></box>
      </geometry><material><diffuse>0.9 0.55 0.1 1</diffuse></material></visual>
      </link></model></sdf>"""
    return [('table', table, (tx, ty, TABLE_TOP / 2)),
            ('box', box, tuple(SOLES_IN_GAZEBO + BOX_CENTER))]


def smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


class PickPlaceNode(Node):
    """Stand under torque control, then run the two-handed pick-and-place."""

    def __init__(self):
        super().__init__('atlas_pick_place',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.box_mass = self.declare_parameter('box_mass', 10.0).value
        self.sync_ms = self.declare_parameter('sync_period_ms', 2).value
        self.debug = self.declare_parameter('debug', False).value
        self.last_debug = -1.0
        for name, sdf, (x, y, z) in scene_sdf(self.box_mass):
            subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-name', name, '-string', sdf,
                            '-x', str(x), '-y', str(y), '-z', str(z)],
                           capture_output=True, timeout=30)
        self.get_logger().info(f'Spawned a table and a {self.box_mass:.0f} kg box.')
        self.urdf = None
        self.position = None
        self.torque = None
        self.stage = -1
        self.stage_t0 = 0.0
        self.start = None
        self.last_tick = -1.0
        self.done = False
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.create_subscription(
            String, '/robot_description', lambda m: setattr(self, 'urdf', m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 1)

    def _palms_now(self):
        """Return the current palm poses {side: (position, rotation)} from the QP's model."""
        w = self.torque.wbc
        pin.framesForwardKinematics(w.model, w.data, self.q)
        return {s: (w.data.oMf[f].translation.copy(), w.data.oMf[f].rotation.copy())
                for s, f in w.palms.items()}

    def _at_limits(self, s):
        m, w = self.torque.wbc.model, self.torque.wbc
        out = []
        for j, (name, x) in enumerate(zip(N, s.position)):
            i = w.iq[j]
            lo, hi = m.lowerPositionLimit[i], m.upperPositionLimit[i]
            if x < lo + 0.03 or x > hi - 0.03:
                out.append(f'{name}={x:+.2f}[{lo:+.2f},{hi:+.2f}]')
        return ' '.join(out)

    def _palm_error(self):
        """Return the worst palm position error (m) against the current targets."""
        if not self.torque.hand_targets:
            return 0.0
        w = self.torque.wbc
        return max(np.linalg.norm(w.data.oMf[f].translation - self.torque.hand_targets[s][0])
                   for s, f in w.palms.items())

    def _hand_targets(self, t):
        """Interpolate the palm targets between waypoints (position and rotation)."""
        name, duration, goal = WAYPOINTS[self.stage]
        goal = goal or self.home
        a = smooth((t - self.stage_t0) / duration)
        out = {}
        for side in 'lr':
            p0, r0 = self.start[side]
            p1, r1 = goal[side]
            p1 = self.origin + np.asarray(p1)  # soles-relative -> the QP's frame
            rot = r0 @ pin.exp3(a * pin.log3(r0.T @ r1))
            out[side] = (p0 + a * (np.asarray(p1) - p0), rot)
        # The QP carries the box's weight from lift-off until it is let go.
        if name == 'lift':
            self.torque.wbc.set_payload(self.box_mass * min(1.0, 2 * a), BOX_SIZE / 2)
        elif name == 'release':
            self.torque.wbc.set_payload(self.box_mass * (1 - a), BOX_SIZE / 2)
        if t - self.stage_t0 >= duration:
            self.start = {s: (self.origin + np.asarray(goal[s][0]), goal[s][1]) for s in 'lr'}
            self.stage_t0 = t
            self.stage += 1
            if self.stage < len(WAYPOINTS):
                self.get_logger().info(f'stage: {WAYPOINTS[self.stage][0]}')
        return out

    def _on_state(self, s):
        if self.urdf is None or len(s.position) != len(N) or self.done:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        if self.position is None:
            self.position = bc.FreeWalkController(s)
            self.get_logger().info('Took over; crouching in position mode.')
        roll, pitch = bc.rpy(s.orientation)
        if max(abs(roll), abs(pitch)) > bc.FALL_TILT:
            self.done = True
            self.get_logger().error(f'Fell (roll {math.degrees(roll):+.0f}, '
                                    f'pitch {math.degrees(pitch):+.0f}); stopped.')
            return
        cmd = AtlasCommand()
        cmd.k_effort = [255] * len(N)
        cmd.header.stamp = s.header.stamp
        if self.torque is None:
            if t - self.last_tick < 0.005 - 1e-4:
                return
            self.last_tick = t
            out = self.position.update(s, t)
            cmd.position, cmd.effort = out.position, out.effort
            cmd.kp_position, cmd.kd_position = out.kp, out.kd
            if self.position.status == 'ready':
                self.torque = tb.TorqueBalance(self.urdf, N, s, {'stepping': False})
                self.ready_t = t
                self.get_logger().info('Standing; torque control.')
        else:
            if self.stage < 0 and t - self.ready_t > 2.0:
                o, w = s.orientation, s.angular_velocity
                self.q, _ = self.torque.wbc.state(
                    np.array(s.position), np.array(s.velocity), [o.x, o.y, o.z, o.w],
                    [w.x, w.y, w.z], 'lr')
                self.start = self._palms_now()
                soles = self.torque.wbc.sole_world
                self.origin = (soles['l'] + soles['r']) / 2
                self.origin[2] = min(soles['l'][2], soles['r'][2])  # sole frames: on the ground
                self.home = {k: (p - self.origin, r) for k, (p, r) in self.start.items()}
                self.stage, self.stage_t0 = 0, t
                self.get_logger().info(f'stage: {WAYPOINTS[0][0]}')
            if 0 <= self.stage < len(WAYPOINTS):
                self.torque.hand_targets = self._hand_targets(t)
            elif self.stage >= len(WAYPOINTS) and not getattr(self, 'finished', False):
                self.finished = True
                self.torque.hand_targets = {}
                self.get_logger().info('Pick-and-place finished.')
            tau = self.torque.update(s, t)
            if self.debug and t - self.last_debug >= 0.5:
                self.last_debug = t
                self.get_logger().info(
                    f'dbg roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f} '
                    f'com={np.round(self.torque.com, 3)} '
                    f'fz=({-s.l_foot.force.z:.0f},{-s.r_foot.force.z:.0f}) '
                    f'palm_err={self._palm_error():.3f} qp={self.torque.wbc.last.get("status")} '
                    f'at_limit={self._at_limits(s)}')
                if self.torque.hand_targets:
                    w = self.torque.wbc
                    self.get_logger().info('dbg ' + ' '.join(
                        f'{k}: palm={np.round(w.data.oMf[f].translation - self.origin, 2)} '
                        f'target={np.round(self.torque.hand_targets[k][0] - self.origin, 2)}'
                        for k, f in w.palms.items()))
            cmd.position = [0.0] * len(N)
            cmd.effort = [float(x) for x in tau]
            cmd.kp_position = [0.0] * len(N)
            cmd.kd_position = [1.0] * len(N)
            cmd.desired_controller_period_ms = int(self.sync_ms)
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
