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
Pick a heavy box off one table, walk it to another table, set it down.

usage: ros2 run atlas_manipulation pick_place.py [--ros-args -p box_mass:=10.0]
Spawns table A (with the box) in front of Atlas and table B where the walk
ROUTE ends (ros_gz_sim create), stands up like torque_stand.py (position
crouch, then the whole-body torque QP), then:
1. pick: both palms come up over table A, close in on the box's sides, the
   fingers curl round it and the palms squeeze 2 cm into it; lift, and
   bring it in to the chest (CARRY);
2. walk: ROUTE (step back, turn right, walk on) under torque control
   (qp_walk) with the box held in the walking body's frame;
3. place: out over table B, down, fingers open, hands back.
While the box is held, the QP's model carries its mass at the palms
(wbc.set_payload). Needs SVH hands (atlas.launch.py's default) and
Pinocchio/ProxQP; runs in lockstep, so start the sim with
sync_max_per_window:=5.0 sync_max_per_step:=0.05.
"""

import math
import os
import subprocess
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
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
import teleop_extras as tx  # noqa: E402
import torque_balance as tb  # noqa: E402

N = bc.N
# All task geometry is in the *body frame*: origin on the ground between the
# soles, x forward, y left. The QP's world frame starts at the pelvis and
# Gazebo's at the spawn point, so neither can be used directly -- sending
# the hands to Gazebo coordinates in the QP's frame put them a metre too
# high and toppled Atlas (measured). While walking, the body frame moves
# with the walking plan.
BOX_SIZE = 0.30                       # m, cube
BOX_CENTER = np.array([0.50, 0.0, 1.0])
SOLES_IN_GAZEBO = np.array([0.032, 0.0, 0.0])  # Atlas spawned at the origin, facing +x
TABLE_TOP = BOX_CENTER[2] - BOX_SIZE / 2
TABLE_SIZE = (0.5, 0.8)               # m, depth x width
TABLE_AHEAD = 0.55                    # m from the soles to a table's centre
GRIP = 0.02                           # m the palms' targets inside the box's sides
SQUEEZE = 200.0                       # N each palm presses on the box (10 kg needs 49 at mu 1)
LIFT = np.array([0.0, 0.0, 0.15])
CARRY = np.array([0.42, 0.0, 1.12])   # box centre while walking: in close, chest high
# Step back from table A, turn right, walk on to table B (vx, vy, wz, steps).
# Walking with the box (zmp_walk constants, set only while it walks -- the
# position-mode crouch shares them): slower than the free walk, the ZMP
# right under the ankles. With the defaults the CoM reached the stance foot
# with no margin left and the turn toppled Atlas (measured).
WALK_TIMING = {'SINGLE_SUPPORT': 1.0, 'DOUBLE_SUPPORT': 0.6, 'ZMP_Y_INSET': 0.0}
# Turning: 0.12 rad per leading step; 0.25 (the cap) toppled Atlas with the box (measured).
ROUTE = [(-0.06, 0.0, 0.0, 4), (0.0, 0.0, -0.0375, 24), (0.08, 0.0, 0.0, 8)]
# Palm orientations (body frame): fingers forward (+x), palms facing the box.
R_LEFT = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
R_RIGHT = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
# SVH finger pose holding the box: flat against its side, like a person's
# hands carrying a box by its sides. Curled fingers came between the palm
# and the box, the weak finger joints took the squeeze and the box slid
# out (measured).
tx.HAND_POSES['svh']['box'] = [0.15, 0.3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]


def palms_at(center, gap):
    """Return {side: (palm position, rotation)} with the palms gap m from the box's sides."""
    half = BOX_SIZE / 2 + gap
    return {'l': (np.asarray(center) + [0.0, half, 0.0], R_LEFT),
            'r': (np.asarray(center) + [0.0, -half, 0.0], R_RIGHT)}


# (name, seconds, palm goals (body frame) or None = where the hands started,
#  hand pose, box held: 'on' ramps the payload in, 'off' out)
PICK = [
    # The hands rest low at the sides: going straight for the box ran them
    # into the side of the table (measured), so they come up over it first.
    ('raise', 2.5, palms_at([0.25, 0.0, BOX_CENTER[2] + 0.15], 0.25), 'open', None),
    ('reach', 2.5, palms_at(BOX_CENTER + [0.0, 0.0, 0.08], 0.15), 'open', None),
    ('pre-grasp', 2.0, palms_at(BOX_CENTER, 0.15), 'open', None),
    ('approach', 2.0, palms_at(BOX_CENTER, 0.02), 'open', None),
    ('grasp', 1.5, palms_at(BOX_CENTER, -GRIP), 'box', None),
    ('lift', 2.5, palms_at(BOX_CENTER + LIFT, -GRIP), 'box', 'on'),
    ('bring in', 2.5, palms_at(CARRY, -GRIP), 'box', None),
]
PLACE = [
    ('present', 1.5, palms_at(CARRY, -GRIP), 'box', None),
    ('reach out', 2.5, palms_at(BOX_CENTER + LIFT, -GRIP), 'box', None),
    ('lower', 2.5, palms_at(BOX_CENTER + [0.0, 0.0, 0.005], -GRIP), 'box', None),
    ('release', 1.5, palms_at(BOX_CENTER, 0.12), 'open', 'off'),
    ('retreat', 2.0, palms_at(BOX_CENTER + [0.0, 0.0, 0.15], 0.15), 'open', None),
    ('clear', 2.0, palms_at([0.25, 0.0, BOX_CENTER[2] + 0.15], 0.25), 'relaxed', None),
    # Back to where the hands started: dropping the hand task with the arms
    # up let the posture task swing them down at once (fell, measured).
    ('home', 2.5, None, 'relaxed', None),
]


class walk_timing:
    """Context: zmp_walk's timing constants set to WALK_TIMING, restored on exit."""

    def __enter__(self):
        self.saved = {k: getattr(qp_walk.zw, k) for k in WALK_TIMING}
        for k, v in WALK_TIMING.items():
            setattr(qp_walk.zw, k, v)

    def __exit__(self, *exc):
        for k, v in self.saved.items():
            setattr(qp_walk.zw, k, v)


def table_b_pose(route=ROUTE):
    """Return table B's centre (x, y) and yaw, body frame of the start: where ROUTE ends."""
    with walk_timing():
        soles, _ = qp_walk.predict(route)
    mid = (soles['l'][0] + soles['r'][0]) / 2
    yaw = soles['r'][1] + math.atan2(math.sin(soles['l'][1] - soles['r'][1]),
                                     math.cos(soles['l'][1] - soles['r'][1])) / 2
    return mid[:2] + TABLE_AHEAD * np.array([math.cos(yaw), math.sin(yaw)]), yaw


def table_sdf(name):
    """
    Return a table: a top on four legs, its origin at the top's centre.

    Solid blocks down to the floor caught Atlas's toes on the last step
    to table B (measured); with legs the feet fit under the top, as a
    person stands at a table.
    """
    d, w = TABLE_SIZE
    thick, leg = 0.04, 0.05
    parts = [('top', f'{d} {w} {thick}', (0.0, 0.0, -thick / 2))]
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        parts.append((f'leg{i}', f'{leg} {leg} {TABLE_TOP - thick}',
                      (sx * (d / 2 - 0.04), sy * (w / 2 - 0.04),
                       -thick - (TABLE_TOP - thick) / 2)))
    body = ''
    for part, size, (x, y, z) in parts:
        pose = f'<pose>{x} {y} {z} 0 0 0</pose>'
        body += (f'<collision name="{part}_c">{pose}<geometry><box><size>{size}</size></box>'
                 '</geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>'
                 f'</surface></collision><visual name="{part}_v">{pose}<geometry><box>'
                 f'<size>{size}</size></box></geometry><material>'
                 '<diffuse>0.45 0.32 0.22 1</diffuse></material></visual>')
    return (f'<sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<link name="link">{body}</link></model></sdf>')


def box_sdf(mass):
    inertia = mass * BOX_SIZE ** 2 / 6
    size = f'{BOX_SIZE} {BOX_SIZE} {BOX_SIZE}'
    return (f'<sdf version="1.9"><model name="box"><link name="link"><inertial>'
            f'<mass>{mass}</mass><inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy>'
            f'<izz>{inertia}</izz></inertia></inertial>'
            f'<collision name="c"><geometry><box><size>{size}</size></box></geometry>'
            '<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface>'
            f'</collision><visual name="v"><geometry><box><size>{size}</size></box></geometry>'
            '<material><diffuse>0.9 0.55 0.1 1</diffuse></material></visual>'
            '</link></model></sdf>')


def scene(box_mass):
    """
    Return [(name, SDF, (x, y, z, yaw)), ...] in Gazebo's frame.

    ros_gz_sim create ignores the SDF's own <pose> (it uses its -x/-y/-z,
    default 0): models landed on Atlas until the pose was passed there.
    """
    a = SOLES_IN_GAZEBO[:2] + [TABLE_AHEAD, 0.0]
    b, yaw = table_b_pose()
    b = SOLES_IN_GAZEBO[:2] + b
    box = SOLES_IN_GAZEBO + BOX_CENTER
    return [('table_a', table_sdf('table_a'), (a[0], a[1], TABLE_TOP, 0.0)),
            ('table_b', table_sdf('table_b'), (b[0], b[1], TABLE_TOP, yaw)),
            ('box', box_sdf(box_mass), (box[0], box[1], box[2], 0.0))]


def smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


class BodyFrame:
    """A body frame: ground point (3,) and heading; maps body-frame poses to the world."""

    def __init__(self, origin, yaw):
        self.origin, self.yaw = np.asarray(origin, dtype=float), float(yaw)

    def rz(self):
        return pin.rpy.rpyToMatrix(0.0, 0.0, self.yaw)

    def to_world(self, pos, rot):
        return self.origin + self.rz() @ np.asarray(pos), self.rz() @ rot

    def to_body(self, pos, rot):
        return self.rz().T @ (np.asarray(pos) - self.origin), self.rz().T @ rot


class PickPlaceNode(Node):
    """Stand under torque control, pick the box, walk it over, place it."""

    def __init__(self):
        super().__init__('atlas_pick_place',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.box_mass = float(self.declare_parameter('box_mass', 10.0).value)
        self.sync_ms = self.declare_parameter('sync_period_ms', 2).value
        self.debug = self.declare_parameter('debug', False).value
        self.w_hand = float(self.declare_parameter('w_hand', 200.0).value)
        self.w_com = float(self.declare_parameter('w_com', 10000.0).value)
        self.k_dcm = float(self.declare_parameter('k_dcm', 3.0).value)
        self.w_rot = float(self.declare_parameter('w_rot', 100.0).value)
        self.squeeze = float(self.declare_parameter('squeeze', SQUEEZE).value)
        for name, value in WALK_TIMING.items():
            WALK_TIMING[name] = float(self.declare_parameter(name.lower(), value).value)
        for name, sdf, (x, y, z, yaw) in scene(self.box_mass):
            subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-name', name, '-string', sdf,
                            '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)],
                           capture_output=True, timeout=30)
        self.get_logger().info(f'Spawned tables A and B and a {self.box_mass:.0f} kg box.')
        self.urdf = None
        self.position = None
        self.torque = None
        self.phase = 'stand'      # stand, pick, walk, place, done
        self.stages, self.stage = [], 0
        self.last_tick = -1.0
        self.last_debug = -1.0
        self.last_hand = -1.0
        self.done = False
        self.grippers = {side: tx.Gripper() for side in 'lr'}
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.hand_pubs = {side: self.create_publisher(
            JointState, f'svh_hands/{"left" if side == "l" else "right"}/command', 10)
            for side in 'lr'}
        self.create_subscription(
            String, '/robot_description', lambda m: setattr(self, 'urdf', m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 1)

    # --- frames ----------------------------------------------------------

    def _feet_frame(self, q):
        """Return the body frame at the measured soles."""
        soles = self.torque._soles(q)
        (pl, yl), (pr, yr) = soles['l'], soles['r']
        origin = (pl + pr) / 2
        origin[2] = min(pl[2], pr[2])
        return BodyFrame(origin, yr + math.atan2(math.sin(yl - yr), math.cos(yl - yr)) / 2)

    def _pelvis(self):
        """Return the measured pelvis (x, y) and yaw in the QP's world frame."""
        q = self.torque.q
        x, y, z, w = q[3:7]
        return q[:2].copy(), math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _walk_frame(self):
        """
        Return the body frame carried by the pelvis while walking.

        The box is held still relative to the chest, like a person carrying
        it: holding it in the walking *plan's* frame twisted it against the
        torso whenever the body lagged the plan in a turn, and the stance
        foot's yaw torque grew step by step until Atlas fell (measured).
        """
        xy, yaw = self._pelvis()
        c, s = math.cos(yaw), math.sin(yaw)
        off = self.walk_offset
        return BodyFrame([xy[0] + c * off[0] - s * off[1],
                          xy[1] + s * off[0] + c * off[1], self.frame.origin[2]],
                         yaw + self.walk_dyaw)

    def _palms_now(self, q):
        w = self.torque.wbc
        pin.framesForwardKinematics(w.model, w.data, q)
        return {s: (w.data.oMf[f].translation.copy(), w.data.oMf[f].rotation.copy())
                for s, f in w.palms.items()}

    # --- task sequencing -------------------------------------------------

    def _begin(self, phase, stages, t):
        self.phase, self.stages, self.stage, self.stage_t0 = phase, stages, 0, t
        self.start = dict(self.torque.hand_targets)
        self.get_logger().info(f'stage: {stages[0][0]}')

    def _run_stages(self, t):
        """Interpolate the palms through self.stages; return True when all are done."""
        name, duration, goal, hand, payload = self.stages[self.stage]
        a = smooth((t - self.stage_t0) / duration)
        goals = {s: self.frame.to_world(*(goal or self.home)[s]) for s in 'lr'}
        out = {}
        for side in 'lr':
            p0, r0 = self.start[side]
            p1, r1 = goals[side]
            out[side] = (p0 + a * (p1 - p0), r0 @ pin.exp3(a * pin.log3(r0.T @ r1)))
        self.torque.hand_targets = out
        for g in self.grippers.values():
            g.pose = hand
        if name == 'grasp':
            self.torque.squeeze = self.squeeze * a
        if payload == 'on':
            self.torque.wbc.set_payload(self.box_mass * min(1.0, 2 * a), BOX_SIZE / 2)
        elif payload == 'off':
            self.torque.wbc.set_payload(self.box_mass * (1 - a), BOX_SIZE / 2)
            self.torque.squeeze = self.squeeze * (1 - min(1.0, 2 * a))
        if t - self.stage_t0 < duration:
            return False
        self.start, self.stage_t0 = goals, t
        self.stage += 1
        if self.stage >= len(self.stages):
            return True
        self.get_logger().info(f'stage: {self.stages[self.stage][0]}')
        return False

    def _task(self, s, t):
        tq = self.torque
        if self.phase == 'stand' and t - self.ready_t > 2.0:
            o, w = s.orientation, s.angular_velocity
            q, _ = tq.wbc.state(np.array(s.position), np.array(s.velocity),
                                [o.x, o.y, o.z, o.w], [w.x, w.y, w.z], 'lr')
            self.frame = self._feet_frame(q)
            tq.hand_targets = self._palms_now(q)
            self.home = {k: self.frame.to_body(*v) for k, v in tq.hand_targets.items()}
            self._begin('pick', PICK, t)
        elif self.phase == 'pick' and self._run_stages(t):
            self.timing = walk_timing()
            self.timing.__enter__()
            tq.start_walk(s, t)
            self.route = qp_walk.Route(ROUTE)
            xy, yaw = self._pelvis()
            off = self.frame.origin[:2] - xy
            c, sn = math.cos(-yaw), math.sin(-yaw)
            self.walk_offset = (c * off[0] - sn * off[1], sn * off[0] + c * off[1])
            self.walk_dyaw = self.frame.yaw - yaw
            self.phase = 'walk'
            self.walk_t0 = t
            self.get_logger().info('stage: walk to table B')
        elif self.phase == 'walk':
            if self.route.update(tq.walk):
                tq.end_walk()
                self.timing.__exit__()
                o, w = s.orientation, s.angular_velocity
                q, _ = tq.wbc.state(np.array(s.position), np.array(s.velocity),
                                    [o.x, o.y, o.z, o.w], [w.x, w.y, w.z], 'lr')
                self.frame = self._feet_frame(q)
                self._begin('place', PLACE, t)
            else:
                self.frame = self._walk_frame()
                tq.hand_targets = {k: self.frame.to_world(*v)
                                   for k, v in palms_at(CARRY, -GRIP).items()}
        elif self.phase == 'place' and self._run_stages(t):
            self.phase = 'done'
            tq.hand_targets = {}
            self.get_logger().info('Pick-and-place finished.')

    # --- control loop ----------------------------------------------------

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
                self.torque = tb.TorqueBalance(self.urdf, N, s, {
                    'stepping': False, 'w_hand': self.w_hand, 'w_com': self.w_com,
                    'k_dcm': self.k_dcm, 'w_rot': self.w_rot})
                self.ready_t = t
                self.get_logger().info('Standing; torque control.')
        else:
            self._task(s, t)
            tau = self.torque.update(s, t)
            cmd.position = [0.0] * len(N)
            cmd.effort = [float(x) for x in tau]
            cmd.kp_position = [0.0] * len(N)
            cmd.kd_position = [1.0] * len(N)
            cmd.desired_controller_period_ms = int(self.sync_ms)
            self._log(t, roll, pitch, s)
        self.pub.publish(cmd)
        self._hands(t)

    def _hands(self, t):
        """Send the SVH finger targets (30 Hz of sim time, rate-limited)."""
        if t - self.last_hand < 1 / 30:
            return
        dt = min(0.1, t - self.last_hand) if self.last_hand > 0 else 0.0
        self.last_hand = t
        for side, g in self.grippers.items():
            msg = JointState()
            msg.name = list(tx.SVH_JOINT_NAMES)
            msg.position = [float(x) for x in g.sample(dt, 'svh')]
            self.hand_pubs[side].publish(msg)

    def _log(self, t, roll, pitch, s):
        """With debug: tilt, foot loads, palm error, QP health, walk state (2 Hz)."""
        if not self.debug or t - self.last_debug < 0.5:
            return
        self.last_debug = t
        tq = self.torque
        err = 0.0
        if tq.hand_targets:
            err = max(np.linalg.norm(tq.wbc.data.oMf[f].translation - tq.hand_targets[k][0])
                      for k, f in tq.wbc.palms.items())
        walk = ''
        if tq.walk:
            c_ref = tq.walk.com_reference(t)[0]
            walk = (f' walk={tq.walk.phase} steps={tq.walk.steps_planned} '
                    f'com_err={np.round(100 * (tq.com[:2] - c_ref), 1)} cm')
        self.get_logger().info(
            f'dbg {self.phase} roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f} '
            f'fz=({-s.l_foot.force.z:.0f},{-s.r_foot.force.z:.0f}) palm_err={err:.3f} '
            f'squeeze={tq.squeeze:.0f} qp_failures={tq.wbc.last.get("failures", 0)}{walk}')


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
