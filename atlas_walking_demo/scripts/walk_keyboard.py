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
Keyboard (and Twist) teleoperation of Atlas walking, grasping and kicks.

Run *after* atlas.launch.py has Atlas standing. Atlas stands because
AtlasPlugin's per-joint PID (k_effort=255, real gains from
atlas_v5_gains.yaml) holds every joint toward its current setpoint --
all zeros by default -- and gravity sags each joint slightly *below* that
setpoint. That sag is the proportional error that produces the holding
torque (these gains have no integral term). So this node takes over the
controller's *setpoint*, not the measured pose: commanding the measured
(sagged) positions zeroes the error, drops the holding torque to nothing,
and Atlas collapses. The setpoint is reconstructed from atlas/atlas_state
as

    setpoint = measured_position + applied_effort / kp_position

(exact at rest; see CLAUDE.md for how this was established).

By default (harness:=true) the node then puts Atlas in VRCPlugin's
"pinned_with_gravity" harness, crouches slightly and runs harness_gait.py's
omnidirectional foot-trajectory gait, moving the harness through
atlas/cmd_vel in step with the feet. It runs on sim time so its velocity
integration matches VRCPlugin's. Besides the keys below it follows any
geometry_msgs/Twist on atlas_walk/cmd_vel (joystick, teleop_twist_keyboard,
a planner), which takes priority while messages keep arriving.

harness:=false walks free-standing instead, with no harness:
balance_controller.py's ZMP preview control and balance feedback, at
200 Hz of sim time. Same keys (slower: about 0.15 m steps), plus r to
reset after a fall: VRCPlugin's harness lifts Atlas upright and sets it
back on its feet -- a testing aid, not a get-up motion (auto_recover:=true
does it automatically 2 s after a fall). Walk
commands take effect after about 2.6 s: the preview plans that far ahead.
In harness mode the harness stays on when the node quits; publish
"nominal" on atlas/mode to release it.

ROS spinning runs on its own executor thread, decoupled from the blocking
keyboard read in main(). Raw keypress reading follows ROS 2's
teleop_twist_keyboard.py termios/tty pattern.
"""

import math
import select
import subprocess
import sys
import termios
import threading
import time
import tty

from atlas_msgs.msg import AtlasCommand, AtlasState
import balance_controller
from gait_controller import ATLAS_JOINT_NAMES
from geometry_msgs.msg import Twist
import harness_gait
from harness_gait import HarnessGait
from osrf_msgs.msg import JointCommands
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import teleop_extras as tx
import zmp_walk

PUBLISH_RATE_HZ = 30.0
DIAG_PERIOD_SEC = 2.0
TWIST_TIMEOUT_SEC = 0.5
RECOVER_WAIT_SEC = 2.0  # free-standing, auto_recover: lying this long before the reset

# (kp, kd) sent in AtlasCommand for the leg yaw/roll joints while walking
# in the harness. atlas_v5_gains.yaml's values (hpz p=5, hpx p=900, akx
# p=300) leave them nearly limp: measured over a harness walk, foot
# friction swung hip yaw across its whole +-45 deg range and hip roll
# +-24 deg while both were commanded 0 -- the "drunk" gait. They now also
# steer the feet (turning, side-stepping), so they must track.
HARNESS_GAIN_OVERRIDES = {
    'hpz': (1000.0, 10.0),
    'hpx': (2500.0, 10.0),
    'akx': (1000.0, 3.0),
}

INSTRUCTIONS = """
atlas_walking_demo -- keyboard teleop (harness:=false to walk free-standing)
  w / s     walk forward / backward        a / d   side-step left / right
  q / e     turn left / right              z / c   curve forward left / right
  space, x  stop (finishes the step)       + / -   speed up / down
  g         grip / relax both hands        [ / ]   grip / relax left / right hand
  h         open both hands flat (relax again with g)
  k / l / j push Atlas: from the right / front / behind
  r         after a fall: harness reset onto its feet (free-standing mode)
  Ctrl-C    quit
Also follows geometry_msgs/Twist on atlas_walk/cmd_vel.
"""


def reconstruct_setpoint(state):
    """
    Return {joint: PID setpoint} reconstructed from one AtlasState message.

    Pure function (no rclpy), split out for unit testing.
    """
    setpoint = {}
    for i, name in enumerate(ATLAS_JOINT_NAMES):
        kp = state.kp_position[i]
        position = state.position[i]
        setpoint[name] = position + state.effort[i] / kp if kp > 0.0 else position
    return setpoint


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WalkKeyboardNode(Node):

    def __init__(self):
        super().__init__('atlas_walk_keyboard')
        self.gait = None  # constructed from the first atlas/atlas_state message
        self._last_state = None
        self._last_diag_time = None
        self._last_tick_time = None
        self._twist = None
        self._twist_time = None
        self.speed_index = 2
        self.grippers = {'l': tx.Gripper(), 'r': tx.Gripper()}
        self.harness = self.declare_parameter('harness', True).value
        self.auto_recover = self.declare_parameter('auto_recover', False).value
        self._fell_at = None
        if not self.get_parameter('use_sim_time').value:
            self.set_parameters([Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.mode_pub = self.create_publisher(String, 'atlas/mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'atlas/cmd_vel', 10)
        # Both hand models' command topics; only the spawned one listens.
        self.sandia_pubs = {
            side: self.create_publisher(
                JointCommands, f'sandia_hands/{side}_hand/joint_commands', 10)
            for side in ('l', 'r')}
        self.svh_pubs = {
            side: self.create_publisher(
                JointState, f'svh_hands/{"left" if side == "l" else "right"}/command', 10)
            for side in ('l', 'r')}
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 10)
        self.create_subscription(Twist, 'atlas_walk/cmd_vel', self._on_twist, 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ if self.harness else zmp_walk.DT, self._tick)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._hands_tick)
        self._last_hands_time = None

    @property
    def speed(self):
        return tx.SPEED_STEPS[self.speed_index]

    def _on_state(self, msg):
        self._last_state = msg
        if self.gait is not None:
            return
        if len(msg.position) != len(ATLAS_JOINT_NAMES):
            return  # plugin not fully initialized yet
        if not self.harness:
            self.gait = balance_controller.FreeWalkController(msg)
            self.get_logger().info(
                'Took over the current PID setpoint -- free-standing: crouching and '
                'settling (5 s), then ready.')
            return
        setpoint = reconstruct_setpoint(msg)
        self.kp = list(msg.kp_position)
        self.kd = list(msg.kd_position)
        if self.harness:
            for i, name in enumerate(ATLAS_JOINT_NAMES):
                joint = name.rsplit('_', 1)[1]
                if '_leg_' in name and joint in HARNESS_GAIN_OVERRIDES:
                    self.kp[i], self.kd[i] = HARNESS_GAIN_OVERRIDES[joint]
            # VRCPlugin holds the pelvis where it is right now, gravity on.
            self.mode_pub.publish(String(data='pinned_with_gravity'))
        self.gait = HarnessGait(setpoint)
        self.get_logger().info(
            'Took over the current PID setpoint and put Atlas in the harness '
            '(crouching in 3 s) -- ready.')

    def _on_twist(self, msg):
        self._twist = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._twist_time = time.monotonic()

    def _tick(self):
        if self.gait is None:
            return
        now = self.get_clock().now()
        dt = 1.0 / PUBLISH_RATE_HZ
        if self._last_tick_time is not None:
            dt = (now - self._last_tick_time).nanoseconds / 1e9
        self._last_tick_time = now
        if dt <= 0.0:
            return
        self._apply_twist()
        if not self.harness:
            self._free_tick(now.nanoseconds / 1e9)
            self._log_diagnostics()
            return
        positions, velocity = self.gait.sample(dt)
        command = AtlasCommand()
        command.header.stamp = now.to_msg()
        command.position = positions
        command.effort = [0.0] * len(ATLAS_JOINT_NAMES)
        command.k_effort = [255] * len(ATLAS_JOINT_NAMES)
        # AtlasPlugin replaces its live gains with any full-length array.
        command.kp_position = self.kp
        command.kd_position = self.kd
        self.pub.publish(command)
        if any(velocity):
            # VRCPlugin stops the warp 0.1 s after the last message, so
            # publishing only while moving is what stops the pelvis.
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = velocity
            self.cmd_vel_pub.publish(twist)
        self._log_diagnostics()

    def _free_tick(self, t):
        """Free-standing mode: one balance_controller tick; harness reset after falls."""
        controller = self.gait
        was_fallen = controller.status == 'fallen'
        out = controller.update(self._last_state, t)
        if controller.status == 'fallen':
            if not was_fallen:
                self._fell_at = t
                self.get_logger().error(
                    f'Fell ({controller.fall_info}); '
                    + ('harness reset in 2 s.' if self.auto_recover
                       else 'press r for a harness reset.'))
            elif self.auto_recover and t - self._fell_at >= RECOVER_WAIT_SEC:
                self._recover()
            return
        if out.mode:
            self.mode_pub.publish(String(data=out.mode))
            if out.mode == 'nominal':
                self.get_logger().info('Back on its feet; harness released.')
        if out.harness_vz:
            twist = Twist()
            twist.linear.z = out.harness_vz
            self.cmd_vel_pub.publish(twist)
        command = AtlasCommand()
        command.position, command.effort = out.position, out.effort
        command.k_effort = [255] * len(ATLAS_JOINT_NAMES)
        command.kp_position, command.kd_position = out.kp, out.kd
        self.pub.publish(command)

    def _recover(self):
        if self.gait.status == 'fallen':
            self.get_logger().info(
                'Harness reset: lift upright, legs into the stance, lower, let go.')
            self.gait.stop()
            self.gait.recover()

    def _hands_tick(self):
        now = self.get_clock().now()
        dt = 1.0 / PUBLISH_RATE_HZ
        if self._last_hands_time is not None:
            dt = (now - self._last_hands_time).nanoseconds / 1e9
        self._last_hands_time = now
        if dt <= 0.0:
            return
        for side, gripper in self.grippers.items():
            sandia = JointCommands()
            sandia.name = [f'{"left" if side == "l" else "right"}_{j}'
                           for j in tx.SANDIA_JOINT_NAMES]
            sandia.position = gripper.sample(dt, 'sandia')
            self.sandia_pubs[side].publish(sandia)
            svh = JointState()
            svh.name = list(tx.SVH_JOINT_NAMES)
            svh.position = gripper.sample(dt, 'svh')
            self.svh_pubs[side].publish(svh)

    def _apply_twist(self):
        """Follow atlas_walk/cmd_vel while it is fresh; stop when it goes stale."""
        if self._twist_time is None:
            return
        if time.monotonic() - self._twist_time > TWIST_TIMEOUT_SEC:
            self._twist_time = None
            self.gait.stop()
            return
        self.gait.set_velocity(*self._twist)

    def _log_diagnostics(self):
        now = self.get_clock().now()
        if (self._last_state is None or (
                self._last_diag_time is not None and
                (now - self._last_diag_time).nanoseconds / 1e9 < DIAG_PERIOD_SEC)):
            return
        self._last_diag_time = now
        if self.harness:
            vx, vy, wz = self.gait.velocity
            self.get_logger().info(
                f'[{self.gait.phase_name}] vx={vx:+.2f} vy={vy:+.2f} m/s '
                f'wz={wz:+.2f} rad/s  speed {int(self.speed * 100)}%  '
                f'hands L/R {self.grippers["l"].pose}/{self.grippers["r"].pose}')
        else:
            c = self.gait
            vx, vy, wz = c.walker.command
            self.get_logger().info(
                f'[{c.status} {c.walker.phase}] vx={vx:+.2f} vy={vy:+.2f} m/s '
                f'wz={wz:+.2f} rad/s  speed {int(self.speed * 100)}%')

    def handle_key(self, key):
        if self.gait is None:
            return
        limits = (harness_gait.MAX_VX, harness_gait.MAX_VX_BACK,
                  harness_gait.MAX_VY, harness_gait.MAX_WZ) if self.harness else (
            zmp_walk.max_velocity())
        command = tx.walk_command(key, self.speed, *limits)
        if command is not None:
            self.get_logger().info(
                f'Walk: vx={command[0]:+.2f} vy={command[1]:+.2f} wz={command[2]:+.2f}')
            self.gait.set_velocity(*command)
        elif key in tx.STOP_KEYS:
            self.get_logger().info('Stopping (finishing the current step).')
            self.gait.stop()
        elif key in ('+', '=', '-', '_'):
            step = 1 if key in ('+', '=') else -1
            self.speed_index = max(0, min(len(tx.SPEED_STEPS) - 1, self.speed_index + step))
            self.get_logger().info(f'Speed {int(self.speed * 100)}% (applies to the next key).')
        elif key == 'g':
            gripping = any(g.pose != 'closed' for g in self.grippers.values())
            for gripper in self.grippers.values():
                gripper.pose = 'closed' if gripping else 'relaxed'
            self.get_logger().info(f'{"Gripping" if gripping else "Relaxing"} both hands.')
        elif key == 'h':
            for gripper in self.grippers.values():
                gripper.pose = 'open'
            self.get_logger().info('Opening both hands flat.')
        elif key in ('[', ']'):
            side = 'l' if key == '[' else 'r'
            self.grippers[side].toggle()
            self.get_logger().info(
                f'{"Gripping" if self.grippers[side].pose == "closed" else "Relaxing"} '
                f'{"left" if side == "l" else "right"} hand.')
        elif key in tx.KICK_KEYS:
            self._kick(tx.KICK_KEYS[key])
        elif key == 'r' and not self.harness:
            self._recover()

    def _kick(self, body_direction):
        """Push the torso (direction given in Atlas's own frame)."""
        yaw = _yaw(self._last_state.orientation) if self._last_state else 0.0
        bx, by = body_direction
        world = (bx * math.cos(yaw) - by * math.sin(yaw), bx * math.sin(yaw) + by * math.cos(yaw))
        apply, clear = tx.kick_commands(world)
        self.get_logger().info('Kick!')

        def run():
            subprocess.run(apply, capture_output=True, timeout=5)
            time.sleep(tx.KICK_DURATION)
            subprocess.run(clear, capture_output=True, timeout=5)

        threading.Thread(target=run, daemon=True).start()


def _read_key(settings, timeout_sec):
    """Return one keypress (blocking up to timeout_sec) or '' on timeout."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = WalkKeyboardNode()
    print(INSTRUCTIONS)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Without a terminal (a launch file, docker exec) follow
    # atlas_walk/cmd_vel only.
    stdin_settings = termios.tcgetattr(sys.stdin.fileno()) if sys.stdin.isatty() else None
    if stdin_settings is None:
        node.get_logger().info('No terminal: following atlas_walk/cmd_vel only.')
    try:
        while rclpy.ok():
            if stdin_settings is None:
                time.sleep(0.1)
                continue
            key = _read_key(stdin_settings, timeout_sec=0.05)
            if key == '\x03':  # Ctrl-C
                break
            if key:
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        if stdin_settings is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, stdin_settings)
        # Stop and join the spin thread before tearing the node down;
        # destroying it under a still-spinning executor aborts the process
        # ("terminate called without an active exception").
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
