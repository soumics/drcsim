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
Keyboard-controlled statically-stable stepping for Atlas.

Run *after* atlas.launch.py has Atlas standing. Atlas stands because
AtlasPlugin's per-joint PID (k_effort=255, real gains from
atlas_v5_gains.yaml) holds every joint toward its current setpoint --
all zeros by default -- and gravity sags each joint slightly *below* that
setpoint. That sag is the proportional error that produces the holding
torque (these gains have no integral term). So this node must take over
the controller's *setpoint*, not the measured pose: commanding the
measured (sagged) positions zeroes the error, drops the holding torque to
nothing, and Atlas collapses. The setpoint isn't published directly, so it
is reconstructed from atlas/atlas_state as

    setpoint = measured_position + applied_effort / kp_position

(exact at rest; see CLAUDE.md for how this was established). k_effort is
kept at 255 throughout -- the value AtlasPlugin already uses -- so taking
over is bumpless: no gain change, no controller reset.

Keys: w = start/continue walking forward, space or s = stop (finishes the
current step, then returns to the neutral stance), q or Ctrl-C = quit.

By default (parameter harness:=true) the node first puts Atlas in
VRCPlugin's "pinned_with_gravity" mode -- the pelvis held like a lab
gantry harness -- lowers the harness a little and runs harness_gait.py's
foot-trajectory gait (leg IK, planted stance feet, arm swing), moving the
held pelvis through atlas/cmd_vel in step with the feet. The node runs on
sim time so its pelvis velocity integrates exactly as VRCPlugin's does,
and stance feet stay put on the floor. harness:=false runs the free-standing
lean-lift-plant WALK_CYCLE instead, which still needs a balance controller
to get further than a step or two (see README.md). The harness is left on
when the node quits; publish "nominal" on atlas/mode to release it.

ROS spinning runs on its own executor thread, decoupled from the blocking
keyboard read in main(). Raw keypress reading follows ROS 2's
teleop_twist_keyboard.py termios/tty pattern.
"""

import select
import sys
import termios
import threading
import tty

from atlas_msgs.msg import AtlasCommand, AtlasState
from gait_controller import ATLAS_JOINT_NAMES, GaitController
from geometry_msgs.msg import Twist
from harness_gait import HarnessGait
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

PUBLISH_RATE_HZ = 30.0

# (kp, kd) sent in AtlasCommand for the leg yaw/roll joints while walking
# in the harness. atlas_v5_gains.yaml's values (hpz p=5, hpx p=900, akx
# p=300) leave them nearly limp: measured over a harness walk, foot
# friction swung hip yaw across its whole +-45 deg range and hip roll
# +-24 deg while both were commanded 0 -- the "drunk" gait. The gait never
# moves these joints, so they only need to hold still.
HARNESS_GAIN_OVERRIDES = {
    'hpz': (1000.0, 10.0),
    'hpx': (2500.0, 10.0),
    'akx': (1000.0, 3.0),
}
DIAG_PERIOD_SEC = 1.0

INSTRUCTIONS = """
atlas_walking_demo: keyboard-controlled stepping
  w         - start/continue walking forward
  space, s  - stop (finishes the current step, then stands)
  q, Ctrl-C - quit
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


class WalkKeyboardNode(Node):

    def __init__(self):
        super().__init__('atlas_walk_keyboard')
        self.gait = None  # constructed from the first atlas/atlas_state message
        self._last_state = None
        self._last_diag_time = None
        self._last_tick_time = None
        self.harness = self.declare_parameter('harness', True).value
        if self.harness and not self.get_parameter('use_sim_time').value:
            self.set_parameters([Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.mode_pub = self.create_publisher(String, 'atlas/mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'atlas/cmd_vel', 10)
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

    def _on_state(self, msg):
        self._last_state = msg
        if self.gait is not None:
            return
        if len(msg.position) != len(ATLAS_JOINT_NAMES):
            return  # plugin not fully initialized yet
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
        self.gait = HarnessGait(setpoint) if self.harness else GaitController(setpoint)
        legs = ', '.join(
            f'{n}={setpoint[n]:+.3f}' for n in ('l_leg_hpy', 'l_leg_kny', 'l_leg_aky'))
        self.get_logger().info(
            f'Took over the current PID setpoint ({legs}, ...)'
            f'{" and put Atlas in the harness" if self.harness else ""}'
            ' -- ready. Press w to walk.')

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
        velocity = (0.0, 0.0)
        if self.harness:
            positions, velocity = self.gait.sample(dt)
        else:
            positions = self.gait.sample(dt)
        command = AtlasCommand()
        command.header.stamp = now.to_msg()
        command.position = positions
        command.effort = [0.0] * len(ATLAS_JOINT_NAMES)
        command.k_effort = [255] * len(ATLAS_JOINT_NAMES)
        # AtlasPlugin replaces its live gains with any full-length array.
        command.kp_position = self.kp
        command.kd_position = self.kd
        self.pub.publish(command)
        if velocity != (0.0, 0.0):
            # VRCPlugin stops the warp 0.1 s after the last message, so
            # publishing only while moving is what stops the pelvis.
            twist = Twist()
            twist.linear.x, twist.linear.z = velocity
            self.cmd_vel_pub.publish(twist)
        self._log_diagnostics(command.position)

    def _log_diagnostics(self, target):
        """Log commanded vs measured leg pitch once a second (tuning aid)."""
        now = self.get_clock().now()
        if (self._last_state is None or (
                self._last_diag_time is not None and
                (now - self._last_diag_time).nanoseconds / 1e9 < DIAG_PERIOD_SEC)):
            return
        self._last_diag_time = now
        s = self._last_state
        parts = []
        for name in ('l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny'):
            i = ATLAS_JOINT_NAMES.index(name)
            parts.append(f'{name} cmd={target[i]:+.3f} meas={s.position[i]:+.3f}')
        self.get_logger().info(f'[diag] {self.gait.phase_name}: ' + ', '.join(parts))

    def handle_key(self, key):
        if self.gait is None:
            return
        if key == 'w':
            if not self.gait.walking:
                self.get_logger().info('Walking forward.')
            self.gait.start_walking()
        elif key in (' ', 's'):
            if self.gait.walking:
                self.get_logger().info('Stopping (finishing current step).')
            self.gait.stop_walking()


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

    stdin_settings = termios.tcgetattr(sys.stdin.fileno())
    try:
        while rclpy.ok():
            key = _read_key(stdin_settings, timeout_sec=0.05)
            if key in ('q', '\x03'):  # 'q' or Ctrl-C
                break
            if key:
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
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
