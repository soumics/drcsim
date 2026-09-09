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

Run *after* atlas.launch.py already has Atlas standing normally (unpinned,
under AtlasPlugin's regular PID control) -- this node only ever publishes
AtlasCommand, the same live topic AtlasCommandController::SetPIDStand()
and drcsim_tutorials/atlas_teleop.py already use; it makes no changes to
any plugin or launch file.

Waits for atlas/joint_states (AtlasPlugin's real, current joint
positions) to actually settle -- not just arrive once -- before
publishing anything at all, and hands that in as GaitController's
neutral_pose: the only pose ever actually proven stable free-standing
this whole session.

The very first command is sent through the atlas/reset_controls
*service*, not the atlas/atlas_command topic -- confirmed the hard way
that going straight to the topic isn't safe here. AtlasPlugin::
ZeroAtlasCommand() (called once at startup, before any AtlasCommand
message ever arrives) sets k_effort=0 for every joint, meaning Atlas has
been standing this entire session on the AtlasSimInterface/BDI
feedforward path, not real user-PID at all. AtlasPlugin::
UpdatePIDControl()'s integral term accumulates every tick *regardless of
k_effort*, even while that path contributes nothing to the output -- so
with atlasCommand.position stuck at 0.0 the whole time while the real
position drifted a little under the BDI path, that integral term had
likely been quietly winding up (clamped, but still substantial) for the
entire session. The instant k_effort flips to 255 for the first time,
that pent-up integral applies all at once as a torque spike -- which is
exactly what a straight-to-topic first command did, immediately, with no
gait phase ever requested and joint values that were otherwise
completely sane (confirmed interactively). atlas/reset_controls's
reset_pid_controller flag zeroes AtlasPlugin's error terms (integral
included), and can atomically apply the new AtlasCommand in the same
call -- a clean, bumpless handoff onto the user-PID path instead.

See gait_controller.py's GaitController docstring for the fuller history
of this pose's back-and-forth (two earlier, different bugs, both
interactively caught, before this one).

Keys: w = start/continue walking forward, space or s = stop (finishes the
current step, then returns to a centered stand), q or Ctrl-C = quit.
Turning is not implemented in this first version.

Raw single-keypress reading follows the same termios/tty pattern ROS 2's
own well-known teleop_twist_keyboard.py uses -- there was no existing
keyboard-input precedent anywhere in this repo to follow instead (every
prior manual-input tool here is Joy-message-based, e.g. atlas_teleop.py),
and that pattern doesn't fit this node's discrete walk/stop commands
naturally.
"""

import sys
import termios
import tty

from atlas_msgs.msg import AtlasCommand
from atlas_msgs.srv import ResetControls
from gait_controller import ATLAS_JOINT_NAMES, GaitController
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState

PUBLISH_RATE_HZ = 30.0

# How long to wait, after the first atlas/joint_states message, before
# trusting a later one as a fixed PID target -- see the module docstring
# on why the very first message alone isn't good enough. A strict
# stillness check was tried instead of a fixed delay and never fired at
# all: Atlas's standing pose has a small persistent oscillation by design
# (already observed and expected all session), so "wait until it stops
# changing" can wait forever. A fixed delay past the initial post-unpin
# settling transient sidesteps that -- whatever small oscillation remains
# by then is the same kind already long since confirmed harmless.
SETTLE_DELAY_SEC = 5.0

INSTRUCTIONS = """
atlas_walking_demo: keyboard-controlled stepping
  w      - start/continue walking forward
  space, s - stop (finishes the current step, then stands centered)
  q, Ctrl-C - quit
"""


class WalkKeyboardNode(Node):

    def __init__(self):
        super().__init__('atlas_walk_keyboard')
        self.gait = None  # constructed once the reset_controls handoff succeeds
        self._first_joint_states_time = None
        self._reset_requested = False
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.reset_controls_client = self.create_client(
            ResetControls, 'atlas/reset_controls')
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.joint_states_sub = self.create_subscription(
            JointState, 'atlas/joint_states', self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        if self.gait is not None or self._reset_requested:
            return  # already initialized, or the handoff is already in flight
        pose = dict(zip(msg.name, msg.position))
        if not all(name in pose for name in ATLAS_JOINT_NAMES):
            return  # not a full report yet (e.g. mid-spawn); wait for one that is

        now = self.get_clock().now()
        if self._first_joint_states_time is None:
            self._first_joint_states_time = now
            self.get_logger().info(
                f'Got the first atlas/joint_states message; waiting '
                f'{SETTLE_DELAY_SEC:.0f}s for the post-unpin settling transient '
                'to pass before using a reading as the starting pose.')
        if (now - self._first_joint_states_time) < Duration(seconds=SETTLE_DELAY_SEC):
            return  # still within the fixed settle delay

        self._reset_requested = True
        command = AtlasCommand()
        command.position = [pose[name] for name in ATLAS_JOINT_NAMES]
        command.effort = [0.0] * len(ATLAS_JOINT_NAMES)
        command.k_effort = [255] * len(ATLAS_JOINT_NAMES)
        request = ResetControls.Request()
        request.reset_pid_controller = True
        request.reset_bdi_controller = False
        request.reload_pid_from_ros = False
        request.atlas_command = command
        future = self.reset_controls_client.call_async(request)
        future.add_done_callback(lambda f: self._on_reset_controls_done(f, pose))

    def _on_reset_controls_done(self, future, pose):
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error(
                f'atlas/reset_controls call failed ({future.exception()!r}); not '
                'starting the walk controller.')
            self._reset_requested = False  # allow another attempt on the next reading
            return
        self.gait = GaitController(pose)
        # Log every joint, not just legs, while this is still being
        # diagnosed interactively -- a bad capture in an arm/back/neck
        # joint would be invisible if only legs were ever printed.
        full_summary = ', '.join(f'{name}={pose[name]:.3f}' for name in ATLAS_JOINT_NAMES)
        self.get_logger().info(
            f'Atlas has settled into a real, stable starting pose, and the PID '
            f'handoff succeeded -- ready. Press w to walk. Captured pose: '
            f'{full_summary}')

    def _tick(self):
        if self.gait is None:
            return  # still waiting on the first atlas/joint_states message
        position = self.gait.sample(1.0 / PUBLISH_RATE_HZ)
        command = AtlasCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.position = position
        # No effort feedforward: this pose is only ever a delta away from
        # Atlas's own already-proven-stable neutral pose (see
        # gait_controller.py's module docstring), which itself already
        # holds under plain PID with no feedforward at all.
        command.effort = [0.0] * len(ATLAS_JOINT_NAMES)
        command.k_effort = [255] * len(ATLAS_JOINT_NAMES)
        self.pub.publish(command)

    def handle_key(self, key):
        if self.gait is None:
            return  # still waiting on the first atlas/joint_states message
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
    import select
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = WalkKeyboardNode()
    print(INSTRUCTIONS)

    stdin_settings = termios.tcgetattr(sys.stdin.fileno())
    try:
        while rclpy.ok():
            key = _read_key(stdin_settings, timeout_sec=0.05)
            if key in ('q', '\x03'):  # 'q' or Ctrl-C
                break
            if key:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, stdin_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
