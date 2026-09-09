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
this whole session. Locking in the very *first* joint_states message
unconditionally still fell, straight backward, even with no gait phase
ever requested -- consistent with grabbing a snapshot while Atlas was
still mid-transient right after unpinning (still actively moving, not at
its real steady state yet) and then commanding that transient snapshot
as a fixed PID target forever after. See gait_controller.py's
GaitController docstring for the fuller history of this pose's
back-and-forth (two earlier, different bugs, both interactively caught).

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
from gait_controller import ATLAS_JOINT_NAMES, GaitController
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState

PUBLISH_RATE_HZ = 30.0

# How settled atlas/joint_states must be, over how long a window, before
# trusting a snapshot of it as a fixed PID target -- see the module
# docstring on why the very first message alone isn't good enough.
STABILITY_WINDOW_SEC = 1.0
STABILITY_THRESHOLD_RAD = 0.01

INSTRUCTIONS = """
atlas_walking_demo: keyboard-controlled stepping
  w      - start/continue walking forward
  space, s - stop (finishes the current step, then stands centered)
  q, Ctrl-C - quit
"""


class WalkKeyboardNode(Node):

    def __init__(self):
        super().__init__('atlas_walk_keyboard')
        self.gait = None  # constructed once a real, settled starting pose is known
        self._recent_poses = []  # [(Time, {name: position}), ...], newest last
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.joint_states_sub = self.create_subscription(
            JointState, 'atlas/joint_states', self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        if self.gait is not None:
            return  # already initialized from a settled starting pose
        pose = dict(zip(msg.name, msg.position))
        if not all(name in pose for name in ATLAS_JOINT_NAMES):
            return  # not a full report yet (e.g. mid-spawn); wait for one that is

        now = self.get_clock().now()
        self._recent_poses.append((now, pose))
        cutoff = now - Duration(seconds=STABILITY_WINDOW_SEC)
        self._recent_poses = [(t, p) for t, p in self._recent_poses if t >= cutoff]
        if (now - self._recent_poses[0][0]) < Duration(seconds=STABILITY_WINDOW_SEC):
            return  # not enough history yet to judge stability

        oldest_pose = self._recent_poses[0][1]
        max_change = max(
            abs(pose[name] - oldest_pose[name]) for name in ATLAS_JOINT_NAMES)
        if max_change > STABILITY_THRESHOLD_RAD:
            return  # still moving -- e.g. mid-transient right after unpinning

        self.gait = GaitController(pose)
        leg_joints = (
            'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
            'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx')
        leg_summary = ', '.join(f'{name}={pose[name]:.3f}' for name in leg_joints)
        self.get_logger().info(
            f'Atlas has settled into a real, stable starting pose -- ready. '
            f'Press w to walk. Captured leg joints: {leg_summary}')

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
