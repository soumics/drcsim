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

Waits for one atlas/joint_states message (AtlasPlugin's real, current
joint positions) before publishing anything at all, and hands that in as
GaitController's neutral_pose -- the only pose ever actually proven
stable free-standing this whole session. See gait_controller.py's
GaitController docstring for the full story of why (an earlier version
of this instead assumed a separately-designed standing pose, and fell
over -- twice, for two different reasons).

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
from rclpy.node import Node
from sensor_msgs.msg import JointState

PUBLISH_RATE_HZ = 30.0

INSTRUCTIONS = """
atlas_walking_demo: keyboard-controlled stepping
  w      - start/continue walking forward
  space, s - stop (finishes the current step, then stands centered)
  q, Ctrl-C - quit
"""


class WalkKeyboardNode(Node):

    def __init__(self):
        super().__init__('atlas_walk_keyboard')
        self.gait = None  # constructed once a real starting pose is known
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.joint_states_sub = self.create_subscription(
            JointState, 'atlas/joint_states', self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        if self.gait is not None:
            return  # already initialized from a first joint_states message
        neutral_pose = dict(zip(msg.name, msg.position))
        if not all(name in neutral_pose for name in ATLAS_JOINT_NAMES):
            return  # not a full report yet (e.g. mid-spawn); wait for one that is
        self.gait = GaitController(neutral_pose)
        self.get_logger().info(
            'Got a real starting pose -- ready. Press w to walk.')

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
