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
Pure (no rclpy) teleop helpers for walk_keyboard.py: keys, grippers, kicks.

Split out so the key map, gripper blending and kick commands are unit
tested without a running simulation.
"""

# Walking keys -> direction of travel (vx, vy, wz), scaled by the current
# speed setting and each axis's maximum. A game-style layout.
WALK_KEYS = {
    'w': (1.0, 0.0, 0.0),    # forward
    's': (-1.0, 0.0, 0.0),   # backward
    'a': (0.0, 1.0, 0.0),    # side-step left
    'd': (0.0, -1.0, 0.0),   # side-step right
    'q': (0.0, 0.0, 1.0),    # turn left
    'e': (0.0, 0.0, -1.0),   # turn right
    'z': (1.0, 0.0, 1.0),    # forward, curving left
    'c': (1.0, 0.0, -1.0),   # forward, curving right
}
STOP_KEYS = (' ', 'x')
SPEED_STEPS = (0.25, 0.5, 0.75, 1.0)

# Sandia hand finger joints, in the order SandiaHandPlugin expects:
# f0..f2 = index/middle/ring, f3 = thumb; j0 = spread, j1/j2 = curl.
# Grasp poses from the original drcsim atlas_teleop tutorial.
HAND_JOINT_NAMES = [
    'f0_j0', 'f0_j1', 'f0_j2', 'f1_j0', 'f1_j1', 'f1_j2',
    'f2_j0', 'f2_j1', 'f2_j2', 'f3_j0', 'f3_j1', 'f3_j2',
]
GRASP_OPEN = [0.0] * 12
GRASP_CLOSED = [0, 1.5, 1.7, 0, 1.5, 1.7, 0, 1.5, 1.7, -0.2, 0.8, 1.7]  # 'cyl'
GRIP_TIME = 1.0  # s to fully open or close


def walk_command(key, speed, max_vx, max_vx_back, max_vy, max_wz):
    """Return (vx, vy, wz) for a walking key, or None if it isn't one."""
    direction = WALK_KEYS.get(key)
    if direction is None:
        return None
    fx, fy, fw = direction
    vx = fx * speed * (max_vx if fx >= 0.0 else max_vx_back)
    return (vx, fy * speed * max_vy, fw * speed * max_wz)


class Gripper:
    """One hand's grasp level (0 open .. 1 closed), moved at a limited rate."""

    def __init__(self):
        self.level = 0.0
        self.target = 0.0

    def toggle(self):
        self.target = 0.0 if self.target > 0.5 else 1.0

    def sample(self, dt):
        """Advance by dt; return the 12 finger joint targets."""
        step = dt / GRIP_TIME
        self.level += max(-step, min(step, self.target - self.level))
        return [o + self.level * (c - o) for o, c in zip(GRASP_OPEN, GRASP_CLOSED)]


def kick_commands(direction, force=900.0, link='atlas::utorso'):
    """
    Return (apply, clear) gz CLI argument lists for a push on Atlas's torso.

    `direction` is a world-frame (x, y) unit vector. ApplyLinkWrench's
    persistent topic holds the force until cleared; walk_keyboard.py clears
    it after KICK_DURATION -- a 900 N, 0.2 s push is ~180 N s, a hard kick.
    """
    dx, dy = direction
    entity = f'entity: {{name: "{link}", type: LINK}}'
    apply = ['gz', 'topic', '-t', '/world/default/wrench/persistent',
             '-m', 'gz.msgs.EntityWrench', '-p',
             f'{entity}, wrench: {{force: {{x: {force * dx:.1f}, y: {force * dy:.1f}, z: 0}}}}']
    clear = ['gz', 'topic', '-t', '/world/default/wrench/clear',
             '-m', 'gz.msgs.Entity', '-p', f'name: "{link}", type: LINK']
    return apply, clear


KICK_DURATION = 0.2
KICK_KEYS = {
    'k': (0.0, 1.0),    # from the right, pushing left
    'l': (-1.0, 0.0),   # from the front, pushing backward
    'j': (1.0, 0.0),    # from behind, pushing forward
}
