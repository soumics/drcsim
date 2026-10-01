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
Observation and action conventions shared by training (MuJoCo) and Gazebo.

Pure numpy, no simulator: the MuJoCo environment and the Gazebo policy node
both build the policy's input and read its output through these functions,
so the two can't drift apart.

Observation (51): gravity direction in the pelvis frame (3), pelvis angular
velocity in the pelvis frame x 0.25 (3), controlled joint angles minus the
stance (15), controlled joint velocities x 0.1 (15), previous action (15).
Action (15): joint target offsets for the legs and back, ACTION_SCALE rad
per unit, around the crouched stance; the arms and neck hold their rest.
"""

import numpy as np

ATLAS_JOINT_NAMES = [
    'back_bkz', 'back_bky', 'back_bkx', 'neck_ry',
    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
    'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx',
    'l_arm_wry2',
    'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx',
    'r_arm_wry2',
]
CONTROLLED = [n for n in ATLAS_JOINT_NAMES if '_leg_' in n or n.startswith('back_')]
CONTROLLED_IDX = np.array([ATLAS_JOINT_NAMES.index(n) for n in CONTROLLED])
ACTION_SCALE = 0.4   # rad per unit action
POLICY_DT = 0.02     # s: 50 Hz
OBS_SIZE = 6 + 3 * len(CONTROLLED)


def gravity_in_body(quat_wxyz):
    """Return the unit gravity vector (world -z) in the body frame."""
    w, x, y, z = quat_wxyz
    # Third row of R (world z axis in body coordinates), negated.
    return -np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def observation(quat_wxyz, omega_body, q, qd, stance, last_action):
    """
    Return the policy observation.

    quat_wxyz: pelvis orientation; omega_body: pelvis angular velocity in
    the pelvis frame; q, qd: all 30 joint angles / velocities (AtlasCommand
    order); stance: the 30-joint stance; last_action: previous action (15).
    """
    q = np.asarray(q)[CONTROLLED_IDX] - np.asarray(stance)[CONTROLLED_IDX]
    qd = np.asarray(qd)[CONTROLLED_IDX]
    return np.concatenate([gravity_in_body(quat_wxyz), 0.25 * np.asarray(omega_body),
                           q, 0.1 * qd, np.asarray(last_action)]).astype(np.float32)


def joint_targets(action, stance):
    """Return the 30 joint position targets for an action (clipped to [-1, 1])."""
    target = np.array(stance, dtype=np.float64)
    target[CONTROLLED_IDX] += ACTION_SCALE * np.clip(np.asarray(action, dtype=np.float64),
                                                     -1.0, 1.0)
    return target
