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
Check the SVH hands are mounted like human hands (zero pose: T-pose arms).

Fingers (SVH base +z) point out along each arm, palms (base +y) face down,
thumbs (right +x... mirrored on the left) point forward, and the left hand is
the mirror image of the right one.
"""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

PACKAGE_DIR = Path(__file__).resolve().parent.parent
XACRO = PACKAGE_DIR / 'robots' / 'atlas_v5_svh_hands.urdf.xacro'


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def _zero_pose():
    urdf = subprocess.run(['xacro', str(XACRO)], capture_output=True, text=True,
                          check=True).stdout
    children = {}
    for joint in ET.fromstring(urdf).findall('joint'):
        origin = joint.find('origin')
        attrs = origin.attrib if origin is not None else {}
        xyz = [float(v) for v in attrs.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in attrs.get('rpy', '0 0 0').split()]
        children.setdefault(joint.find('parent').get('link'), []).append(
            (joint.find('child').get('link'), xyz, _rpy(*rpy)))
    pose = {'pelvis': ([0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])}
    todo = ['pelvis']
    while todo:
        link = todo.pop()
        pos, rot = pose[link]
        for child, xyz, r in children.get(link, []):
            pose[child] = ([p + d for p, d in zip(pos, _apply(rot, xyz))], _mul(rot, r))
            todo.append(child)
    return pose


def _axis(rot, i):
    return [rot[0][i], rot[1][i], rot[2][i]]


def test_svh_hands_point_along_the_arms_with_palms_down():
    pose = _zero_pose()
    for side, outward in (('r', -1.0), ('l', 1.0)):
        _, rot = pose[f'{side}_svh_base_link']
        fingers, palm = _axis(rot, 2), _axis(rot, 1)
        assert fingers[1] * outward > 0.99, f'{side}: fingers point {fingers}'
        assert palm[2] < -0.99, f'{side}: palm faces {palm}'


def test_svh_hands_mirror_each_other_and_sit_past_the_wrist():
    pose = _zero_pose()
    hand = {s: [p for n, (p, _) in pose.items() if n.startswith(f'{s}_svh_')] for s in 'lr'}
    assert len(hand['l']) == len(hand['r']) > 20
    for side in 'lr':
        wrist_y = abs(pose[f'{side}_hand'][0][1])
        assert min(abs(p[1]) for p in hand[side]) > wrist_y - 0.005, f'{side} hand inside wrist'
    mirrored_right = [[p[0], -p[1], p[2]] for p in hand['r']]
    worst = max(min(math.dist(p, m) for m in mirrored_right) for p in hand['l'])
    assert worst < 0.01, f'a left hand link is {100 * worst:.1f} cm from any mirrored right one'
