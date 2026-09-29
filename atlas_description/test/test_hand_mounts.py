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
Check every Atlas-with-hands variant mounts both hands correctly.

Zero-pose forward kinematics from the rendered URDF (arms straight out to
the sides): each hand must reach outward past its wrist link, and the left
hand must be the mirror image of the right one (compared as point sets, as
Robotiq/iRobot use the same, non-mirrored hand model on both sides).
Regression test for two bugs: bare <insert_block> tags (ROS 1 xacro only)
silently dropped every hand's mount transform, and v4/v5 left mounts that
put the left hand back inside the forearm.
"""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

PACKAGE_DIR = Path(__file__).resolve().parent.parent
HAND_XACROS = sorted((PACKAGE_DIR / 'robots').glob('*_hands*.urdf.xacro'))
TOLERANCE = 0.02  # m


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


def _zero_pose_positions(urdf_xml):
    children = {}
    for joint in ET.fromstring(urdf_xml).findall('joint'):
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
    return {name: pos for name, (pos, _) in pose.items()}, children


def _subtree(children, link):
    out, todo = [], [link]
    while todo:
        for child, _, _ in children.get(todo.pop(), []):
            out.append(child)
            todo.append(child)
    return out


def _centroid(points):
    return [sum(p[i] for p in points) / len(points) for i in range(3)]


@pytest.mark.parametrize('xacro_file', HAND_XACROS, ids=lambda p: p.name)
def test_hands_reach_out_past_the_wrist_and_mirror_each_other(xacro_file):
    urdf = subprocess.run(['xacro', str(xacro_file)], capture_output=True, text=True,
                          cwd=str(PACKAGE_DIR), check=True).stdout
    pos, children = _zero_pose_positions(urdf)
    hands = {side: [pos[n] for n in _subtree(children, f'{side}_hand')] for side in 'lr'}
    if not hands['l'] and not hands['r']:
        pytest.skip('hand stumps only')
    assert hands['l'] and hands['r'], 'only one hand attached'

    for side in 'lr':
        wrist_y = abs(pos[f'{side}_hand'][1])
        out = abs(_centroid(hands[side])[1]) - wrist_y
        assert out > 0.03, f'{side} hand centroid is {100 * out:+.1f} cm past its wrist'

    left, right = _centroid(hands['l']), _centroid(hands['r'])
    mirrored = [right[0], -right[1], right[2]]
    assert math.dist(left, mirrored) < TOLERANCE, (
        f'left hand centroid {left} is not the mirror of the right {right}')
    mirrored_right = [[p[0], -p[1], p[2]] for p in hands['r']]
    worst = max(min(math.dist(p, m) for m in mirrored_right) for p in hands['l'])
    assert worst < TOLERANCE, (
        f'a left hand link is {100 * worst:.1f} cm from any mirrored right hand link')
