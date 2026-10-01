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

"""Tests for pick_place.py's task geometry, route and scene (no simulator)."""

import math
import pathlib
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

pytest.importorskip('rclpy')
pytest.importorskip('ament_index_python')
pin = pytest.importorskip('pinocchio')
try:
    import pick_place as pp  # noqa: E402
except Exception as e:  # atlas_walking_demo's scripts not installed
    pytest.skip(f'pick_place not importable: {e}', allow_module_level=True)


def test_palms_face_each_other_across_the_box():
    palms = pp.palms_at(pp.BOX_CENTER, 0.0)
    (pl, rl), (pr, rr) = palms['l'], palms['r']
    assert pl[1] - pr[1] == pytest.approx(pp.BOX_SIZE)
    # Palm normal (+y of the palm frame) points from each palm to the box.
    assert rl[:, 1] @ (pp.BOX_CENTER - pl) > 0
    assert rr[:, 1] @ (pp.BOX_CENTER - pr) > 0
    for r in (rl, rr):
        assert np.allclose(r.T @ r, np.eye(3)) and np.linalg.det(r) == pytest.approx(1.0)


def test_the_grip_holds_from_grasp_to_release():
    gaps = {name: goal['l'][0][1] - goal['r'][0][1]
            for name, _, goal, _, _ in pp.PICK + pp.PLACE if goal is not None}
    assert gaps['grasp'] < pp.BOX_SIZE < gaps['approach']
    for held in ('lift', 'bring in', 'present', 'reach out', 'lower'):
        assert gaps[held] == pytest.approx(gaps['grasp'])
    assert gaps['release'] > pp.BOX_SIZE + 0.1
    assert [s[4] for s in pp.PICK + pp.PLACE].count('on') == 1
    assert [s[4] for s in pp.PICK + pp.PLACE].count('off') == 1
    assert pp.PLACE[-1][2] is None  # ends back where the hands started
    # Fingers: open to approach, curled round the box while it is held.
    hands = {s[0]: s[3] for s in pp.PICK + pp.PLACE}
    assert hands['approach'] == 'open' and hands['grasp'] == 'box' and hands['lower'] == 'box'
    assert hands['release'] == 'open'
    assert len(pp.tx.HAND_POSES['svh']['box']) == len(pp.tx.SVH_JOINT_NAMES)


def test_the_box_sits_on_table_a_and_table_b_waits_where_the_walk_ends():
    scene = {name: (ET.fromstring(sdf), np.array(pose)) for name, sdf, pose in pp.scene(10.0)}
    for name in ('table_a', 'table_b'):
        table, pose = scene[name]
        assert table.find('.//static').text == 'true'
        top = table.find(".//collision[@name='top_c']")
        z = float(top.find('pose').text.split()[2])
        size = [float(x) for x in top.find('.//size').text.split()]
        assert pose[2] + z + size[2] / 2 == pytest.approx(pp.TABLE_TOP)
        legs = [c for c in table.iter('collision') if c.get('name').startswith('leg')]
        assert len(legs) == 4
        for leg in legs:  # each leg stands on the floor
            lz = float(leg.find('pose').text.split()[2])
            lh = float(leg.find('.//size').text.split()[2])
            assert pose[2] + lz - lh / 2 == pytest.approx(0.0, abs=1e-9)
    box, box_pose = scene['box']
    assert float(box.find('.//mass').text) == 10.0
    assert box_pose[2] - pp.BOX_SIZE / 2 == pytest.approx(pp.TABLE_TOP)
    # Placing puts the box BOX_CENTER ahead of the feet where the walk ends:
    # that must land on table B, inside its top.
    with pp.walk_timing():
        soles, _ = pp.qp_walk.predict(pp.ROUTE)
    mid = (soles['l'][0] + soles['r'][0]) / 2
    yaw = soles['l'][1]
    placed = pp.SOLES_IN_GAZEBO[:2] + mid[:2] + pp.BOX_CENTER[0] * np.array(
        [math.cos(yaw), math.sin(yaw)])
    b = scene['table_b'][1]
    c, s = math.cos(-b[3]), math.sin(-b[3])
    d = placed - b[:2]
    local = (c * d[0] - s * d[1], s * d[0] + c * d[1])
    assert abs(local[0]) + pp.BOX_SIZE / 2 < pp.TABLE_SIZE[0] / 2
    assert abs(local[1]) + pp.BOX_SIZE / 2 < pp.TABLE_SIZE[1] / 2
    # The walk really goes somewhere: table B is well away from table A.
    assert np.linalg.norm(b[:2] - scene['table_a'][1][:2]) > 1.0


def test_body_frame_round_trip():
    frame = pp.BodyFrame([1.0, 2.0, 0.1], 0.7)
    pos, rot = frame.to_world(np.array([0.5, -0.2, 1.0]), pp.R_LEFT)
    back_pos, back_rot = frame.to_body(pos, rot)
    assert np.allclose(back_pos, [0.5, -0.2, 1.0]) and np.allclose(back_rot, pp.R_LEFT)
