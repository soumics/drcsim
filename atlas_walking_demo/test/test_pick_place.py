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

"""Tests for pick_place.py's task geometry and scene (no simulator)."""

import pathlib
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

pytest.importorskip('rclpy')
pin = pytest.importorskip('pinocchio')
pp = pytest.importorskip('pick_place')


def test_palms_face_each_other_across_the_box():
    palms = pp.palms_at(pp.BOX_CENTER, 0.0)
    (pl, rl), (pr, rr) = palms['l'], palms['r']
    assert pl[1] - pr[1] == pytest.approx(pp.BOX_SIZE)
    # Palm normal (+y of the palm frame) points from each palm to the box.
    assert rl[:, 1] @ (pp.BOX_CENTER - pl) > 0
    assert rr[:, 1] @ (pp.BOX_CENTER - pr) > 0
    for r in (rl, rr):
        assert np.allclose(r.T @ r, np.eye(3)) and np.linalg.det(r) == pytest.approx(1.0)


def test_the_grip_squeezes_and_release_opens():
    gaps = {name: goal['l'][0][1] - goal['r'][0][1]
            for name, _, goal in pp.WAYPOINTS if goal is not None}
    assert gaps['squeeze'] < pp.BOX_SIZE < gaps['approach']
    assert gaps['lift'] == pytest.approx(gaps['squeeze'])
    assert gaps['carry'] == pytest.approx(gaps['squeeze'])
    assert gaps['release'] > pp.BOX_SIZE + 0.1
    assert pp.WAYPOINTS[-1][2] is None  # ends back where the hands started


def test_the_box_sits_on_the_table_and_the_table_holds_both_spots():
    scene = {name: (ET.fromstring(sdf), np.array(pose))
             for name, sdf, pose in pp.scene_sdf(10.0)}
    table, table_pose = scene['table']
    size = [float(x) for x in table.find('.//collision//size').text.split()]
    assert table.find('.//static').text == 'true'
    box, box_pose = scene['box']
    assert float(box.find('.//mass').text) == 10.0
    assert box_pose[2] - pp.BOX_SIZE / 2 == pytest.approx(table_pose[2] + size[2] / 2)
    for y in (box_pose[1], box_pose[1] + pp.CARRY_Y):
        assert abs(y - table_pose[1]) + pp.BOX_SIZE / 2 <= size[1] / 2 + 1e-9
