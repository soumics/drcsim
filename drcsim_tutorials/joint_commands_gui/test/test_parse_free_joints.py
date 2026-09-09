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

"""Unit tests for joint_command_gui.py's parse_free_joints(), no rclpy/Tk needed."""

import importlib.util
import math
import pathlib

_SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'joint_command_gui.py')
_SPEC = importlib.util.spec_from_file_location('joint_command_gui_script', _SCRIPT_PATH)
joint_command_gui_script = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(joint_command_gui_script)

_URDF = """<?xml version="1.0"?>
<robot name="test">
  <joint name="back_bkz" type="revolute">
    <limit lower="-0.7" upper="0.7" effort="1" velocity="1"/>
  </joint>
  <joint name="neck_ry" type="continuous"/>
  <joint name="not_in_atlas_list" type="revolute">
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="head_camera_fixed" type="fixed"/>
  <joint name="back_bky" type="revolute">
    <limit lower="0.2" upper="1.0" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def test_filters_to_allowed_names_and_skips_fixed():
    allowed = {'back_bkz', 'back_bky', 'neck_ry'}
    joints = joint_command_gui_script.parse_free_joints(_URDF, allowed)

    assert set(joints.keys()) == {'back_bkz', 'back_bky', 'neck_ry'}


def test_revolute_zero_within_range_uses_zero():
    joints = joint_command_gui_script.parse_free_joints(_URDF, {'back_bkz'})

    assert joints['back_bkz'] == {'min': -0.7, 'max': 0.7, 'zero': 0.0}


def test_revolute_range_excluding_zero_uses_midpoint():
    joints = joint_command_gui_script.parse_free_joints(_URDF, {'back_bky'})

    assert joints['back_bky']['zero'] == 0.6


def test_continuous_joint_uses_plus_minus_pi():
    joints = joint_command_gui_script.parse_free_joints(_URDF, {'neck_ry'})

    assert joints['neck_ry']['min'] == -math.pi
    assert joints['neck_ry']['max'] == math.pi
