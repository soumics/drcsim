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

"""Unit tests for balance_controller.py's sequencing, with a fake AtlasState."""

import math
import pathlib
import sys
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'scripts'))

import balance_controller as bc  # noqa: E402
import pytest  # noqa: E402
import zmp_walk as zw  # noqa: E402

N = len(bc.N)


def _state(pitch=0.0, position=None):
    return SimpleNamespace(
        position=list(position or [0.0] * N), effort=[10.0] * N, kp_position=[100.0] * N,
        kd_position=[1.0] * N,
        orientation=SimpleNamespace(w=math.cos(pitch / 2), x=0.0, y=math.sin(pitch / 2), z=0.0),
        angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0))


def _run(controller, state, t0, duration):
    outputs = []
    for k in range(int(round(duration / zw.DT))):
        outputs.append(controller.update(state, t0 + k * zw.DT))
    return outputs


def test_takes_over_the_setpoint_then_crouches_settles_and_becomes_ready():
    c = bc.FreeWalkController(_state())
    first = c.update(_state(), 100.0)
    assert first.position[0] == pytest.approx(0.1)  # position + effort / kp
    assert c.status == 'crouch'
    assert first.kp[bc.N.index('l_leg_hpx')] == bc.GAIN_OVERRIDES['hpx'][0]
    _run(c, _state(), 100.0, bc.CROUCH_TIME + 0.1)
    assert c.status == 'settle'
    _run(c, _state(), 100.0 + bc.CROUCH_TIME + 0.1, bc.SETTLE_TIME)
    assert c.status == 'ready'


def test_walk_command_given_early_is_applied_once_ready():
    c = bc.FreeWalkController(_state())
    c.set_velocity(0.1, 0.0, 0.0)
    _run(c, _state(), 0.0, bc.CROUCH_TIME + 0.5)
    assert c.walker.idle
    _run(c, _state(), bc.CROUCH_TIME + 0.5, bc.SETTLE_TIME)
    assert c.walking


def test_a_fall_stops_commanding_and_recovery_gets_up_through_the_harness():
    c = bc.FreeWalkController(_state())
    _run(c, _state(), 0.0, 1.0)
    assert c.update(_state(pitch=math.radians(60)), 1.0) is None
    assert c.status == 'fallen'
    assert c.update(_state(pitch=math.radians(60)), 1.1) is None
    c.recover()
    lying = _state(pitch=math.radians(80), position=[0.5] * N)
    outputs = _run(c, lying, 2.0, bc.RECOVER_RELEASE + zw.DT)
    modes = [o.mode for o in outputs if o.mode]
    assert modes == ['recover', 'nominal']
    assert outputs[0].position == pytest.approx([0.5] * N)  # blends from where it lies
    height = bc.RECOVER_Z + sum(o.harness_vz for o in outputs) * zw.DT
    assert height == pytest.approx(bc.STAND_Z - bc.RECOVER_PRELOAD, abs=0.002)
    assert c.status == 'settle'
