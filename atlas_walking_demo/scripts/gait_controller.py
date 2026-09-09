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
Statically-stable stepping gait, as a pure state machine (no rclpy).

There is no CoM/ZMP/balance controller anywhere in this codebase (see
drcsim's CLAUDE.md) -- AtlasPlugin is a plain per-joint PID with no
velocity/rate limiter, so a big instantaneous position jump just yanks the
joint as hard as its effort limit allows. This module never snaps: every
gait phase is a target pose, linearly interpolated to from the previous
phase's end pose over that phase's duration, sampled once per publish tick
by GaitController.sample(dt).

GaitController's "neutral" reference pose is whatever Atlas's real joint
positions were (from atlas/joint_states) at the moment it was constructed
-- NOT a separately-designed standing pose. An earlier version of this
file hardcoded VRCPlugin::AtlasCommandController::SetPIDStand()'s pose
(a deep crouch) as that reference instead, reasoning it was "real,
already-tuned." It was real and tuned, but only ever for a *pinned*
robot -- every call site in VRCPlugin either re-pins immediately
("pid_stand" mode) or is already mid-vehicle-entry/exit, base rigidly
held in place either way. Nothing had ever proven it stable for a fully
free-standing robot, and it wasn't: interpolating into it fell over,
confirmed interactively. Atlas's real starting pose -- whatever
atlas.launch.py's default startup leaves it holding -- is the only pose
actually proven stable free-standing this whole session, so that's what
every gait phase's lean/lift/swing deltas are now computed relative to,
and the controller never asks Atlas to move away from it before a walk
is actually requested.

The joint-space "walking" mechanism here: to move a foot forward, the
swing leg's hip is flexed forward (SWING_HPY) while the leg is lifted
clear of the ground; to actually propel the pelvis forward, the *support*
leg's hip angle -- left forward-flexed by its own previous swing phase --
is what gets driven back toward neutral while planted, walking-stick-like,
rotating the pelvis forward over the fixed foot. This falls directly out
of composing each phase as (support leg, swing leg, lifted?, forward?)
relative to the neutral pose -- see GaitController._target_pose_for() and
the WALK_CYCLE sequence below.

All lean/lift/swing magnitudes and phase durations are starting estimates
from Atlas v5's real leg geometry (see the module-level plan this was
built from), not guaranteed-correct values -- expect to retune these
against what atlas_walking_demo's milestone checks actually show; no
rebuild needed to do so, this is plain Python.
"""

# The exact 30-joint order AtlasPlugin::Configure() builds internally --
# same list (and the same cross-reference comment) as drcsim_tutorials'
# atlas_teleop.py/joint_command_gui.py use for the same reason: this is
# how AtlasCommand's position array is matched to joints, by index.
ATLAS_JOINT_NAMES = [
    'back_bkz', 'back_bky', 'back_bkx', 'neck_ry',
    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
    'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx',
    'l_arm_wry2',
    'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx',
    'r_arm_wry2',
]

# Starting-estimate tuning constants (radians, seconds) -- see the module
# docstring. Expect to adjust these against the milestone checks in
# atlas_walking_demo's README/plan before a full walk cycle looks right.
LEAN_HPX = 0.12
LEAN_AKX = 0.12
LIFT_KNY = 0.30
LIFT_AKY = -0.15
SWING_HPY = -0.25

SHIFT_DURATION = 0.8
LIFT_DURATION = 0.5
SWING_DURATION = 0.5
PLANT_DURATION = 0.5

# How long returning to the neutral pose takes after a stop request --
# normally a small gap (already near-neutral at a SHIFT boundary), kept
# slow/deliberate anyway rather than snapping.
IDLE_DURATION = 3.0


class GaitPhase:
    """One phase of the walk cycle: which leg supports/swings, and how."""

    def __init__(self, name, support, swing, lifted, forward, duration):
        assert support in ('l', 'r') and swing in ('l', 'r') and support != swing
        self.name = name
        self.support = support
        self.swing = swing
        self.lifted = lifted
        self.forward = forward
        self.duration = duration


# One full stride: shift weight onto the support leg, lift+swing+plant the
# other leg forward, then mirror for the other side. Repeats indefinitely
# while walking; see GaitController._advance() for how/where a stop
# request is honored (only at a SHIFT_* boundary -- both feet grounded,
# weight already recentering).
WALK_CYCLE = [
    GaitPhase('SHIFT_LEFT', support='l', swing='r', lifted=False, forward=False,
              duration=SHIFT_DURATION),
    GaitPhase('LIFT_RIGHT', support='l', swing='r', lifted=True, forward=False,
              duration=LIFT_DURATION),
    GaitPhase('SWING_RIGHT', support='l', swing='r', lifted=True, forward=True,
              duration=SWING_DURATION),
    GaitPhase('PLANT_RIGHT', support='l', swing='r', lifted=False, forward=True,
              duration=PLANT_DURATION),
    GaitPhase('SHIFT_RIGHT', support='r', swing='l', lifted=False, forward=False,
              duration=SHIFT_DURATION),
    GaitPhase('LIFT_LEFT', support='r', swing='l', lifted=True, forward=False,
              duration=LIFT_DURATION),
    GaitPhase('SWING_LEFT', support='r', swing='l', lifted=True, forward=True,
              duration=SWING_DURATION),
    GaitPhase('PLANT_LEFT', support='r', swing='l', lifted=False, forward=True,
              duration=PLANT_DURATION),
]


def _lerp_pose(pose_a, pose_b, alpha):
    return {name: pose_a[name] + (pose_b[name] - pose_a[name]) * alpha
            for name in ATLAS_JOINT_NAMES}


class GaitController:
    """
    Drives an interpolated position target through WALK_CYCLE over time.

    Usage: call start_walking()/stop_walking() from keyboard input, and
    sample(dt) once per publish tick (dt = seconds since the last call) to
    get the current 30-element position list, in ATLAS_JOINT_NAMES order.

    `neutral_pose` must be Atlas's real current joint positions (e.g. from
    the last `atlas/joint_states` message) -- the only pose this
    controller ever assumes is safe to freely stand in, since it's the
    only one actually observed working. The controller starts already
    "at" this pose (no transition needed, nothing to fall over during),
    and every gait phase's target is this same pose plus a lean/lift/
    swing delta -- see the module docstring for why a previous version
    that instead transitioned into a hardcoded pose fell over.
    """

    def __init__(self, neutral_pose):
        self.walking = False
        self._stop_requested = False
        self.neutral_pose = dict(neutral_pose)
        self._phase_index = None  # None means idle (not in WALK_CYCLE)
        self._elapsed = 0.0
        self._prev_pose = dict(self.neutral_pose)
        self._target_pose = dict(self.neutral_pose)

    def start_walking(self):
        self._stop_requested = False
        self.walking = True
        if self._phase_index is None:
            self._begin_phase(0)

    def stop_walking(self):
        self._stop_requested = True

    def sample(self, dt):
        """Advance by dt seconds and return the current 30-value position list."""
        self._elapsed += dt
        duration = (
            WALK_CYCLE[self._phase_index].duration
            if self._phase_index is not None else IDLE_DURATION)
        alpha = min(1.0, self._elapsed / duration)
        pose = _lerp_pose(self._prev_pose, self._target_pose, alpha)
        if alpha >= 1.0:
            self._advance()
        return [pose[name] for name in ATLAS_JOINT_NAMES]

    def _target_pose_for(self, phase):
        """
        Return the full 30-joint target pose {name: value} for one GaitPhase.

        self.neutral_pose plus: a stance lean (both legs' hpx/akx) toward
        `phase.support`, and, if `phase.lifted`/`phase.forward`, knee/ankle
        lift and/or hip-forward-swing deltas on `phase.swing`'s leg only.
        The support leg's own hpy is deliberately left untouched here --
        it either stays at the neutral baseline, or (if this same leg was
        the swing leg forward-flexed by the *previous* phase) interpolates
        back down to that baseline as this phase is entered, which is
        what actually propels the pelvis forward over a planted foot. See
        the module docstring.
        """
        pose = dict(self.neutral_pose)
        lean_sign = 1.0 if phase.support == 'l' else -1.0
        pose['l_leg_hpx'] += lean_sign * LEAN_HPX
        pose['r_leg_hpx'] += -lean_sign * LEAN_HPX
        pose['l_leg_akx'] += -lean_sign * LEAN_AKX
        pose['r_leg_akx'] += lean_sign * LEAN_AKX

        if phase.lifted:
            pose[f'{phase.swing}_leg_kny'] += LIFT_KNY
            pose[f'{phase.swing}_leg_aky'] += LIFT_AKY
        if phase.forward:
            pose[f'{phase.swing}_leg_hpy'] += SWING_HPY

        return pose

    def _begin_phase(self, index):
        self._phase_index = index
        self._elapsed = 0.0
        self._prev_pose = dict(self._target_pose)
        self._target_pose = self._target_pose_for(WALK_CYCLE[index])

    def _begin_idle(self):
        self._phase_index = None
        self._elapsed = 0.0
        self._prev_pose = dict(self._target_pose)
        self._target_pose = dict(self.neutral_pose)
        self.walking = False

    def _advance(self):
        if self._phase_index is None:
            return  # already idle and settled; nothing to advance
        finished_name = WALK_CYCLE[self._phase_index].name
        if self._stop_requested and finished_name.startswith('SHIFT'):
            self._begin_idle()
            return
        self._begin_phase((self._phase_index + 1) % len(WALK_CYCLE))
