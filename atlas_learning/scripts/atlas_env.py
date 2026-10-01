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
Gymnasium environment: Atlas standing balance with random pushes (MuJoCo).

Policy at 50 Hz; physics at 1 kHz with AtlasPlugin's PD law in position
actuators (build_mjcf.py). Action: offsets (rad, scaled) to the leg and back
joint targets around the crouched stance; arms and neck hold their rest pose.
Observation (all available in Gazebo from AtlasState): gravity direction and
angular velocity in the pelvis frame, joint angles minus the stance and joint
velocities for the controlled joints, the previous action.
"""
import math
import os
import sys

from ament_index_python.packages import get_package_prefix

sys.path.insert(0, os.path.join(get_package_prefix('atlas_walking_demo'), 'lib',
                                'atlas_walking_demo'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gymnasium as gym  # noqa: E402
import harness_gait as hg  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import policy_io as pio  # noqa: E402
import zmp_walk as zw  # noqa: E402

N = pio.ATLAS_JOINT_NAMES
CONTROLLED = pio.CONTROLLED
ACTION_SCALE = pio.ACTION_SCALE
SUBSTEPS = 20            # 1 kHz physics per 50 Hz policy step
EPISODE_STEPS = 500      # 10 s


def stance():
    """Return the crouched stance: 30 joint angles in AtlasCommand order."""
    angles, _, _ = zw.ZmpWalker().sample()
    q = np.zeros(len(N))
    for name, a in angles.items():
        q[N.index(name)] = a
    for name, a in hg.ARM_REST.items():
        q[N.index(name)] = a
    return q


class AtlasPushEnv(gym.Env):
    """Stand; survive random pushes on the torso."""

    def __init__(self, model_path, push_max=650.0, seed=None, wide_randomization=True):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in N])
        self.vadr = np.array([m.jnt_dofadr[m.joint(n).id] for n in N])
        self.ctrl_idx = np.array([N.index(n) for n in CONTROLLED])
        self.act_id = np.array([m.actuator(n).id for n in N])
        self.torso = m.body('utorso').id
        self.pelvis = m.body('pelvis').id
        self.feet = [m.body('l_foot').id, m.body('r_foot').id]
        self.nominal = stance()
        self.push_max = push_max
        self.base = {'friction': m.geom_friction.copy(), 'mass': m.body_mass.copy(),
                     'ipos': m.body_ipos.copy(), 'damping': m.dof_damping.copy(),
                     'solref': m.geom_solref.copy(),
                     'kp': m.actuator_gainprm[:, 0].copy(),
                     'bias1': m.actuator_biasprm[:, 1].copy(),
                     'bias2': m.actuator_biasprm[:, 2].copy()}
        nc = len(CONTROLLED)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (nc,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (pio.OBS_SIZE,), np.float32)
        self.rng = np.random.default_rng(seed)
        self.wide = wide_randomization

    # --- helpers --------------------------------------------------------

    def _randomize(self):
        """
        Domain randomisation.

        v1 (narrow: friction, mass, gains) balanced in MuJoCo but drifted
        over in Gazebo; v2 widens it so the policy can't lean on MuJoCo's
        particular contact and actuator response.
        """
        m, r = self.model, self.rng
        wide = self.wide
        m.geom_friction[:, 0] = self.base['friction'][:, 0] * r.uniform(
            0.4 if wide else 0.6, 1.2)
        spread = 0.10 if wide else 0.05
        m.body_mass[:] = self.base['mass'] * r.uniform(1 - spread, 1 + spread,
                                                       len(self.base['mass']))
        g = 0.2 if wide else 0.1
        gain = r.uniform(1 - g, 1 + g, m.nu)
        m.actuator_gainprm[:, 0] = self.base['kp'] * gain
        m.actuator_biasprm[:, 1] = self.base['bias1'] * gain
        m.actuator_biasprm[:, 2] = self.base['bias2'] * r.uniform(
            0.5 if wide else 0.8, 1.5 if wide else 1.2, m.nu)
        if wide:
            m.body_ipos[:] = self.base['ipos']
            m.body_ipos[self.torso] += r.uniform(-0.03, 0.03, 3)
            m.dof_damping[:] = self.base['damping'] + r.uniform(0.0, 2.0, m.nv)
            m.geom_solref[:, 0] = r.uniform(0.002, 0.02)  # contact stiffness
        self.latency = r.integers(0, 3)  # policy steps of action delay (0-40 ms)
        # A small steady push, as if leaning on something (wide only).
        self.drift_force = r.normal(0, 15.0, 2) if wide else np.zeros(2)

    def _tilt(self):
        R = self.data.xmat[self.pelvis].reshape(3, 3)
        return math.acos(max(-1.0, min(1.0, R[2, 2])))

    def _obs(self):
        """Return the observation (policy_io.observation) with sensor noise."""
        d = self.data
        obs = pio.observation(d.qpos[3:7], d.qvel[3:6], d.qpos[self.qadr], d.qvel[self.vadr],
                              self.nominal, self.last_action)
        nc = len(CONTROLLED)
        sigma = np.concatenate([np.full(3, 0.02), np.full(3, 0.05), np.full(nc, 0.005),
                                np.full(nc, 0.005), np.zeros(nc)])
        return (obs + self.rng.normal(0, 1, obs.size) * sigma).astype(np.float32)

    def _schedule_pushes(self):
        self.pushes = []
        t = self.rng.uniform(1.0, 2.5)
        while t < EPISODE_STEPS * SUBSTEPS * self.model.opt.timestep - 1.5:
            angle = self.rng.uniform(0, 2 * math.pi)
            mag = self.rng.uniform(0.0, self.push_max)
            dur = self.rng.uniform(0.1, 0.25)
            self.pushes.append((t, t + dur, mag * math.cos(angle), mag * math.sin(angle)))
            t += dur + self.rng.uniform(2.0, 4.0)

    # --- gym API ---------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        m, d = self.model, self.data
        self._randomize()
        mujoco.mj_resetData(m, d)
        q = self.nominal + self.rng.normal(0, 0.02, len(N))
        d.qpos[self.qadr] = q
        if self.wide:  # start tilted up to ~3 deg
            tilt = self.rng.normal(0, 0.025, 2)
            d.qpos[3:7] = [1.0, tilt[0] / 2, tilt[1] / 2, 0.0]
            d.qpos[3:7] /= np.linalg.norm(d.qpos[3:7])
        d.ctrl[self.act_id] = q
        mujoco.mj_forward(m, d)
        d.qpos[2] -= min(d.xpos[f][2] for f in self.feet) - 0.0745
        mujoco.mj_forward(m, d)
        self.steps = 0
        self.last_action = np.zeros(len(CONTROLLED), np.float32)
        self.queue = [np.zeros(len(CONTROLLED))] * 3
        self._schedule_pushes()
        return self._obs(), {}

    def step(self, action):
        m, d = self.model, self.data
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self.queue = (self.queue + [action])[-4:]
        applied = self.queue[-1 - self.latency]  # simulated sensing/command latency
        d.ctrl[self.act_id] = pio.joint_targets(applied, self.nominal)
        t = d.time
        d.xfrc_applied[self.torso, :3] = 0.0
        d.xfrc_applied[self.torso, :2] = self.drift_force
        for t0, t1, fx, fy in self.pushes:
            if t0 <= t < t1:
                d.xfrc_applied[self.torso, :2] = (fx, fy)
        mujoco.mj_step(m, d, nstep=SUBSTEPS)
        d.xfrc_applied[self.torso, :3] = 0.0
        self.steps += 1

        tilt = self._tilt()
        height = d.xpos[self.pelvis][2]
        fell = tilt > 0.8 or height < 0.55
        # Reward: stay up and upright, calmly, near the stance.
        torque = d.actuator_force[self.act_id][self.ctrl_idx]
        rate = action - self.last_action
        reward = (1.0
                  + 1.0 * math.exp(-(tilt / 0.2) ** 2)
                  + 0.5 * math.exp(-((height - 0.89) / 0.05) ** 2)
                  - 2e-6 * float(torque @ torque)
                  - 0.05 * float(rate @ rate)
                  - 0.02 * float(action @ action))
        if fell:
            reward -= 50.0
        self.last_action = action.astype(np.float32)
        truncated = self.steps >= EPISODE_STEPS
        return self._obs(), reward, fell, truncated, {}
