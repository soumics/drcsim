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
Run a learned balance policy (trained in MuJoCo) on Atlas in Gazebo.

usage: ros2 run atlas_learning policy_stand.py --ros-args -p policy:=push_v1
Takes over and crouches in position mode (atlas_walking_demo's
balance_controller), then hands the legs and back to the policy at 50 Hz of
sim time. The PD law matches training exactly: AtlasPlugin gets the policy's
joint targets with kp from atlas_v5_gains.yaml (+ walking overrides),
kd_position = 0, and -kd * qdot as effort every AtlasState (1 kHz) -- with
AtlasPlugin's own kd, which acts on d(error)/dt, every 50 Hz target change
became a one-tick torque spike that training never saw. Requests lockstep
(desired_controller_period_ms) so the policy runs on time; start the sim with
sync_max_per_window:=5.0.
"""

import math
import os
import pickle
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, os.path.join(get_package_prefix('atlas_walking_demo'), 'lib',
                                'atlas_walking_demo'))

from atlas_msgs.msg import AtlasCommand, AtlasState  # noqa: E402
import balance_controller as bc  # noqa: E402
import harness_gait as hg  # noqa: E402
import numpy as np  # noqa: E402
import policy_io as pio  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
import yaml  # noqa: E402
import zmp_walk as zw  # noqa: E402

FALL_TILT = math.radians(45)


def load_gains():
    """Return (kp, kd) arrays as in training: atlas_v5_gains.yaml + walking overrides."""
    path = os.path.join(get_package_share_directory('drcsim_gazebo'), 'config',
                        'atlas_v5_gains.yaml')
    gains = yaml.safe_load(open(path))['atlas_plugin']['ros__parameters']
    overrides = {'hpz': (1000.0, 10.0), 'hpx': (2500.0, 10.0), 'akx': (1000.0, 3.0)}
    kp, kd = [], []
    for name in pio.ATLAS_JOINT_NAMES:
        p = gains[f'atlas_controller.gains.{name}.p']
        d = gains[f'atlas_controller.gains.{name}.d']
        joint = name.rsplit('_', 1)[1]
        if '_leg_' in name and joint in overrides:
            p, d = overrides[joint]
        kp.append(p)
        kd.append(d)
    return np.array(kp), np.array(kd)


class Policy:
    """A trained PPO actor (Stable-Baselines3) with its observation normalisation."""

    def __init__(self, directory):
        from stable_baselines3 import PPO
        self.model = PPO.load(os.path.join(directory, 'policy.zip'), device='cpu')
        with open(os.path.join(directory, 'vecnormalize.pkl'), 'rb') as f:
            norm = pickle.load(f)
        self.mean = norm.obs_rms.mean
        self.std = np.sqrt(norm.obs_rms.var + norm.epsilon)
        self.clip = norm.clip_obs

    def __call__(self, obs):
        obs = np.clip((obs - self.mean) / self.std, -self.clip, self.clip)
        action, _ = self.model.predict(obs.astype(np.float32), deterministic=True)
        return np.clip(action, -1.0, 1.0)


class PolicyStandNode(Node):
    """Crouch in position mode, then balance with the learned policy."""

    def __init__(self):
        super().__init__('atlas_policy_stand',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        name = self.declare_parameter('policy', 'push_v1').value
        directory = name if os.path.isdir(name) else os.path.join(
            get_package_share_directory('atlas_learning'), 'models', name)
        self.policy = Policy(directory)
        self.sync_ms = self.declare_parameter('sync_period_ms', 2).value
        self.debug = self.declare_parameter('debug', False).value
        self.last_debug = -1.0
        self.kp, self.kd = load_gains()
        angles, _, _ = zw.ZmpWalker().sample()
        self.stance = np.zeros(len(pio.ATLAS_JOINT_NAMES))
        for joint, a in angles.items():
            self.stance[pio.ATLAS_JOINT_NAMES.index(joint)] = a
        for joint, a in hg.ARM_REST.items():
            self.stance[pio.ATLAS_JOINT_NAMES.index(joint)] = a
        self.position = None
        self.active = False
        self.fallen = False
        self.last_action = np.zeros(len(pio.CONTROLLED), np.float32)
        self.targets = self.stance.copy()
        self.next_policy_t = 0.0
        self.last_tick = -1.0
        self.last_log = -1.0
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 1)
        self.get_logger().info(f'Policy loaded from {directory}.')

    def _on_state(self, s):
        if len(s.position) != len(pio.ATLAS_JOINT_NAMES) or self.fallen:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        if self.position is None:
            self.position = bc.FreeWalkController(s)
            self.get_logger().info('Took over; crouching in position mode.')
        o = s.orientation
        quat = (o.w, o.x, o.y, o.z)
        tilt = math.acos(max(-1.0, min(1.0, 1 - 2 * (o.x ** 2 + o.y ** 2))))
        if tilt > FALL_TILT:
            self.fallen = True
            self.get_logger().error(f'Fell (tilt {math.degrees(tilt):.0f} deg); stopped.')
            return
        cmd = AtlasCommand()
        cmd.header.stamp = s.header.stamp
        cmd.k_effort = [255] * len(pio.ATLAS_JOINT_NAMES)
        if not self.active:
            if t - self.last_tick < 0.005 - 1e-4:  # balance_controller runs at 200 Hz
                return
            self.last_tick = t
            out = self.position.update(s, t)
            cmd.position, cmd.effort = out.position, out.effort
            cmd.kp_position, cmd.kd_position = out.kp, out.kd
            if self.position.status == 'ready':
                self.active = True
                self.next_policy_t = t
                self.get_logger().info('Standing; the learned policy has control.')
        else:
            if t >= self.next_policy_t:
                self.next_policy_t += pio.POLICY_DT
                w = s.angular_velocity
                obs = pio.observation(quat, (w.x, w.y, w.z), s.position, s.velocity,
                                      self.stance, self.last_action)
                action = self.policy(obs)
                if self.debug and t - self.last_debug >= 0.2:
                    self.last_debug = t
                    roll = math.degrees(math.atan2(2 * (o.w * o.x + o.y * o.z),
                                                   1 - 2 * (o.x ** 2 + o.y ** 2)))
                    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (o.w * o.y - o.z * o.x)))))
                    self.get_logger().info(
                        f'dbg roll={roll:+.1f} pitch={pitch:+.1f} g={np.round(obs[:3], 3)} '
                        f'w={np.round(obs[3:6] * 4, 2)} q_err={np.round(obs[6:21], 2)} '
                        f'a={np.round(action, 2)}')
                self.last_action = action.astype(np.float32)
                self.targets = pio.joint_targets(action, self.stance)
            cmd.position = [float(x) for x in self.targets]
            cmd.kp_position = [float(x) for x in self.kp]
            cmd.kd_position = [0.0] * len(pio.ATLAS_JOINT_NAMES)
            cmd.effort = [float(-k * v) for k, v in zip(self.kd, s.velocity)]
            cmd.desired_controller_period_ms = int(self.sync_ms)
        self.pub.publish(cmd)
        if self.active and t - self.last_log >= 1.0:
            self.last_log = t
            self.get_logger().info(f'policy: tilt={math.degrees(tilt):.1f} deg, action '
                                   f'|a|={np.linalg.norm(self.last_action):.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = PolicyStandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
