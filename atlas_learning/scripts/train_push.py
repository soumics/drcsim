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
Train the push-recovery policy with PPO.

usage: train_push.py MODEL_XML OUT_DIR TOTAL_STEPS [N_ENVS]
MODEL_XML from build_mjcf.py. Writes OUT_DIR/policy.zip and vecnormalize.pkl
(the pair policy_stand.py loads) plus checkpoints and TensorBoard logs.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from atlas_env import AtlasPushEnv  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import CheckpointCallback  # noqa: E402
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecNormalize  # noqa: E402,E501
import torch  # noqa: E402

model_xml, out, total = sys.argv[1], sys.argv[2], int(float(sys.argv[3]))
n_envs = int(sys.argv[4]) if len(sys.argv) > 4 else 16


def make(i):
    return lambda: AtlasPushEnv(model_xml, seed=1000 + i)


if __name__ == '__main__':
    os.makedirs(out, exist_ok=True)
    env = VecMonitor(SubprocVecEnv([make(i) for i in range(n_envs)]))
    init = os.environ.get('INIT_FROM')  # fine-tune: directory with policy.zip + vecnormalize.pkl
    if init:
        env = VecNormalize.load(os.path.join(init, 'vecnormalize.pkl'), env)
        env.training = True
    else:
        env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)
    if init:
        model = PPO.load(os.path.join(init, 'policy.zip'), env=env, device='cpu',
                         tensorboard_log=os.path.join(out, 'tb'))
    else:
        model = PPO('MlpPolicy', env, n_steps=512, batch_size=4096, n_epochs=5,
                    learning_rate=3e-4, gamma=0.99, gae_lambda=0.95, clip_range=0.2,
                    ent_coef=0.0, policy_kwargs={'net_arch': [256, 256], 'log_std_init': -1.0,
                                                 'activation_fn': torch.nn.ELU},
                    device='cpu', verbose=1, tensorboard_log=os.path.join(out, 'tb'))
    model.learn(total, callback=CheckpointCallback(2_000_000 // n_envs, out, 'ppo'),
                progress_bar=False)
    model.save(os.path.join(out, 'policy'))
    env.save(os.path.join(out, 'vecnormalize.pkl'))
    print('saved', out)
