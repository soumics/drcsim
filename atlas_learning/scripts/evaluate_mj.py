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
Score a trained push policy in MuJoCo with fixed pushes, like the Gazebo tests.

usage: evaluate_mj.py MODEL_XML POLICY_DIR [TRIALS]
For each direction (forward, backward, left, right) and impulse (N s over
0.2 s on the torso, 2 s in), runs TRIALS episodes with domain randomisation
and prints the fraction still standing 5 s after the push.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from atlas_env import AtlasPushEnv  # noqa: E402
import numpy as np  # noqa: E402
from policy_stand import Policy  # noqa: E402

DIRECTIONS = {'forward': (1, 0), 'backward': (-1, 0), 'left': (0, 1), 'right': (0, -1)}
IMPULSES = (60, 90, 120, 150)


def run(env, policy, direction, impulse):
    """Return True if Atlas is still up 5 s after the push."""
    obs, _ = env.reset()
    force = impulse / 0.2
    env.pushes = [(2.0, 2.2, force * direction[0], force * direction[1])]
    for _ in range(350):  # 7 s
        obs, _, fell, _, _ = env.step(policy(obs))
        if fell:
            return False
    return True


if __name__ == '__main__':
    env = AtlasPushEnv(sys.argv[1], seed=7)
    policy = Policy(sys.argv[2])
    trials = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    print('impulse (N s)  ' + '  '.join(f'{d:>8s}' for d in DIRECTIONS))
    for impulse in IMPULSES:
        rates = [np.mean([run(env, policy, v, impulse) for _ in range(trials)])
                 for v in DIRECTIONS.values()]
        print(f'{impulse:13d}  ' + '  '.join(f'{r:8.0%}' for r in rates), flush=True)
