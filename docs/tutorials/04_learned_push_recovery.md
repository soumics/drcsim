# 4. Learned push recovery

A PPO policy is trained in **MuJoCo** and run unchanged in **Gazebo**. The shipped policy
`push_v2` survives:
- 90 N·s pushes in every direction;
- 120–150 N·s forward and backward.

## Run the shipped policy

```bash
docker/sim.sh sync_max_per_window:=5.0 sync_max_per_step:=0.05
docker/shell.sh
ros2 run atlas_learning policy_stand.py          # policy:=push_v2 by default
drcsim_push 600 0 0.2                            # 120 N s from behind
```

Atlas crouches in position mode first; the node then logs `the learned policy has control`.

## Install the training tools (once per container)

```bash
docker exec -it drcsim /entrypoint.sh drcsim_install_learning   # PyTorch (CUDA 12.8), MuJoCo, SB3
```

## Train

Run these inside the container (`docker/shell.sh`), from `atlas_learning/scripts`:

```bash
python3 build_mjcf.py /root/learn/atlas.xml          # MuJoCo model from the Gazebo URDF
python3 train_push.py /root/learn/atlas.xml /root/learn/my_policy 30e6 16
tensorboard --logdir /root/learn/my_policy/tb        # optional
python3 evaluate_mj.py /root/learn/atlas.xml /root/learn/my_policy 20
ros2 run atlas_learning policy_stand.py --ros-args -p policy:=/root/learn/my_policy
```

To fine-tune from an existing policy, set `INIT_FROM=<policy dir>` before
`train_push.py`. That is how v2 was trained from v1.

## Design

| | |
|---|---|
| Observation (51) | gravity in the pelvis frame, pelvis angular velocity, leg and back joint angles relative to the stance, their velocities, the previous action (`policy_io.py`, shared by training and Gazebo) |
| Action (15) | leg and back joint target offsets, ±0.4 rad around the crouched stance; arms and neck hold |
| Rate | 50 Hz policy, 1 kHz PD (AtlasPlugin's gains) |
| Reward | stay up, upright, at standing height; small torque, action and action-rate penalties |
| Pushes | random direction, 0–650 N for 0.1–0.25 s, every 2–4 s |

**Sim-to-sim lessons:**
- **Self-collision off.** The URDF's torso and hip shapes overlap by up to 5 cm and
  pushed Atlas over in MuJoCo.
- **Stiff floor** (`solref` 0.005). The default was soft: the feet sank 7 mm and the heel
  rolled.
- **The PD law must match exactly.** `AtlasPlugin`'s kd acts on d(error)/dt, so every
  50 Hz target change gave a one-tick torque spike. `policy_stand.py` sends
  `kd_position = 0` and `effort = −kd·q̇` instead.
- **`push_v1`** used narrow randomisation. It was excellent in MuJoCo and fell over in
  Gazebo within 3 s.
- **`push_v2`** was fine-tuned with wide randomisation and transfers: contact stiffness,
  gains, damping, a shifted torso CoM, a steady force and the starting tilt.
