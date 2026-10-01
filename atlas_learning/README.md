# atlas_learning

A learned push-recovery policy for Atlas: trained with PPO in MuJoCo, run in Gazebo.

| File | |
|---|---|
| `scripts/build_mjcf.py` | MuJoCo model from the URDF Gazebo spawns |
| `scripts/atlas_env.py` | Gymnasium environment: stand, survive random pushes, domain randomisation |
| `scripts/train_push.py` | PPO training (Stable-Baselines3); `INIT_FROM=<dir>` fine-tunes |
| `scripts/evaluate_mj.py` | fixed-push scoring in MuJoCo |
| `scripts/policy_io.py` | observation/action conventions shared by training and Gazebo |
| `scripts/policy_stand.py` | ROS node: runs a policy on Atlas in Gazebo |
| `models/push_v1`, `models/push_v2` | trained policies (`push_v2` is the default; v1 doesn't transfer) |

```bash
docker/sim.sh sync_max_per_window:=5.0 sync_max_per_step:=0.05
ros2 run atlas_learning policy_stand.py [--ros-args -p policy:=push_v2 -p debug:=true]
```

Full guide: [docs/tutorials/04_learned_push_recovery.md](../docs/tutorials/04_learned_push_recovery.md).
