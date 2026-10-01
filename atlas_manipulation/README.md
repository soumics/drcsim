# atlas_manipulation

Manipulation with Atlas under whole-body torque control.

`pick_place.py` picks a 10 kg box off one table with both hands, walks it to another
table (stepping back, turning, walking on) and sets it down. Atlas balances on its own
throughout.

```bash
ros2 launch drcsim_gazebo atlas.launch.py sync_max_per_window:=5.0 sync_max_per_step:=0.05
ros2 run atlas_manipulation pick_place.py      # -p box_mass:=10.0 -p debug:=true
```

It builds on `atlas_walking_demo`:
- `wbc.py`: the QP, with palm tasks, squeeze, payload and joint limits;
- `torque_balance.py`: balance, and walking via `qp_walk.py`.

Needs SVH hands (the launch default), Pinocchio and ProxQP.

Full guide: [docs/tutorials/05_pick_and_place.md](../docs/tutorials/05_pick_and_place.md).
