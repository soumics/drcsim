# atlas_walking_demo

Statically-stable, keyboard-controlled stepping for Atlas. New tutorial
content built on top of the ported `drcsim` packages — not a port of
anything from the original repo, and not a real dynamic/balance-controlled
walk: there is no CoM/ZMP controller anywhere in this codebase (see
`src/drcsim/CLAUDE.md`). `gait_controller.py`'s tuning constants (lean
angle, step height/length, phase durations) are starting estimates from
Atlas v5's real leg geometry, not guaranteed-correct — expect to retune
them against what you actually see, no rebuild needed since it's plain
Python.

## Run it

Atlas must already be standing normally first:

```bash
ros2 launch drcsim_gazebo atlas.launch.py
```

Then, in a second terminal:

```bash
ros2 run atlas_walking_demo walk_keyboard.py
```

On startup this node reads `atlas/atlas_state` once and takes over
`AtlasPlugin`'s *current PID setpoint* -- reconstructed as
`position + effort / kp`, all zeros by default -- as the neutral stance
every gait phase leans/lifts/swings away from. It keeps `k_effort=255`
(what `AtlasPlugin` already uses), so taking over is bumpless and nothing
moves until you press `w`. Why the setpoint and not the measured pose:
these gains have no integral term, so each joint's small gravity sag below
its setpoint is exactly the error that produces the torque holding Atlas
up; commanding the measured (sagged) pose zeroes that torque and Atlas
collapses. (Several earlier versions did exactly that -- see
`src/drcsim/CLAUDE.md`, bug #8.) You should see one line, `Took over the
current PID setpoint ... -- ready.`, followed by a `[diag]` line each
second.

Keys: `w` = start/continue walking forward, `space`/`s` = stop (finishes
the current step, then stands centered), `q`/Ctrl-C = quit. Turning isn't
implemented yet.

### Harness mode (default)

Like a real lab gantry, the node first puts Atlas in a harness: it
publishes `pinned_with_gravity` on `atlas/mode`, so `VRCPlugin` holds the
pelvis at its current pose while gravity still acts on the limbs. With
balance taken care of, `gait_controller.HARNESS_CYCLE` runs a human-like
swing/stance cycle:

- lift the knee;
- reach the thigh forward;
- plant the heel ahead of the hip;
- sweep the stance leg back and push off.

All the while the node publishes `atlas/cmd_vel`
(`forward_speed`, default 0.35 m/s, matched to the stance foot's sweep) so
the held pelvis actually travels forward. Stopping finishes the current
step with both feet down, then stops the pelvis.

```bash
ros2 run atlas_walking_demo walk_keyboard.py                          # harness
ros2 run atlas_walking_demo walk_keyboard.py --ros-args -p forward_speed:=0.2
ros2 topic pub --once atlas/mode std_msgs/msg/String "{data: nominal}"   # release
```

`-p harness:=false` runs the free-standing lean/lift/plant `WALK_CYCLE`
below instead. Without a balance controller it only manages a step or two
before falling (see `src/drcsim/CLAUDE.md`).

## Tuning, one milestone at a time

`atlas/debug/{l,r}_foot_contact` (`geometry_msgs/WrenchStamped`, already
published by `AtlasPlugin` whenever cheats are enabled — on by default in
`atlas.launch.py`) gives a numeric, not just visual, way to check each
step of the gait is doing what it should:

```bash
ros2 topic echo /atlas/debug/l_foot_contact
```

1. **Lean only** — comment out everything in `WALK_CYCLE` after
   `SHIFT_LEFT` (or just watch the first ~`SHIFT_DURATION` seconds after
   pressing `w`) and confirm: Atlas leans without falling, `l_foot_contact`
   force rises, `r_foot_contact` force drops.
2. **Lift while shifted** — let it continue into `LIFT_RIGHT`; confirm
   `r_foot_contact` drops to ~zero while still balanced on the left foot.
3. **Swing and plant** — let it continue through `SWING_RIGHT`/
   `PLANT_RIGHT`; confirm the right foot lands forward of its start
   position without toppling, and `r_foot_contact` force reappears.
4. **Full cycle** — hold `w` through multiple strides; confirm forward
   progress and that `space` cleanly returns Atlas to a centered stand.

If a milestone doesn't look right, the relevant constants are all at the
top of `gait_controller.py` (`LEAN_HPX`/`LEAN_AKX`/`LIFT_KNY`/`LIFT_AKY`/
`SWING_HPY` and the four `*_DURATION` values) — adjust and `ros2 run`
again.
