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

On startup this node waits 5 seconds (`SETTLE_DELAY_SEC` in
`walk_keyboard.py`) after the first `atlas/joint_states` message before
trusting a later one as the neutral reference every gait phase
leans/lifts/swings away from -- letting the post-unpin settling
transient pass.

The very first command then goes through the `atlas/reset_controls`
*service*, not the `atlas/atlas_command` topic. `AtlasPlugin` starts up
with `k_effort=0` for every joint (real user-PID control is *never*
active by default -- Atlas has been standing all session on the
AtlasSimInterface/BDI feedforward path instead), but its PID integral
term accumulates every tick regardless of `k_effort`, so it had likely
been quietly winding up, unapplied, for the entire time Atlas stood. A
version of this node that published straight to the topic as the very
first command (`k_effort=255`, otherwise identical, sane joint values)
made Atlas fall immediately, before any key was even pressed -- almost
certainly that pent-up integral suddenly applying the instant the
user-PID path activated for the first time. `atlas/reset_controls`'s
`reset_pid_controller` flag zeroes that state and applies the new command
atomically in the same call, for a clean handoff.

You should see three log lines: the first `atlas/joint_states` message
arriving, then (5s later) `Atlas has settled into a real, stable
starting pose, and the PID handoff succeeded -- ready. Press w to walk.`
with the full captured pose printed alongside it. Nothing should move at
all until you press `w`.

This pose (and the mechanism for setting it) went through four earlier,
different interactively-caught bugs before this one -- see
`gait_controller.py`'s `GaitController` docstring and `src/drcsim/
CLAUDE.md` for the fuller history if curious.

Keys: `w` = start/continue walking forward, `space`/`s` = stop (finishes
the current step, then stands centered), `q`/Ctrl-C = quit. Turning isn't
implemented yet.

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
