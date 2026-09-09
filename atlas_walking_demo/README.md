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
transient pass. It does **not** move Atlas into any different standing
pose first: an earlier version tried that (a separately-designed "ideal"
standing pose) and Atlas fell over, because that pose had only ever been
used while pinned in the original code, never proven for free standing.
A version after that trusted the very first `atlas/joint_states` message
unconditionally, and that fell too -- consistent with grabbing a snapshot
mid-transient. A version after *that* tried waiting for readings to stop
changing at all, and that never finished waiting -- Atlas's standing pose
has a small persistent oscillation by design (already known, harmless),
so "wait until it's perfectly still" can wait forever. You should see two
log lines: one when the first `atlas/joint_states` message arrives, and
`Atlas has settled into a real, stable starting pose -- ready. Press w to
walk.` (with the captured leg joint values) five seconds after that.
Nothing should move at all until you press `w`.

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
