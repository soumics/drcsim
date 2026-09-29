# atlas_walking_demo

Keyboard teleoperation for Atlas. It walks in every direction, turns, grasps
with its five-finger SCHUNK SVH hands (or Sandia hands) and can be kicked. The
hands rest in a natural relaxed curl while standing and walking. This is new tutorial content built
on top of the ported `drcsim` packages, not a port of anything from the
original repo.

Walking runs in a **harness**, like a lab gantry: `VRCPlugin` holds the
pelvis with a stiff spring-damper, and the legs carry the weight. There is
no free-standing balance controller in this codebase yet (see
`src/drcsim/CLAUDE.md`).

## Run it

```bash
ros2 launch drcsim_gazebo atlas.launch.py          # Atlas with Sandia hands
ros2 run atlas_walking_demo walk_keyboard.py        # second terminal
```

The node takes over Atlas's current PID setpoint (bumpless), puts it in the
harness and crouches 2.5 cm, lowering the arms from the zero T-pose. This
takes 3 s. The setpoint is reconstructed as `position + effort / kp`,
because commanding the measured, gravity-sagged pose drops the holding
torque (see `CLAUDE.md`, bug #8).

| Keys | Action |
|---|---|
| `w` / `s` | walk forward / backward |
| `a` / `d` | side-step left / right |
| `q` / `e` | turn in place left / right |
| `z` / `c` | walk forward curving left / right |
| `space` / `x` | stop (finishes the step, then stands) |
| `+` / `-` | speed 25–100% (applies to the next walking key) |
| `g`, `[` / `]` | grip / relax both hands; left / right hand only |
| `h` | open both hands flat (`g` relaxes them again) |
| `k` / `l` / `j` | kick Atlas from the right / front / behind (900 N, 0.2 s) |
| Ctrl-C | quit (the harness stays on) |

Any `geometry_msgs/Twist` on `atlas_walk/cmd_vel` also drives it, for
example a joystick or `teleop_twist_keyboard`. Twist takes priority while
messages keep arriving and stops Atlas 0.5 s after they stop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=atlas_walk/cmd_vel
ros2 topic pub --once atlas/mode std_msgs/msg/String "{data: nominal}"   # release harness
```

You can also push Atlas with the mouse: Gazebo's **Apply Force/Torque** GUI
plugin works, because `atlas.world` loads the `ApplyLinkWrench` system.

## How the gait works (`harness_gait.py`)

The gait is driven by a body velocity (vx, vy, turn rate). Every tick it
plans where each ankle goes relative to the pelvis and solves the full leg
IK (hip yaw, roll, pitch, knee, and ankle pitch and roll), keeping the soles
level:

- **Stance:** the ankle moves exactly opposite to the pelvis, including
  rotation, so the planted foot stays still.
- **Swing:** the foot lifts off at ground speed, arcs 8 cm high and lands
  half a step ahead in the direction of travel.
- **Knees:** bent about 30° in stance. Each foot is down 60% of the stride,
  and each arm swings with the opposite leg.
- **Velocity changes:** rate-limited. Walking and turning at once is scaled
  so the outer foot never outreaches the leg.
- **Stopping:** ends with one stride in place, bringing the feet home.

Measured in the simulator (30 Hz commands, sim time):

| Test | Result |
|---|---|
| Forward 0.30 m/s, 6 s | 1.89 m straight |
| Backward 0.25 m/s | 1.61 m |
| Side-step 0.15 m/s | 0.95 m, no forward drift |
| Turn 0.35 rad/s | 127°, turning in place |
| Stride / stance-foot slip (forward) | 0.50 m / ~2.5 cm |
| Pelvis tilt while walking | under 2° |
| Kick from the side (900 N, 0.2 s) | pelvis moves 3.7 cm, back within ~1 s, keeps walking |

`-p harness:=false` runs the older free-standing lean/lift/plant
`WALK_CYCLE` in `gait_controller.py` instead (only `w` and `space`).
Without a balance controller it only manages a step or two before falling.

## Free-standing mode: tuning, one milestone at a time

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
