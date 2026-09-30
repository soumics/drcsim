# atlas_walking_demo

Keyboard teleoperation for Atlas. It walks in every direction, turns, grasps
with its five-finger SCHUNK SVH hands (or Sandia hands) and can be kicked. The
hands rest in a natural relaxed curl while standing and walking. This is new tutorial content built
on top of the ported `drcsim` packages, not a port of anything from the
original repo.

Two ways to walk:

- **Harness** (default): like a lab gantry, `VRCPlugin` holds the pelvis
  with a stiff spring-damper while the legs carry the weight. Fast (up to
  0.42 m/s) and can't fall.
- **Free-standing** (`-p harness:=false`): no harness. ZMP preview control
  plus balance feedback, in every direction. Slower (about 0.1 m/s). It
  cannot get up by itself yet after a fall (see below). See
  [Free-standing walking](#free-standing-walking) below.

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
| `r` | free-standing, after a fall: harness reset onto its feet (a testing aid) |
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

## Free-standing walking

No harness: position-controlled ZMP walking. It plans where the centre of
pressure must go, moves the body so it goes there, and corrects from the
IMU and leg kinematics.

```bash
ros2 launch drcsim_gazebo atlas.launch.py           # let Atlas stand (~10 s)
ros2 run atlas_walking_demo walk_keyboard.py --ros-args -p harness:=false
# or scripted: N steps (any mix of forward / sideways / turning), then stand
ros2 run atlas_walking_demo free_walk.py --ros-args -p steps:=10 -p step_length:=0.2
```

It takes over the standing controller, crouches 2.5 cm (3 s) and settles
(2 s); then the same keys (and `atlas_walk/cmd_vel` Twist) walk it. A walk
command takes effect about **2.6 s** later: the preview has to see a
change that far ahead, and starting sooner jerks the CoM (0.8 s gave 5×
the ZMP error, offline). Stopping finishes the planned steps and brings
the feet together.

**Falls.** Past 35° of tilt it stops commanding. Atlas cannot yet get up
by itself the way a person does (roll over, push up, kneel, stand); that
is being built. Until then `r` (or `auto_recover:=true`, 2 s after a fall;
the default for the scripted `free_walk.py`) does a **harness reset**, a
testing aid, not a motion: `VRCPlugin`'s `recover` mode lifts the pelvis
upright 15 cm above spawn height in the spring harness (a teleport), the
legs blend into the stance in the air, the harness lowers until the legs
carry the weight, then lets go and it balances again.

| Layer | What it does | Where |
|---|---|---|
| Footsteps | planned online from the commanded (vx, vy, wz), far enough ahead for the preview; side steps and turns taken by the leading foot so the feet never cross | `zmp_walk.ZmpWalker` |
| ZMP reference | on the stance foot (4 cm inside the ankle) in single support, ramping across in double support | `ZmpWalker._plan_step` |
| Preview control | Kajita 2003: LIPM, CoM 1.12 m, 1.6 s preview, 200 Hz | `ZmpWalker._step_com` |
| Leg IK | pelvis = CoM − body offset, heading between the feet; `harness_gait.leg_ik_3d` | `ZmpWalker.sample` |
| Gravity feedforward | model-based joint torques for each leg's planned load at its planned CoP | `zmp_walk.gravity_feedforward` |
| Ankle / hip stabilizer | `aky`, `akx`, `hpx` from IMU pitch/roll on the loaded legs | `balance_controller.py` |
| CoM feedback | measured CoM (leg FK + IMU) vs plan, blended by load; shifts the pelvis target | `FreeWalkController._com_feedback` |
| Reset | fall detection, harness reset onto the feet | `FreeWalkController.recover` |

Measured (Docker, sim time, `free_walk.py`, 10 steps each, all finished
standing):

| Walk | Result |
|---|---|
| 0.15 m steps (0.125 m/s) | 1.37 m, repeatable |
| 0.20 m / 0.25 m steps | 1.8 m / 2.3 m |
| 0.15 m steps, faster timing (0.6 s + 0.3 s) | 1.37 m |
| side-stepping 0.03 m/s | 0.21 m sideways |
| turning 0.1 rad/s | 43° in 6 steps |
| curve: 0.1 m steps + 0.03 m/s + 0.08 rad/s | 58°, 0.53 m / 0.67 m |
| 0.30 m steps | falls at the 10th step (so steps are capped at 0.25 m) |

What it took, for anyone tuning further:
- Gravity feedforward must be *model-based*. Feeding back the measured foot
  load made a load → torque → lift oscillation. The ankle term is needed
  too; without it Atlas tipped onto its toes in single support.
- **Lateral CoM feedback is what made it walk** (`com_kp_y` 1.0,
  `com_kd_y` 0.1). Without it a sideways sway grew each step and Atlas fell
  after 2–4 steps; the IMU hip/ankle roll stabilizers alone did not stop
  it (akx has little authority with both feet down, measured). The
  wider 4 cm ZMP inset helped too.
- Harness reset: at spawn height a foot still in its fallen pose dragged on
  the floor and stuck (hip roll held 10° off by friction), and a harness
  lowered too far pressed Atlas into the floor (2140 N on the feet for
  1760 N of weight); either way it fell again on release.

Parameters (both nodes): `stabilizer_kp`/`kd` (ankle pitch),
`stabilizer_kp_roll`/`kd_roll`, `hip_roll_kp`/`kd`, `com_kp`/`com_kd`
(fore-aft), `com_kp_y`/`com_kd_y` (lateral) for `free_walk.py`;
`auto_recover` for both; `free_walk.py` also takes `steps`,
`step_length`, `side_speed`, `turn_speed`, `single_support`,
`double_support`, `zmp_y_inset`.

The original keyframe gait (`gait_controller.py`'s lean/lift/plant
`WALK_CYCLE`, no balance control) is no longer used by the nodes; it
fell within a step or two free-standing.
