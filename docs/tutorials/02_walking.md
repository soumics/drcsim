# 2. Walking

There are two walking controllers, both in `atlas_walking_demo`. Both send `AtlasCommand`
joint targets (position mode) to `AtlasPlugin`.

| | Harness (default) | Free-standing |
|---|---|---|
| Support | `VRCPlugin` holds the pelvis with a stiff spring-damper; the legs carry the weight | none |
| Gait | IK foot trajectories (`harness_gait.py`) | ZMP preview control + balance feedback (`zmp_walk.py`, `balance_controller.py`) |
| Speed | up to 0.42 m/s | about 0.1 m/s, steps up to 0.25 m |
| Falls | can't fall | can fall; `r` is a harness reset (a testing aid) |

## Keyboard teleop

```bash
docker/sim.sh                     # or: ros2 launch drcsim_gazebo atlas.launch.py
docker/teleop.sh                  # harness
docker/teleop.sh --ros-args -p harness:=false   # free-standing
```

Without Docker: `ros2 run atlas_walking_demo walk_keyboard.py [--ros-args -p harness:=false]`.

| Keys | Action |
|---|---|
| `w` / `s` | forward / backward |
| `a` / `d` | side-step left / right |
| `q` / `e` | turn left / right |
| `z` / `c` | forward, curving left / right |
| `space` / `x` | stop (finishes the step) |
| `+` / `-` | speed 25–100 % |
| `g`, `[`, `]` | grip / relax both hands; left / right only |
| `h` | open the hands flat |
| `k` / `l` / `j` | kick Atlas from the right / front / behind |
| `r` | after a free-standing fall: harness reset |
| Ctrl-C | quit |

In free-standing mode a command takes effect about **2.6 s** after the key, because the
ZMP preview plans that far ahead.

## Driving with a Twist

Any `geometry_msgs/Twist` on `atlas_walk/cmd_vel` drives both modes; Atlas stops 0.5 s
after messages stop. With no terminal attached (for example `docker exec` without `-t`),
the node follows only this topic.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=atlas_walk/cmd_vel
ros2 topic pub -r 10 atlas_walk/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

## Scripted free-standing walk

```bash
ros2 run atlas_walking_demo free_walk.py --ros-args -p steps:=10 -p step_length:=0.2
ros2 run atlas_walking_demo free_walk.py --ros-args -p steps:=6 -p turn_speed:=0.1
```

Parameters: `steps`, `step_length` (≤ 0.25), `side_speed`, `turn_speed`,
`single_support`, `double_support`; the balance gains are listed in the package README.

## How the free-standing walker works

1. **Footsteps** are planned online from the commanded velocity, just far enough ahead
   for the preview. Side steps and turns are taken by the leading foot, so the feet never
   cross.
2. **The ZMP reference** sits on the stance foot (4 cm inside the ankle) and ramps across
   in double support.
3. **Preview control** (Kajita 2003): a linear inverted pendulum with the CoM at 1.12 m,
   1.6 s preview, 200 Hz.
4. **Leg IK** (`harness_gait.leg_ik_3d`) places the pelvis at the planned CoM minus the
   body's CoM offset.
5. **Feedback:**
   - model-based gravity feed-forward;
   - IMU ankle and hip stabilisers;
   - CoM feedback from leg kinematics. The lateral term is what made it walk.

## Taking over without a jolt

`AtlasPlugin` holds Atlas with a PD controller whose setpoint is all zeros; gravity makes
each joint sag a little below it. That sag is what produces the holding torque.
Commanding the *measured* pose would zero the error and drop Atlas. Every controller in
this repo therefore starts from the controller's *setpoint*, reconstructed from
`atlas/atlas_state` as `position + effort / kp`. See [tutorial 8](08_your_own_controller.md).

Further reading: [atlas_walking_demo/README.md](../../atlas_walking_demo/README.md) has the
measured results and tuning notes.
