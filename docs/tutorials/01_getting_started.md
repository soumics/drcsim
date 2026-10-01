# 1. Getting started

## Start the simulation

With Docker (see [docker/README.md](../../docker/README.md) for requirements):

```bash
docker/build.sh && docker/run.sh     # once
docker/sim.sh                        # (re)start the simulation
docker/gui.sh                        # Gazebo GUI + RViz
```

Natively: `ros2 launch drcsim_gazebo atlas.launch.py` (add `headless:=true` for no GUI).

What happens:
1. `VRCPlugin` spawns Atlas from `/robot_description`.
2. It holds the pelvis pinned for a few seconds, then lets go.
3. Atlas stands under `AtlasPlugin`'s joint PD controller (gains from
   `drcsim_gazebo/config/atlas_v5_gains.yaml`).

It slowly rocks about 3° forward and back. That's expected: the default controller has no
balance feedback.

`docker/sim.sh` restarts the simulation every time it runs. Restart after a fall, and
before every test: two simulations on the same ROS domain and gz partition mix their
data, and `sim.sh` warns about that.

## Launch arguments

| Argument | Default | Values |
|---|---|---|
| `hands` | `svh` | `svh` (SCHUNK five-finger), `sandia`, `robotiq`, `none` |
| `skin` | `accents` | `accents` (original textures + rose-gold hands, cyan lights), `glam`, `classic` |
| `x`, `y`, `z`, `roll`, `pitch`, `yaw` | `0 0 0.90 0 0 0` | spawn pose |
| `startup_mode` | `pinned` | `pinned` (hold, then release); `bdi_stand` is unreliable |
| `headless` | `false` | server only |
| `demo_camera` | `false` | the chase camera used for videos |
| `sync_max_per_step` / `sync_max_per_window` | `0.025` / `0.25` | lockstep budgets (s); use `0.05` / `5.0` for torque control |
| `lidar_spindle_speed` | `1.5` | rad/s |
| `cheats_enabled` | `true` | `VRC_CHEATS_ENABLED`: harness `cmd_vel`, foot contact debug topics |
| `robot_xacro` | | a custom robot xacro |

Example: `docker/sim.sh hands:=sandia skin:=classic`.

## Topics

| Topic | What |
|---|---|
| `atlas/atlas_state` | full robot state at 1 kHz (joints, IMU, foot and wrist F/T) |
| `atlas/atlas_command` | joint commands to `AtlasPlugin` ([tutorial 8](08_your_own_controller.md)) |
| `atlas/joint_states`, `/tf` | for RViz and `robot_state_publisher` |
| `atlas/imu` | pelvis IMU |
| `atlas/mode` | `pinned`, `pinned_with_gravity`, `nominal`, `recover` |
| `atlas/cmd_vel` | move the harness while pinned |
| `atlas/debug/{l,r}_foot_contact` | foot contact wrenches |
| `multisense/left/image_raw`, `multisense/right/image_raw` | head stereo cameras |
| `multisense/lidar_points`, `multisense/lidar_scan` | spinning lidar |
| `atlas/{l,r}_situational_awareness_camera/image_raw` | torso cameras |
| `svh_hands/{left,right}/command`, `.../joint_states` | five-finger hands |
| `/clock` | simulation time; use `use_sim_time` |

## Try it

```bash
docker/shell.sh
ros2 topic echo --once /atlas/imu
ros2 topic pub --once /atlas/mode std_msgs/msg/String "{data: pinned}"    # hold the pelvis
ros2 topic pub --once /atlas/mode std_msgs/msg/String "{data: nominal}"   # let go
drcsim_push 0 300 0.2              # push the torso (N, N, s of sim time)
```

Gazebo's **Apply Force/Torque** GUI tool also works for pushing Atlas with the mouse.

## Performance

With every sensor rendering, Atlas simulates at about 0.6× real time on an RTX GPU. All
controllers here run on simulation time, so results don't depend on the speed.
