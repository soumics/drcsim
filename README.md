# DRCSim for ROS 2 Jazzy + Gazebo Harmonic

The Boston Dynamics **Atlas** humanoid from the DARPA Robotics Challenge
simulator ([osrf/drcsim](https://github.com/osrf/drcsim)), ported from ROS 1 +
Gazebo Classic to **ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8)**. It also adds
new controllers: walking, whole-body torque control, a learned push-recovery
policy, two-handed pick-and-place and dance retargeting from video.

| | |
|---|---|
| **Simulation** | Atlas v5 with its sensors (MultiSense SL stereo + spinning lidar, IMUs, foot/wrist F/T). Hands: five-finger SCHUNK SVH (default), Sandia or Robotiq |
| **Teleop walking** | keyboard or `Twist`, in every direction, in a harness or free-standing (ZMP preview control) |
| **Torque control** | whole-body QP (Pinocchio + ProxQP) at 1 kHz: DCM balance, contact-aware, joint and torque limits, hand tasks |
| **Learned policy** | PPO push-recovery policy trained in MuJoCo, runs in Gazebo (sim-to-sim transfer) |
| **Manipulation** | picks up a 10 kg box with both hands, carries it, sets it down, balancing all the way |
| **Dance** | MediaPipe pose from a video, retargeted onto Atlas's upper body; moonwalk glide |

Demo videos are listed in [video/README.md](video/README.md) (the `.mp4` files themselves
are not in git).

## Quick start (Docker, recommended)

You need Docker with the NVIDIA Container Toolkit and an X11 desktop. Details are in
[docker/README.md](docker/README.md).

```bash
git clone -b ros2-jazzy-harmonic https://github.com/soumics/drcsim.git
cd drcsim
docker/build.sh        # build the image (first time: several minutes)
docker/run.sh          # start the container "drcsim"
docker/sim.sh          # start the simulation: Atlas stands up after ~10 s
docker/gui.sh          # Gazebo GUI + RViz on your screen
docker/teleop.sh       # walk Atlas with the keyboard (w/a/s/d, q/e, space; Ctrl-C quits)
```

Everything else runs inside the container: `docker/shell.sh` opens a shell there with the
workspace sourced.

## Native build (Ubuntu 24.04)

```bash
# ROS 2 Jazzy desktop + Gazebo Harmonic installed; then:
mkdir -p ~/drcsim_ws/src && cd ~/drcsim_ws/src
git clone -b ros2-jazzy-harmonic https://github.com/soumics/drcsim.git
drcsim/docker/fetch_external.sh            # SVH hand description (GPL-3.0, kept outside the repo)
cd ~/drcsim_ws
rosdep install --from-paths src --ignore-src -y
sudo apt install ros-jazzy-pinocchio && pip install --break-system-packages proxsuite quadprog
colcon build --symlink-install
source install/setup.bash
ros2 launch drcsim_gazebo atlas.launch.py
```

The optional learning and dance tools (PyTorch, MuJoCo, Stable-Baselines3, MediaPipe) are
installed by `docker/in_container/drcsim_install_learning`; read it before running it on a
host.

## What to run

Start the simulation first (`docker/sim.sh`, or `ros2 launch drcsim_gazebo atlas.launch.py`),
then run **one** controller in another terminal. They all take over from the default
standing controller without a jolt.

| Demo | Command | Tutorial |
|---|---|---|
| Keyboard walking in a harness | `ros2 run atlas_walking_demo walk_keyboard.py` | [2](docs/tutorials/02_walking.md) |
| Free-standing walking | `ros2 run atlas_walking_demo walk_keyboard.py --ros-args -p harness:=false` | [2](docs/tutorials/02_walking.md) |
| Scripted free walk | `ros2 run atlas_walking_demo free_walk.py --ros-args -p steps:=10` | [2](docs/tutorials/02_walking.md) |
| Torque-controlled standing + pushes | `ros2 run atlas_walking_demo torque_stand.py`, then `drcsim_push 0 525 0.2` | [3](docs/tutorials/03_torque_control.md) |
| Learned push recovery | `ros2 run atlas_learning policy_stand.py` | [4](docs/tutorials/04_learned_push_recovery.md) |
| Pick-and-place (10 kg box) | `ros2 run atlas_walking_demo pick_place.py` | [5](docs/tutorials/05_pick_and_place.md) |
| Dance from a video | `extract_pose.py` → `retarget.py` → `dance_player.py` | [6](docs/tutorials/06_dance.md) |
| Record a demo video | `drcsim_record_demo /tmp/video [harness\|free\|push\|pick]` | [7](docs/tutorials/07_recording_videos.md) |

For the torque-controlled demos, the learned policy and pick-and-place, start the simulation
with a lockstep budget, so the Python controller is never late:

```bash
docker/sim.sh sync_max_per_window:=5.0 sync_max_per_step:=0.05
```

## Tutorials

All tutorials are in [docs/tutorials/](docs/tutorials/README.md):

1. [Getting started](docs/tutorials/01_getting_started.md): launch options, hands, sensors,
   topics, RViz.
2. [Walking](docs/tutorials/02_walking.md): harness gait, free-standing ZMP walking, teleop.
3. [Whole-body torque control](docs/tutorials/03_torque_control.md): the QP, DCM balance,
   pushes, lockstep.
4. [Learned push recovery](docs/tutorials/04_learned_push_recovery.md): MuJoCo model,
   training, evaluation, running in Gazebo.
5. [Pick-and-place](docs/tutorials/05_pick_and_place.md): hand tasks, payload, joint limits.
6. [Dance from a video](docs/tutorials/06_dance.md): pose extraction, retargeting, playback.
7. [Recording videos](docs/tutorials/07_recording_videos.md).
8. [Writing your own controller](docs/tutorials/08_your_own_controller.md): the AtlasCommand
   interface, bumpless takeover, position and torque modes.

## Repository layout

| Package | Contents |
|---|---|
| `drcsim_gazebo` | `atlas.launch.py`, world, gains, bridges, RViz config |
| `drcsim_gazebo_ros_plugins` | gz-sim system plugins: `AtlasPlugin` (1 kHz joint controller), `VRCPlugin` (spawn, pin/harness), hand and sensor plugins |
| `drcsim_gazebo_plugins` | vehicle/building plugins, `FollowCameraPlugin`, `HandJointController` |
| `atlas_description`, `*_hand_description`, `multisense_sl_description`, `atlas_svh_hands` | robot models |
| `atlas_msgs`, `handle_msgs`, `sandia_hand_msgs`, `osrf-common` | messages |
| `atlas_walking_demo` | walking, balance, whole-body QP, pick-and-place |
| `atlas_learning` | MuJoCo model, PPO training, learned policy node |
| `atlas_dance` | video → pose → Atlas retargeting and playback |
| `drcsim_tutorials` | the original tutorials, ported (teleop, joint sliders) |
| `drcsim_model_resources` | worlds and models from the original DRC |
| `docker/` | image, host scripts, in-container tools |

## Tests

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test && colcon test-result --verbose
```

The Docker image is built without tests; rebuild with `-DBUILD_TESTING=ON` before running
`colcon test` in the container.

## Status and limits

- **Free-standing walking** handles up to 0.25 m steps, at about 0.1 m/s.
- **Getting up after a fall:** Atlas can't do this by itself yet. `r` in teleop is a harness
  reset (a teleport), only meant for testing.
- **Capture-point stepping** under torque control is experimental.
- **Push recovery:** the learned policy handles 90 N·s in every direction and 150 N·s
  forward/backward.
- **iRobot hands** aren't offered: their closed-loop fingers diverge in DART.

`CLAUDE.md` is the detailed engineering log of the port: every design decision, gz-sim gap
and bug found.

## License and credits

- The original DRCSim is by the Open Source Robotics Foundation, Apache-2.0. This port and
  its new packages keep that license.
- The SCHUNK SVH hand description is **GPL-3.0-or-later**. It is fetched next to the repo,
  never copied into it; see [docker/README.md](docker/README.md).
- The dance pipeline doesn't include any video. Use footage you have the rights to.
