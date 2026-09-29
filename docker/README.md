# drcsim in Docker (NVIDIA GPU + CycloneDDS)

A self-contained image for the whole repository:
- **Base:** ROS 2 Jazzy, Gazebo Harmonic (`osrf/ros:jazzy-desktop-full`).
- **Middleware:** CycloneDDS.
- **Dependencies:** every repo dependency, installed with rosdep.
- **Workspace:** prebuilt at `/root/ros2_ws`.

## Requirements

- Docker with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
  (`docker run --gpus all ...` must work).
- An X11 desktop; `docker/run.sh` runs `xhost +local:` for local containers.

## Quick start

```bash
docker/build.sh          # build the image drcsim:jazzy (first time: several minutes;
                         # fetches the GPL-3.0 SCHUNK SVH hand model next to the repo)
docker/run.sh            # start the container "drcsim" in the background
docker/sim.sh            # (re)start the simulation (Atlas, SVH five-finger hands, classic look + accents)
docker/gui.sh            # Gazebo GUI + RViz on your screen, camera following Atlas
docker/teleop.sh         # drive Atlas from the keyboard (Ctrl-C quits)
docker/stop.sh           # remove the container
```

`docker/sim.sh` passes its arguments to `atlas.launch.py`:

```bash
docker/sim.sh hands:=sandia             # Sandia four-finger hands
docker/sim.sh hands:=robotiq            # Robotiq 3-finger grippers
docker/sim.sh hands:=none skin:=classic # the original look, no hands
docker/sim.sh skin:=glam                # pearl & rose gold repaint (no logo)
docker/sim.sh demo_camera:=true         # add the chase camera used for demo videos
```

`docker/shell.sh` opens a shell with ROS and the workspace sourced.

Inside the container these commands are on `PATH`:

| Command | Does |
|---|---|
| `drcsim_sim` | launch the simulation |
| `drcsim_gui` | Gazebo GUI + RViz |
| `drcsim_teleop` | keyboard teleop |
| `drcsim_snapshot DIR` | save Gazebo/RViz screenshots (overview, each hand, front) from a private virtual display |
| `drcsim_record_demo DIR` | record the side-by-side Gazebo + RViz demo video (needs `demo_camera:=true`) |
| `drcsim_record_topic TOPIC OUT.mp4` | record any image topic to MP4 |

## Third-party hand model

- The default hands are the **SCHUNK SVH** (five fingers, 9 motors, 20 joints).
- Their description (`schunk_svh_description`) is **GPL-3.0-or-later**. It is never copied into
  this Apache-2.0 repository: `docker/fetch_external.sh` clones it next to the repo
  (`<workspace>/src/external/`), and the image build runs it.
- `atlas_svh_hands` (Apache-2.0) only references it. Check the GPL terms before distributing a
  combined product.
- Outside Docker, run `docker/fetch_external.sh` once before `colcon build`.

## Graphics

- `DRCSIM_PRIME_OFFLOAD=1` (the default) is for hybrid laptops, where an
  Intel GPU drives the display and the NVIDIA GPU renders.
  - The container then sets `__NV_PRIME_RENDER_OFFLOAD=1` and
    `__GLX_VENDOR_LIBRARY_NAME=nvidia` for GUI apps.
- On a desktop where the NVIDIA GPU drives the display, use
  `DRCSIM_PRIME_OFFLOAD=0 docker/run.sh`.
- `NVIDIA_DRIVER_CAPABILITIES=all` gives the container graphics, display,
  compute and utility access (`nvidia-smi` works inside).

## Middleware

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, configured by
  `docker/cyclonedds.xml`.
  - It uses loopback only, with unicast to localhost, so the simulation
    stays off the LAN and away from other ROS 2 systems on the machine.
- `ROS_DOMAIN_ID=77` and `GZ_PARTITION=drcsim` separate it further. Change
  them with `ROS_DOMAIN_ID=… docker/run.sh`.
- To talk to the simulation from outside the container, use the same RMW,
  domain and CycloneDDS config.
