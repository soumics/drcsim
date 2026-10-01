# Tutorials

Hands-on guides for the ROS 2 Jazzy + Gazebo Harmonic port of DRCSim. Start with tutorial 1;
the others can be read in any order.

| # | Tutorial | You'll learn |
|---|---|---|
| 1 | [Getting started](01_getting_started.md) | launch options, hands, sensors, topics |
| 2 | [Walking](02_walking.md) | keyboard and Twist teleop, harness and free-standing gaits |
| 3 | [Whole-body torque control](03_torque_control.md) | the QP, DCM balance, walking, pushes, lockstep |
| 4 | [Learned push recovery](04_learned_push_recovery.md) | train in MuJoCo, run in Gazebo |
| 5 | [Pick-and-place](05_pick_and_place.md) | grasping, carrying a box while walking, placing it |
| 6 | [Dance from a video](06_dance.md) | pose extraction, retargeting, playback |
| 7 | [Recording videos](07_recording_videos.md) | side-by-side Gazebo + RViz demos |
| 8 | [Writing your own controller](08_your_own_controller.md) | AtlasCommand, bumpless takeover, torque mode |

The commands assume the Docker setup (`docker/run.sh`); inside the container, open a
shell with `docker/shell.sh`. Natively, source your workspace and run the same `ros2`
commands. The `drcsim_*` helper scripts are in `docker/in_container/`.
