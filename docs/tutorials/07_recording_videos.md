# 7. Recording videos

`drcsim_record_demo` (in the Docker image) records **Gazebo** and **RViz** side by side,
both at true simulation speed:

- **Gazebo side:** a chase camera that follows Atlas smoothly and stays level
  (`FollowCameraPlugin`). The gz server renders it on the GPU, and it is recorded frame by
  frame in sim time.
- **RViz side:** RViz on a private virtual display (`Xvfb :98`), screen-recorded and then
  retimed by the measured real-time factor.

The chase camera exists only when the sim is started with `demo_camera:=true`.

```bash
docker/sim.sh demo_camera:=true                     # add sync_max_per_window:=5.0 for push/pick
docker exec drcsim /entrypoint.sh drcsim_record_demo /tmp/video MODE ...
docker cp drcsim:/tmp/video/NAME.mp4 video/
```

| MODE | Shows | Output |
|---|---|---|
| `harness` (default) | hands, walking in every direction, a kick, a turn | `atlas_demo.mp4` |
| `free` | free-standing walking | `atlas_free_demo.mp4` |
| `push FX FY DUR NAME "CAPTION"` | torque-controlled stand, one push | `NAME.mp4` |
| `pick [BOX_KG]` | pick-and-place | `atlas_pick_place.mp4` |

Set `DEMO_PUSH_CONTROLLER=policy` to film the learned policy instead of the torque
controller:

```bash
docker exec -e DEMO_PUSH_CONTROLLER=policy drcsim /entrypoint.sh \
  drcsim_record_demo /tmp/video push 600 0 0.2 atlas_rl_push_forward "Learned policy - 120 N s"
```

`push` and `pick` exit with status 1 if Atlas fell, so a script can retry until it gets a
clean take. Restart the simulation between takes (`docker/sim.sh`).

For a dance next to its source clip, use `drcsim_record_dance`
([tutorial 6](06_dance.md)).

Other tools:
- `drcsim_record_topic TOPIC OUT.mp4 [FPS] [wall|sim]` records any image topic.
- `drcsim_grab_frame TOPIC OUT.png` saves a single GPU-rendered frame.
- `drcsim_snapshot DIR` takes Gazebo and RViz screenshots.
