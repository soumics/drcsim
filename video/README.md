# Demo videos

`atlas_demo.mp4` (2560×720, 25 fps, ~35 s) shows Atlas in Gazebo Harmonic (left) and RViz 2
(right), side by side:
- **Hands:** relaxed five-finger SCHUNK SVH hands, then gripping, opening flat and relaxing.
- **Walking:** forward, curving, side-stepping, backward, a kick from the side, and a turn in
  place.
- **Look:** the pearl & rose-gold skin with cyan accent lights.
- **RViz:** the spinning MultiSense lidar's 3D point cloud and scan, plus the head and torso
  camera feeds.

It is simulation time at true speed. Atlas with all its sensors simulates at about 0.5× real
time here; the Gazebo side is the server-rendered chase camera recorded frame by frame, and the
RViz side is retimed to match.

Record a new one (Docker; see `docker/README.md`):

```bash
docker/sim.sh demo_camera:=true
docker exec drcsim /entrypoint.sh drcsim_record_demo /tmp/video
docker cp drcsim:/tmp/video/atlas_demo.mp4 video/
```

The `.mp4` files are git-ignored to keep the repository small.
