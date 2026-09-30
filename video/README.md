# Demo videos

`atlas_demo.mp4` (2560×720, 25 fps, ~35 s) shows Atlas in Gazebo Harmonic (left) and RViz 2
(right), side by side:
- **Hands:** relaxed five-finger SCHUNK SVH hands, then gripping, opening flat and relaxing.
- **Walking:** forward, curving, side-stepping, backward, a kick from the side, and a turn in
  place.
- **Look:** Atlas's original black & white textures and Boston Dynamics logo, with rose-gold
  hands and cyan visor and chest lights (`skin:=accents`).
- **Views:** Gazebo and RViz both look at Atlas from the front-right.
- **RViz:** the spinning MultiSense lidar's 3D point cloud and scan, plus the head and torso
  camera feeds.

`atlas_free_demo.mp4` (2560×720, 25 fps, ~2 min): the same views with Atlas walking
**free-standing, no harness** (ZMP preview control + balance feedback):
- walking forward, a curve, side-stepping right, turning in place;
- a 1000 N, 0.4 s shove on the chest knocks it onto its back; after 2 s it gets back up
  by itself (an upright harness lifts it, the legs straighten into the stance, it is lowered
  and let go);
- then it walks on and stops.
Real-time factor while recording: 0.67.

Both are simulation time at true speed. The Gazebo side is a chase camera that follows
Atlas's position and heading smoothly and stays level (`FollowCameraPlugin`); an earlier
camera fixed to the pelvis rolled with every step and pointed at the sky when Atlas fell. Atlas with all its sensors simulates at about 0.5× real
time here; the Gazebo side is the server-rendered chase camera recorded frame by frame, and the
RViz side is retimed to match.

Record a new one (Docker; see `docker/README.md`):

```bash
docker/sim.sh demo_camera:=true
docker exec drcsim /entrypoint.sh drcsim_record_demo /tmp/video
docker cp drcsim:/tmp/video/atlas_demo.mp4 video/
# free-standing version (restart the sim first):
docker exec drcsim /entrypoint.sh drcsim_record_demo /tmp/video free
docker cp drcsim:/tmp/video/atlas_free_demo.mp4 video/
```

The `.mp4` files are git-ignored to keep the repository small.
