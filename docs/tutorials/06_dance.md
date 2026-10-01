# 6. Dance from a video

`atlas_dance` turns a video of a dancer into Atlas motion. The pipeline has three steps:

```
video ──extract_pose.py──▶ pose.npz ──retarget.py──▶ moves.npz ──dance_player.py──▶ Atlas
        (MediaPipe, venv)              (Pinocchio IK)            (ROS node, Gazebo)
```

Use footage you have the rights to; no video is included.

## Setup (once per container)

```bash
docker exec -it drcsim /entrypoint.sh drcsim_install_learning   # includes /opt/mp_venv + model
```

MediaPipe needs numpy 2 and ROS Jazzy ships numpy 1.26, so MediaPipe lives in its own
venv, `/opt/mp_venv`.

## Run

```bash
docker cp my_dance.mp4 drcsim:/root/dance/
docker/shell.sh
cd /root/dance
S=$(ros2 pkg prefix atlas_dance)/lib/atlas_dance
/opt/mp_venv/bin/python $S/extract_pose.py my_dance.mp4 pose.npz pose_landmarker_heavy.task
python3 $S/retarget.py pose.npz moves.npz
# with the simulation running:
ros2 run atlas_dance dance_player.py --ros-args -p moves:=/root/dance/moves.npz -p legs:=torque
```

## Retargeting

The upper body only. The legs carry 180 kg and stay with the balance controller.

- **Back:** the chest frame (shoulders) relative to the pelvis frame (hips), as Atlas's
  z-y-x back chain. A constant calibration from Atlas's neutral pose removes the
  difference between human and robot proportions.
- **Arms:** a small least-squares IK per frame makes Atlas's upper arm and forearm point
  along the dancer's. It works on directions in the chest frame, so body size doesn't
  matter. The shoulder point must be the `shz` joint, which is fixed to the torso.
- **Neck:** the nose above or below the ears.
- **Smoothing:** each frame is warm-started from, and smoothed toward, the previous one.
  The round-trip test (Atlas's own poses as landmarks) recovers arm directions within
  0.8–3.4°.

## Leg modes (`legs:=`)

| Mode | What it does |
|---|---|
| `torque` | whole-body QP balance; the dance sets posture targets. Survives full-amplitude moves |
| `moonwalk` | backward glide: ZMP walking backward, the swing foot skimming 5 cm above the floor |
| `stand` | position-mode standing. Large moves can topple it (no fore-aft CoM feedback) |

Other parameters:
- `amplitude` (0.6): scales the moves;
- `max_joint_speed` (2.0 rad/s);
- `glide_speed`, `skim_height`;
- `start_delay`.

The node logs `dance start` in sim time, which you can use to sync a recording with the
source clip.
