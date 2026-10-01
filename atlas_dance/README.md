# atlas_dance

Make Atlas dance like a person in a video: MediaPipe 3D pose → upper-body retargeting →
playback while balancing.

| File | |
|---|---|
| `scripts/extract_pose.py` | MediaPipe Pose Landmarker on a video (runs in `/opt/mp_venv`) |
| `scripts/retarget.py` | pose → Atlas back, arm and neck angles (Pinocchio IK) |
| `scripts/dance_player.py` | ROS node: plays the moves; legs `torque`, `moonwalk` or `stand` |

No video is included: use footage you have the rights to.

Full guide: [docs/tutorials/06_dance.md](../docs/tutorials/06_dance.md).
