#!/usr/bin/env python3
#
# Copyright 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Extract a dancer's 3D pose from a video with MediaPipe Pose Landmarker.

usage: extract_pose.py VIDEO OUT.npz [MODEL.task]
Runs in MediaPipe's own Python environment (it needs numpy 2; ROS Jazzy's
is 1.26): drcsim's Docker image has it at /opt/mp_venv. Writes, per frame:
world (T, 33, 3) metric landmarks centred on the hips (MediaPipe's world
frame: x to the image's right, y down, z away from the camera), image
(T, 33, 2) normalised image coordinates (for overlays), visibility
(T, 33), valid (T,) -- plus fps and the frame size. Frames without a
detection are filled from the previous one.
"""

import sys

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision
import numpy as np


def main():
    video, out = sys.argv[1], sys.argv[2]
    model = sys.argv[3] if len(sys.argv) > 3 else '/root/dance/pose_landmarker_heavy.task'
    options = vision.PoseLandmarkerOptions(
        base_options=mp_tasks.BaseOptions(model_asset_path=model),
        running_mode=vision.RunningMode.VIDEO, num_poses=1,
        min_pose_detection_confidence=0.5, min_tracking_confidence=0.5)
    cap = cv2.VideoCapture(video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    world, image, vis, valid = [], [], [], []
    with vision.PoseLandmarker.create_from_options(options) as landmarker:
        k = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            rgb = mp.Image(image_format=mp.ImageFormat.SRGB,
                           data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            result = landmarker.detect_for_video(rgb, int(k * 1000 / fps))
            k += 1
            if result.pose_world_landmarks:
                w = result.pose_world_landmarks[0]
                n = result.pose_landmarks[0]
                world.append([[p.x, p.y, p.z] for p in w])
                image.append([[p.x, p.y] for p in n])
                vis.append([p.visibility for p in n])
                valid.append(True)
            elif world:
                world.append(world[-1])
                image.append(image[-1])
                vis.append([0.0] * 33)
                valid.append(False)
    if not world:
        sys.exit(f'no person detected in {video}')
    # Fill leading misses (none before the first detection were appended).
    np.savez(out, world=np.array(world), image=np.array(image), visibility=np.array(vis),
             valid=np.array(valid), fps=fps, size=np.array(size))
    print(f'{out}: {len(world)} frames at {fps:.1f} fps, '
          f'{int(np.sum(valid))} with a detection')


if __name__ == '__main__':
    main()
