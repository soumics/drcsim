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

"""Axis-aligned bounding boxes of COLLADA meshes, scene-graph transforms applied.

usage: dae_bbox.py file.dae ...   -> prints YAML: name: [cx, cy, cz, sx, sy, sz] (metres)
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

C = '{http://www.collada.org/2005/11/COLLADASchema}'


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def ident():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def node_matrix(node):
    m = ident()
    for el in node:
        tag = el.tag.replace(C, '')
        v = [float(x) for x in (el.text or '').split()]
        if tag == 'matrix':
            t = [v[i * 4:(i + 1) * 4] for i in range(4)]
        elif tag == 'translate':
            t = ident()
            t[0][3], t[1][3], t[2][3] = v
        elif tag == 'scale':
            t = ident()
            t[0][0], t[1][1], t[2][2] = v
        elif tag == 'rotate':
            x, y, z, a = v[0], v[1], v[2], math.radians(v[3])
            n = math.sqrt(x * x + y * y + z * z) or 1.0
            x, y, z = x / n, y / n, z / n
            c, s, k = math.cos(a), math.sin(a), 1 - math.cos(a)
            t = [[c + x * x * k, x * y * k - z * s, x * z * k + y * s, 0],
                 [y * x * k + z * s, c + y * y * k, y * z * k - x * s, 0],
                 [z * x * k - y * s, z * y * k + x * s, c + z * z * k, 0], [0, 0, 0, 1]]
        else:
            continue
        m = matmul(m, t)
    return m


def bbox(path):
    root = ET.parse(path).getroot()
    unit = root.find(f'{C}asset/{C}unit')
    scale = float(unit.get('meter')) if unit is not None else 1.0
    up = root.find(f'{C}asset/{C}up_axis')
    geoms = {}
    for geom in root.iter(C + 'geometry'):
        pts = []
        mesh = geom.find(C + 'mesh')
        if mesh is None:
            continue
        vid = mesh.find(C + 'vertices')
        src_id = vid.find(C + 'input[@semantic="POSITION"]').get('source')[1:]
        for src in mesh.findall(C + 'source'):
            if src.get('id') == src_id:
                v = [float(x) for x in src.find(C + 'float_array').text.split()]
                pts = [v[i:i + 3] for i in range(0, len(v) - 2, 3)]
        geoms[geom.get('id')] = pts
    lo, hi = [math.inf] * 3, [-math.inf] * 3

    def walk(node, m):
        m = matmul(m, node_matrix(node))
        for inst in node.findall(C + 'instance_geometry'):
            for p in geoms.get(inst.get('url')[1:], []):
                q = [sum(m[i][k] * (p + [1.0])[k] for k in range(4)) for i in range(3)]
                for i in range(3):
                    lo[i], hi[i] = min(lo[i], q[i]), max(hi[i], q[i])
        for child in node.findall(C + 'node'):
            walk(child, m)

    for scene in root.iter(C + 'visual_scene'):
        for node in scene.findall(C + 'node'):
            walk(node, ident())
    if up is not None and up.text.strip() == 'Y_UP':  # rotate Y-up to Z-up
        lo, hi = [lo[0], -hi[2], lo[1]], [hi[0], -lo[2], hi[1]]
    lo, hi = [x * scale for x in lo], [x * scale for x in hi]
    return [(a + b) / 2 for a, b in zip(lo, hi)] + [b - a for a, b in zip(lo, hi)]


for f in sys.argv[1:]:
    print(f'{os.path.basename(f)}: [{", ".join(f"{v:.5f}" for v in bbox(f))}]')
