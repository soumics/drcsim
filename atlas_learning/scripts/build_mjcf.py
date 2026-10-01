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
Build a MuJoCo model of Atlas (v5 + SVH hands) from the same URDF Gazebo spawns.

usage: build_mjcf.py OUT.xml
- URDF: atlas_v5 + SVH via atlas.launch.py's own helpers (classic plugins
  stripped, SVH collision meshes -> boxes); visuals dropped; finger and lidar
  joints fixed; a URDF 'floating' joint frees the pelvis.
- MJCF additions: floor, 2 ms timestep, one torque motor per Atlas joint
  (ctrlrange = URDF effort limit), foot contact friction 1.0.
"""
import importlib.util
import os
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, os.path.join(get_package_prefix('atlas_walking_demo'), 'lib',
                                'atlas_walking_demo'))

import balance_controller as bc  # noqa: E402
import harness_gait as hg  # noqa: E402
import mujoco  # noqa: E402
import xacro  # noqa: E402
import yaml  # noqa: E402

N = hg.ATLAS_JOINT_NAMES
launch_path = os.path.join(get_package_share_directory('drcsim_gazebo'), 'launch',
                           'atlas.launch.py')
spec = importlib.util.spec_from_file_location('atlas_launch', launch_path)
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)

urdf = launch._strip_classic_plugins(xacro.process_file(launch._hand_xacro('svh')).toxml())
urdf = launch._svh_for_gz(urdf)
root = ET.fromstring(urdf)
for tag in ('gazebo', 'transmission'):
    for el in root.findall(tag):
        root.remove(el)
for link in root.findall('link'):
    for el in link.findall('visual'):
        link.remove(el)
    for col in link.findall('collision'):
        if col.find('geometry/mesh') is not None:
            link.remove(col)
for joint in root.findall('joint'):
    for el in joint.findall('mimic'):
        joint.remove(el)
    if joint.get('type') in ('revolute', 'continuous') and joint.get('name') not in N:
        joint.set('type', 'fixed')
# Free the pelvis: a URDF floating joint from a dummy root.
ET.SubElement(root, 'link', name='world_root')
float_joint = ET.SubElement(root, 'joint', name='floating_base', type='floating')
ET.SubElement(float_joint, 'parent', link='world_root')
ET.SubElement(float_joint, 'child', link='pelvis')
ET.SubElement(float_joint, 'origin', xyz='0 0 0.95', rpy='0 0 0')
mj = ET.SubElement(root, 'mujoco')
ET.SubElement(mj, 'compiler', discardvisual='true', balanceinertia='true',
              fusestatic='true', strippath='false')

model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
tmp = '/tmp/atlas_raw.xml'
mujoco.mj_saveLastXML(tmp, model)

mjcf = ET.parse(tmp).getroot()
opt = mjcf.find('option')
if opt is None:
    opt = ET.SubElement(mjcf, 'option')
opt.set('timestep', '0.001')
opt.set('integrator', 'implicitfast')
default = mjcf.find('default')
if default is None:
    default = ET.Element('default')
    mjcf.insert(1, default)
# No self-collision (as in Gazebo, where the URDF's overlapping torso and hip
# shapes never touch): robot geoms collide with the floor only.
ET.SubElement(default, 'geom', friction='1.0 0.02 0.001', condim='3',
              contype='2', conaffinity='1')
world = mjcf.find('worldbody')
ET.SubElement(world, 'light', pos='0 0 4', dir='0 0 -1', directional='true')
# Stiff floor contact (MuJoCo's default sank the soles 7 mm and let the
# heel roll, unlike Gazebo's rigid floor).
world.insert(0, ET.Element('geom', name='floor', type='plane', size='20 20 0.1',
                           rgba='0.8 0.8 0.8 1', contype='1', conaffinity='2',
                           solref='0.005 1', solimp='0.95 0.99 0.001'))
# AtlasPlugin's control law as MuJoCo position actuators, run inside the
# 1 kHz physics step: force = kp (target - q) - kd qdot, clamped to the URDF
# effort limit. Gains: atlas_v5_gains.yaml with the walking overrides
# (balance_controller.GAIN_OVERRIDES).
gains = yaml.safe_load(open(os.path.join(
    get_package_share_directory('drcsim_gazebo'), 'config', 'atlas_v5_gains.yaml')))
gains = gains['atlas_plugin']['ros__parameters']
act = ET.SubElement(mjcf, 'actuator')
limits = {j.get('name'): float(j.find('limit').get('effort'))
          for j in root.findall('joint') if j.get('name') in N}
for name in N:
    kp = gains[f'atlas_controller.gains.{name}.p']
    kd = gains[f'atlas_controller.gains.{name}.d']
    joint = name.rsplit('_', 1)[1]
    if '_leg_' in name and joint in bc.GAIN_OVERRIDES:
        kp, kd = bc.GAIN_OVERRIDES[joint]
    lim = limits[name]
    ET.SubElement(act, 'position', name=name, joint=name, kp=str(kp), kv=str(kd),
                  forcelimited='true', forcerange=f'{-lim} {lim}', ctrllimited='false')
ET.ElementTree(mjcf).write(sys.argv[1])
m = mujoco.MjModel.from_xml_path(sys.argv[1])
print(f'{sys.argv[1]}: nq={m.nq} nv={m.nv} nu={m.nu} bodies={m.nbody} geoms={m.ngeom} '
      f'mass={sum(m.body_mass):.1f} kg')
