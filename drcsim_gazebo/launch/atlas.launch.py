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
Bring up Atlas (v5, Sandia hands by default) in Gazebo Harmonic under VRCPlugin/AtlasPlugin.

Replaces the original drcsim's atlas.launch -> atlas_no_controllers.launch
-> atlas_bringup.launch -> atlas_v5_bringup.launch roslaunch XML chain.
Most of the other ~60 launch files in this package are the same template
with a different world/pose/robot-version/hand-suffix -- this is the one
representative, fully-working path (see CLAUDE.md for what's deferred).

Neither VRCPlugin nor AtlasPlugin can receive ROS parameters from a
launch file the normal way -- see drcsim_gazebo_ros_plugins'
RosNodeOptions.hpp for why (they're gz-sim System plugins loaded into
gzserver's single process, not started via `ros2 run`). This launch file
works around that the way RosNodeOptionsFromEnv() expects: it renders
the robot_description xacro, merges it with the initial-pose/startup-mode
parameters and the real per-joint PID gains (config/atlas_v5_gains.yaml)
into one combined ROS params YAML, writes it to a temp file, and points
the DRCSIM_ROS_PARAMS_FILE environment variable at it before gz-sim
(and therefore every plugin loaded into it) ever starts.
"""

import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


HAND_XACROS = {
    # Five-finger SCHUNK SVH hands (atlas_svh_hands + the separately fetched,
    # GPL-3.0 schunk_svh_description; docker/fetch_external.sh). Falls back
    # to Sandia when not installed.
    'svh': 'atlas_v5_svh_hands.urdf.xacro',
    'sandia': 'atlas_v5_sandia_hands.urdf.xacro',
    # Robotiq's finger linkages are closed kinematic loops; DART (gz-sim's
    # physics) drops the loop-closing joints, so the passive bars get no
    # joint state (no TF in RViz) -- the grippers still mount and actuate.
    'robotiq': 'atlas_v5_robotiq_hands.urdf.xacro',
    # No 'irobot': its fingers are closed loops too, and with their ~0.6 g
    # flex links the simulation diverges (NaN efforts, ODE collision
    # assertion) within seconds -- even with 10x masses. The xacro and its
    # mount are correct for URDF/RViz use (see CLAUDE.md).
    'none': 'atlas_v5.urdf.xacro',
}

# Joint-state topics each hand plugin publishes outside atlas/joint_states.
HAND_JOINT_STATE_TOPICS = {
    'svh': [f'svh_hands/{s}/joint_states' for s in ('left', 'right')],
    'sandia': [f'sandia_hands/{s}_hand/joint_states' for s in ('l', 'r')],
    'robotiq': [f'robotiq_hands/{s}_hand/joint_states' for s in ('left', 'right')],
    'irobot': [f'irobot_hands/{s}_hand/joint_states' for s in ('l', 'r')],
}

# Skins (gz only; RViz always shows the URDF's own textures):
# - accents (default): Atlas's original black & white textures and Boston
#   Dynamics logo untouched -- so Gazebo and RViz show the same body -- plus
#   rose-gold/pearl hands and glowing cyan (emissive) visor and chest light.
# - glam: "pearl & rose gold" -- every visual repainted with physically
#   based materials (gz-sim/Ogre2 PBR); replaces the textures, logo included.
# - classic: the original model, nothing added.
# Each paint is (diffuse RGB, metalness, roughness).
PAINTS = {
    # Metalness stays moderate: the world has no environment map, and a fully
    # metallic Ogre2 PBR surface takes nearly all its colour from reflecting
    # one -- rose gold at metalness 1.0 rendered black on the GPU.
    'pearl': ((0.97, 0.96, 0.94), 0.05, 0.18),
    'rose_gold': ((1.0, 0.72, 0.62), 0.15, 0.25),
    'gloss_black': ((0.02, 0.02, 0.025), 0.1, 0.12),
}
# First matching substring of the visual/link name wins.
PAINT_RULES = [
    (('svh_e1', 'svh_e2', 'svh_base', 'palm'), 'pearl'),
    (('head', 'hokuyo', 'multisense', 'foot'), 'gloss_black'),
    (('svh', '_f0', '_f1', '_f2', '_f3', 'finger'), 'rose_gold'),
    (('utorso', 'pelvis', 'uleg', 'lleg', 'uarm', 'larm', 'ufarm'), 'pearl'),
]
DEFAULT_PAINT = 'rose_gold'
# Visuals the accents skin paints (the hands); glam paints everything.
HAND_VISUAL_KEYS = ('svh', 'palm', '_f0', '_f1', '_f2', '_f3', 'finger')
# (link, name, pose "x y z r p y", geometry element, emissive RGB).
ACCENT_LIGHTS = [
    # Above the stereo cameras' field of view (+-24 deg vertically).
    ('head', 'visor_light', '0.045 0 0.05 0 0 0', ('box', {'size': '0.006 0.16 0.012'}),
     (0.0, 0.85, 1.0)),
    ('utorso', 'chest_light', '0.27 0 0.33 0 1.5708 0',
     ('cylinder', {'radius': '0.05', 'length': '0.008'}), (0.0, 0.85, 1.0)),
]


def _hand_xacro(hands):
    """Return the robot xacro for a `hands` choice (SVH falls back to Sandia)."""
    if hands == 'svh':
        try:
            return os.path.join(get_package_share_directory('atlas_svh_hands'), 'robots',
                                HAND_XACROS['svh'])
        except PackageNotFoundError:
            print('[atlas.launch.py] hands:=svh needs atlas_svh_hands and the separately '
                  'fetched schunk_svh_description (docker/fetch_external.sh); using Sandia.')
            hands = 'sandia'
    return os.path.join(
        get_package_share_directory('atlas_description'), 'robots', HAND_XACROS[hands])


def _strip_classic_plugins(robot_description):
    """
    Drop Gazebo Classic <plugin filename="lib*.so"> blocks from a URDF.

    Third-party descriptions (e.g. the SVH's gazebo_ros_control and mimic
    joint plugins) still carry them; gz-sim would only log a load error for
    each. This repo's own plugins are all in gz-sim form already.
    """
    root = ET.fromstring(robot_description)
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            filename = plugin.get('filename', '')
            if filename.startswith('lib') and filename.endswith('.so'):
                gazebo.remove(plugin)
        if not len(gazebo) and not (gazebo.text or '').strip():
            root.remove(gazebo)
    return ET.tostring(root, encoding='unicode')


def _rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _svh_for_gz(robot_description):
    """
    Adapt the SCHUNK SVH hands in a URDF for gz-sim (spawn only).

    * Collision meshes become their bounding boxes
      (atlas_svh_hands/config/svh_collision_boxes.yaml): DART's ODE
      collision backend segfaults on the meshes' degenerate submeshes.
    * <mimic> couplings move into each hand's HandJointController config:
      DART has no mimic constraints (gz logs an error per joint and ignores
      them); the controller enforces them instead.
    """
    if 'package://schunk_svh_description/' not in robot_description:
        return robot_description
    with open(os.path.join(get_package_share_directory('atlas_svh_hands'), 'config',
                           'svh_collision_boxes.yaml')) as boxes_file:
        boxes = yaml.safe_load(boxes_file)
    root = ET.fromstring(robot_description)
    for collision in root.iter('collision'):
        mesh = collision.find('geometry/mesh')
        if mesh is None or 'schunk_svh_description' not in mesh.get('filename', ''):
            continue
        cx, cy, cz, sx, sy, sz = boxes[os.path.basename(mesh.get('filename'))]
        scale = [float(v) for v in mesh.get('scale', '1 1 1').split()]
        center = [c * k for c, k in zip((cx, cy, cz), scale)]
        origin = collision.find('origin')
        if origin is None:
            origin = ET.SubElement(collision, 'origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rot = _rpy_matrix(*[float(v) for v in origin.get('rpy', '0 0 0').split()])
        xyz = [x + sum(rot[i][k] * center[k] for k in range(3)) for i, x in enumerate(xyz)]
        origin.set('xyz', ' '.join(f'{v:.6f}' for v in xyz))
        geometry = collision.find('geometry')
        geometry.remove(mesh)
        ET.SubElement(geometry, 'box', size=' '.join(
            f'{max(abs(s * k), 0.002):.5f}' for s, k in zip((sx, sy, sz), scale)))
    controllers = [p for p in root.iter('plugin') if p.findtext('joint_prefix')]
    for joint in root.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is None:
            continue
        for plugin in controllers:
            if joint.get('name').startswith(plugin.findtext('joint_prefix')):
                ET.SubElement(plugin, 'mimic', joint=joint.get('name'),
                              leader=mimic.get('joint'),
                              multiplier=mimic.get('multiplier', '1'),
                              offset=mimic.get('offset', '0'))
        joint.remove(mimic)
    return ET.tostring(root, encoding='unicode')


def _fix_robotiq_passive_joints(robot_description):
    """
    Return the URDF with the Robotiq fingers' passive linkage joints fixed.

    For robot_state_publisher/RViz only. DART can't close the fingers'
    parallel-linkage loops, so those joints are never simulated and never
    get a joint state: RViz then shows the whole RobotModel in error.
    Fixed, they get static TFs; only the actuated finger joints (joint_1..3
    and palm_finger_*_joint) keep moving.
    """
    root = ET.fromstring(robot_description)
    for joint in root.findall('joint'):
        name = joint.get('name')
        if 'finger_' in name and joint.get('type') != 'fixed' and not (
                name.endswith(('_joint_1', '_joint_2', '_joint_3')) or
                ('palm_finger_' in name and name.endswith('_joint'))):
            joint.set('type', 'fixed')
    return ET.tostring(root, encoding='unicode')


# Chase camera for demo videos (demo_camera:=true): front-right of Atlas,
# looking back at it. Its own model, rendered by the gz server (Ogre2, GPU,
# full PBR look). It used to be a sensor fixed to the pelvis, which rolled
# with every step and pointed at the sky when Atlas fell; FollowCameraPlugin
# follows the pelvis's smoothed position and heading at a fixed height.
DEMO_CAMERA_SDF = """<sdf version="1.9">
  <model name="demo_camera">
    <pose>0 0 -10 0 0 0</pose>
    <link name="link">
      <gravity>false</gravity>
      <inertial><mass>0.1</mass>
        <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia>
      </inertial>
      <sensor name="demo_camera" type="camera">
        <update_rate>25</update_rate>
        <topic>demo_camera/image</topic>
        <camera>
          <horizontal_fov>1.0</horizontal_fov>
          <image><width>960</width><height>540</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>100</far></clip>
        </camera>
      </sensor>
    </link>
    <plugin filename="FollowCameraPlugin" name="drcsim_gazebo_plugins::FollowCameraPlugin">
      <target_model>atlas</target_model>
      <target_link>pelvis</target_link>
      <offset>3.3 -2.8 0.3 0 0.06 2.44</offset>
      <time_constant>1.0</time_constant>
    </plugin>
  </model>
</sdf>"""


def _paint_for(name):
    for keys, paint in PAINT_RULES:
        if any(key in name for key in keys):
            return paint
    return DEFAULT_PAINT


def _skin_sdf(robot_description, paint_body):
    """
    Convert the URDF to SDF, paint visuals with PBR materials, add accent lights.

    paint_body=False (skin:=accents) paints only the hands and keeps the
    body's own textures; True (skin:=glam) repaints every visual.

    Only VRCPlugin's spawn (gz) sees this; robot_state_publisher/RViz keep
    the plain URDF. gz sdf -p is the same URDF->SDF conversion VRCPlugin's
    sdf::Root::LoadSdfString() would do, so the model is otherwise
    identical. Fixed-joint children are merged into their parent link by
    that conversion, so visuals are matched by their own (lumped) names.
    """
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as urdf:
        urdf.write(robot_description)
    try:
        sdf = subprocess.run(['gz', 'sdf', '-p', urdf.name], capture_output=True,
                             text=True, check=True).stdout
    finally:
        os.unlink(urdf.name)
    root = ET.fromstring(sdf[sdf.index('<sdf'):])
    for link in root.iter('link'):
        for visual in link.findall('visual'):
            name = visual.get('name', link.get('name'))
            if not paint_body and not any(key in name for key in HAND_VISUAL_KEYS):
                continue
            rgb, metalness, roughness = PAINTS[_paint_for(name)]
            for old in visual.findall('material'):
                visual.remove(old)
            color = ' '.join(f'{c:.3f}' for c in rgb) + ' 1'
            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color
            ET.SubElement(material, 'specular').text = '0.9 0.9 0.9 1'
            metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
            ET.SubElement(metal, 'metalness').text = str(metalness)
            ET.SubElement(metal, 'roughness').text = str(roughness)
    links = {link.get('name'): link for link in root.iter('link')}
    for link_name, name, pose, (shape, attrs), rgb in ACCENT_LIGHTS:
        if link_name not in links:
            continue
        visual = ET.SubElement(links[link_name], 'visual', name=name)
        ET.SubElement(visual, 'pose').text = pose
        geometry = ET.SubElement(ET.SubElement(visual, 'geometry'), shape)
        for key, value in attrs.items():
            ET.SubElement(geometry, key).text = value
        glow = ' '.join(f'{c:.3f}' for c in rgb) + ' 1'
        material = ET.SubElement(visual, 'material')
        for tag in ('ambient', 'diffuse', 'emissive'):
            ET.SubElement(material, tag).text = glow
    return ET.tostring(root, encoding='unicode')


def _sandia_hand_gains():
    """
    Return ROS params for both SandiaHandPlugin nodes from the hand's gains YAML.

    sandia_hand_gazebo_gains.yaml keys gains per side (gains.left_f0_j0: {p,
    d, ...}); each plugin instance is its own node,
    /sandia_hands/<l|r>_hand/sandia_hand_plugin, declaring gains.f0_j0.p etc.
    """
    path = os.path.join(
        get_package_share_directory('sandia_hand_description'), 'config',
        'sandia_hand_gazebo_gains.yaml')
    with open(path) as gains_file:
        gains = yaml.safe_load(gains_file)['gains']
    params = {}
    for side in ('left', 'right'):
        node = f'/sandia_hands/{side[0]}_hand/sandia_hand_plugin'
        values = params.setdefault(node, {'ros__parameters': {}})['ros__parameters']
        for joint, joint_gains in gains.items():
            if joint.startswith(side + '_'):
                for key, value in joint_gains.items():
                    values[f'gains.{joint[len(side) + 1:]}.{key}'] = float(value)
    return params


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('drcsim_model_resources'), 'worlds', 'atlas.world'),
            description='Path to the gz-sim world file to load.'),
        DeclareLaunchArgument(
            'hands', default_value='svh', choices=list(HAND_XACROS),
            description='Hands on Atlas: SCHUNK SVH (5 fingers), Sandia (4 '
                        'fingers), Robotiq (3-finger gripper), or none.'),
        DeclareLaunchArgument(
            'skin', default_value='accents', choices=['accents', 'glam', 'classic'],
            description="Atlas's look in Gazebo: accents (original textures and logo + "
                        'rose-gold hands and cyan lights), glam (pearl & rose gold '
                        'repaint) or classic.'),
        DeclareLaunchArgument(
            'robot_xacro', default_value='',
            description='Path to the robot xacro file VRCPlugin will spawn '
                        '(overrides `hands`).'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.90'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # VRCPlugin atlas.startup_mode. "pinned" (default): pin, hold, auto-unpin
        # after time_to_unpin, then AtlasPlugin's real gains hold the zero pose
        # against gravity -- the same simple path VRCPlugin's own gtest already
        # exercises reliably. "bdi_stand": the fuller pin/stand-prep/unpin/
        # dynamic-stand choreography -- currently unreliable, see CLAUDE.md.
        DeclareLaunchArgument('startup_mode', default_value='pinned'),
        DeclareLaunchArgument('gz_verbosity', default_value='3'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gz-sim server-only (-s), no GUI -- for CI/test use.'),
        # Sets VRC_CHEATS_ENABLED, gating VRCPlugin extras including the
        # atlas/cmd_vel warp-move topic.
        DeclareLaunchArgument('cheats_enabled', default_value='true'),
        DeclareLaunchArgument(
            'demo_camera', default_value='false',
            description='Add a chase camera following Atlas (demo_camera/image, 960x540 '
                        '@ 25 Hz, rendered by the gz server) for demo videos.'),
        DeclareLaunchArgument(
            'lidar_spindle_speed', default_value='1.5',
            description='MultiSense lidar spin rate (rad/s); 0 keeps it still (a 2D scan).'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    hands = LaunchConfiguration('hands').perform(context)
    robot_xacro = LaunchConfiguration('robot_xacro').perform(context) or _hand_xacro(hands)
    startup_mode = LaunchConfiguration('startup_mode').perform(context)
    gz_verbosity = LaunchConfiguration('gz_verbosity').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    cheats_enabled = LaunchConfiguration('cheats_enabled').perform(context).lower() in (
        'true', '1')
    pose = {
        axis: float(LaunchConfiguration(axis).perform(context))
        for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
    }

    robot_description = _strip_classic_plugins(xacro.process_file(robot_xacro).toxml())
    skin = LaunchConfiguration('skin').perform(context)
    gz_description = _svh_for_gz(robot_description)
    demo_camera = LaunchConfiguration('demo_camera').perform(context).lower() in ('true', '1')
    spawn_description = (
        _skin_sdf(gz_description, paint_body=skin == 'glam') if skin != 'classic'
        else gz_description)
    rsp_description = (
        _fix_robotiq_passive_joints(robot_description)
        if 'robotiq_hands' in os.path.basename(robot_xacro) else robot_description)

    gains_yaml_path = os.path.join(
        get_package_share_directory('drcsim_gazebo'), 'config', 'atlas_v5_gains.yaml')
    with open(gains_yaml_path) as gains_file:
        combined_params = yaml.safe_load(gains_file)

    combined_params.setdefault('vrc_plugin', {}).setdefault('ros__parameters', {}).update({
        'robot_description': spawn_description,
        'robot_initial_pose.x': pose['x'],
        'robot_initial_pose.y': pose['y'],
        'robot_initial_pose.z': pose['z'],
        'robot_initial_pose.roll': pose['roll'],
        'robot_initial_pose.pitch': pose['pitch'],
        'robot_initial_pose.yaw': pose['yaw'],
        'atlas.startup_mode': startup_mode,
        'atlas.time_to_unpin': 1.0,
        'atlas.delay_window_size': 5.0,
        'atlas.delay_max_per_window': 0.25,
        'atlas.delay_max_per_step': 0.025,
    })

    extra_joint_state_topics = ['multisense/joint_states']
    for hand, topics in HAND_JOINT_STATE_TOPICS.items():
        if f'{hand}_hands' in os.path.basename(robot_xacro):
            extra_joint_state_topics += topics
    if 'sandia_hands' in os.path.basename(robot_xacro):
        combined_params.update(_sandia_hand_gains())

    combined_params.setdefault('multisense_sl_plugin', {}).setdefault(
        'ros__parameters', {})['spindle_speed'] = float(
            LaunchConfiguration('lidar_spindle_speed').perform(context))

    params_fd, params_path = tempfile.mkstemp(
        prefix='drcsim_gazebo_ros_params_', suffix='.yaml')
    with os.fdopen(params_fd, 'w') as params_file:
        yaml.safe_dump(combined_params, params_file)

    server_only_flag = '-s ' if headless else ''
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{server_only_flag}-r -v {gz_verbosity} {world}'}.items())

    return [
        # Must be set before gz-sim (and therefore every plugin loaded
        # into its single process) starts -- see RosNodeOptionsFromEnv().
        SetEnvironmentVariable('DRCSIM_ROS_PARAMS_FILE', params_path),
        # Also must be set before gz-sim starts -- VRCPlugin reads this at
        # its own Configure() time, same as RosNodeOptionsFromEnv() above.
        SetEnvironmentVariable('VRC_CHEATS_ENABLED', '1' if cheats_enabled else '0'),
        gz_sim_launch,
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=[
                '--ros-args', '-p',
                'config_file:=' + os.path.join(
                    get_package_share_directory('drcsim_gazebo'), 'config', 'clock_bridge.yaml'),
            ],
            output='screen'),
        # Atlas's sensors (MultiSense stereo pair, spinning lidar, IMUs,
        # torso cameras) into ROS -- see config/sensors_bridge.yaml.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sensors_bridge',
            parameters=[{
                'config_file': os.path.join(
                    get_package_share_directory('drcsim_gazebo'), 'config',
                    'sensors_bridge.yaml'),
                'use_sim_time': True,
            }],
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='atlas_robot_state_publisher',
            parameters=[{'robot_description': rsp_description, 'use_sim_time': True}],
            remappings=[('joint_states', 'atlas/joint_states')],
            output='screen'),
    ] + [
        # Joints published outside atlas/joint_states -- the MultiSense
        # head's spinning lidar (hokuyo_joint) and the hands' fingers -- need
        # their own state publisher each; without their TFs RViz shows the
        # whole RobotModel in error (red). The original relayed these into
        # joint_states with topic_tools (not in Jazzy's base install).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=topic.replace('/', '_').replace('_joint_states', '_state_publisher'),
            parameters=[{'robot_description': rsp_description, 'use_sim_time': True}],
            remappings=[('joint_states', topic),
                        ('robot_description', topic.replace('joint_states', 'robot_description'))],
            output='screen')
        for topic in extra_joint_state_topics
    ] + ([
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'demo_camera', '-string', DEMO_CAMERA_SDF],
            output='screen'),
    ] if demo_camera else [])
