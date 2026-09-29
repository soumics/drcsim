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

import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
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
    'sandia': [f'sandia_hands/{s}_hand/joint_states' for s in ('l', 'r')],
    'robotiq': [f'robotiq_hands/{s}_hand/joint_states' for s in ('left', 'right')],
    'irobot': [f'irobot_hands/{s}_hand/joint_states' for s in ('l', 'r')],
}

# skin:=modern -- a "brand-new car" paint job: physically based materials
# (gz-sim/Ogre2 PBR) replacing the meshes' old textures. Each entry is
# (diffuse RGB, metalness, roughness): pearl-white multi-coat body panels,
# gloss piano-black joints and trim, graphite-metallic hands, an "ultra
# red" head as the badge.
PAINTS = {
    'pearl': ((0.92, 0.93, 0.95), 0.25, 0.12),
    'piano_black': ((0.02, 0.02, 0.025), 0.2, 0.08),
    'graphite': ((0.20, 0.21, 0.23), 0.85, 0.28),
    'ultra_red': ((0.62, 0.02, 0.04), 0.45, 0.15),
}
# First matching substring of the link/visual name wins.
PAINT_RULES = [
    (('hokuyo',), 'piano_black'),
    (('head', 'multisense'), 'ultra_red'),
    (('palm', '_f0', '_f1', '_f2', '_f3', 'finger', 'irobot', 'robotiq', 'base_link'),
     'graphite'),
    (('utorso', 'pelvis', 'uleg', 'lleg', 'uarm', 'larm', 'ufarm'), 'pearl'),
]


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


def _paint_for(name):
    for keys, paint in PAINT_RULES:
        if any(key in name for key in keys):
            return paint
    return 'piano_black'


def _modern_skin_sdf(robot_description):
    """
    Convert the URDF to SDF and give every visual a PBR car-paint material.

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
            rgb, metalness, roughness = PAINTS[_paint_for(visual.get('name', link.get('name')))]
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
            'hands', default_value='sandia', choices=list(HAND_XACROS),
            description='Hands on Atlas: Sandia (4 fingers), Robotiq (3-finger '
                        'gripper), or none.'),
        DeclareLaunchArgument(
            'skin', default_value='modern', choices=['modern', 'classic'],
            description="Atlas's look in Gazebo: modern car-paint PBR materials "
                        'or the original textures.'),
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
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    hands = LaunchConfiguration('hands').perform(context)
    robot_xacro = LaunchConfiguration('robot_xacro').perform(context) or os.path.join(
        get_package_share_directory('atlas_description'), 'robots', HAND_XACROS[hands])
    startup_mode = LaunchConfiguration('startup_mode').perform(context)
    gz_verbosity = LaunchConfiguration('gz_verbosity').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    cheats_enabled = LaunchConfiguration('cheats_enabled').perform(context).lower() in (
        'true', '1')
    pose = {
        axis: float(LaunchConfiguration(axis).perform(context))
        for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
    }

    robot_description = xacro.process_file(robot_xacro).toxml()
    skin = LaunchConfiguration('skin').perform(context)
    spawn_description = (
        _modern_skin_sdf(robot_description) if skin == 'modern' else robot_description)
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
    ]
