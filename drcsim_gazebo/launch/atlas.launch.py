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
Bring up Atlas (v5, no hands) in Gazebo Harmonic under VRCPlugin/AtlasPlugin.

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
import tempfile

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


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('drcsim_model_resources'), 'worlds', 'atlas.world'),
            description='Path to the gz-sim world file to load.'),
        DeclareLaunchArgument(
            'robot_xacro',
            default_value=os.path.join(
                get_package_share_directory('atlas_description'), 'robots',
                'atlas_v5.urdf.xacro'),
            description='Path to the robot xacro file VRCPlugin will spawn.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.90'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'startup_mode', default_value='bdi_stand',
            description='VRCPlugin atlas.startup_mode -- see its design notes.'),
        DeclareLaunchArgument('gz_verbosity', default_value='3'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    robot_xacro = LaunchConfiguration('robot_xacro').perform(context)
    startup_mode = LaunchConfiguration('startup_mode').perform(context)
    gz_verbosity = LaunchConfiguration('gz_verbosity').perform(context)
    pose = {
        axis: float(LaunchConfiguration(axis).perform(context))
        for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
    }

    robot_description = xacro.process_file(robot_xacro).toxml()

    gains_yaml_path = os.path.join(
        get_package_share_directory('drcsim_gazebo'), 'config', 'atlas_v5_gains.yaml')
    with open(gains_yaml_path) as gains_file:
        combined_params = yaml.safe_load(gains_file)

    combined_params.setdefault('vrc_plugin', {}).setdefault('ros__parameters', {}).update({
        'robot_description': robot_description,
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

    params_fd, params_path = tempfile.mkstemp(
        prefix='drcsim_gazebo_ros_params_', suffix='.yaml')
    with os.fdopen(params_fd, 'w') as params_file:
        yaml.safe_dump(combined_params, params_file)

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v {gz_verbosity} {world}'}.items())

    return [
        # Must be set before gz-sim (and therefore every plugin loaded
        # into its single process) starts -- see RosNodeOptionsFromEnv().
        SetEnvironmentVariable('DRCSIM_ROS_PARAMS_FILE', params_path),
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
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            remappings=[('joint_states', 'atlas/joint_states')],
            output='screen'),
    ]
