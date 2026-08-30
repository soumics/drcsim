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
Publish robot_description and spawn the Sandia Hand into a running gz-sim world.

ROS 2 equivalent of the old `gazebo/spawn_model` node: `ros_gz_sim create`
spawns from the robot_description topic instead of a ROS 1 param.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    upload = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sandia_hand_description'),
                'launch', 'upload.launch.py',
            ])
        ),
    )

    spawn_sandia_hand_model = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_sandia_hand_model',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'sandia_hand',
            '-z', '1.4',
        ],
    )

    return LaunchDescription([
        upload,
        spawn_sandia_hand_model,
    ])
