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

"""Publish the Sandia Hand's robot_description via robot_state_publisher.

Note: the original ROS 1 upload.launch pointed at
robots/sandia_hand.urdf.xacro, which does not exist (the actual file is
urdf/sandia_hand.urdf.xacro, a disabled mesh-based variant with no
standalone parent link) -- a pre-existing broken path. This uses
sandia_hand_left_on_box.urdf.xacro instead, the smallest xacro entry point
that actually produces a valid standalone robot.
"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('sandia_hand_description'),
        'robots',
        'sandia_hand_left_on_box.urdf.xacro',
    ])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
