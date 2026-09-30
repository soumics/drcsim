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
Stand Atlas under torque control (whole-body QP + DCM balance).

usage: ros2 run atlas_walking_demo torque_stand.py
Takes over like free_walk.py and crouches in position mode
(balance_controller), then switches to torque mode: AtlasPlugin
kp_position = 0 and a small joint damping, effort = torque_balance's
torques, every AtlasState message (1 kHz of sim time; each solve takes
about 0.5 ms). Needs Pinocchio and ProxQP (in the Docker image). Stops
commanding past 35 deg of tilt. Parameters: torque_balance.DEFAULTS.
"""

import math

from atlas_msgs.msg import AtlasCommand, AtlasState
import balance_controller as bc
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import torque_balance as tb

N = bc.N


class TorqueStandNode(Node):
    """Crouch in position mode, then balance in torque mode."""

    def __init__(self):
        super().__init__('atlas_torque_stand',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.params = {name: self.declare_parameter(name, value).value
                       for name, value in tb.DEFAULTS.items()}
        self.urdf = None
        self.position = None
        self.torque = None
        self.fallen = False
        self.last_log = -1.0
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.create_subscription(
            String, '/robot_description', self._on_urdf,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Depth 1: always the newest state; each solve takes ~0.5 ms and
        # AtlasState arrives at 1 kHz, so a deeper queue would feed old states.
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 1)

    def _on_urdf(self, msg):
        self.urdf = msg.data

    def _on_state(self, s):
        if self.urdf is None or len(s.position) != len(N) or self.fallen:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        if self.position is None:
            self.position = bc.FreeWalkController(s)
            self.get_logger().info('Took over; crouching in position mode.')
        roll, pitch = bc.rpy(s.orientation)
        if max(abs(roll), abs(pitch)) > bc.FALL_TILT:
            self.fallen = True
            self.get_logger().error(
                f'Fell (roll {math.degrees(roll):+.0f}, pitch {math.degrees(pitch):+.0f} deg); '
                'stopped.')
            return
        cmd = AtlasCommand()
        cmd.k_effort = [255] * len(N)
        if self.torque is None:
            out = self.position.update(s, t)
            cmd.position, cmd.effort = out.position, out.effort
            cmd.kp_position, cmd.kd_position = out.kp, out.kd
            if self.position.status == 'ready':
                self.torque = tb.TorqueBalance(self.urdf, N, s, self.params)
                self.get_logger().info('Standing; switched to torque control.')
        else:
            tau = self.torque.update(s)
            cmd.position = [0.0] * len(N)
            cmd.effort = [float(x) for x in tau]
            cmd.kp_position = [0.0] * len(N)
            cmd.kd_position = [self.params['kd_joint']] * len(N)
        self.pub.publish(cmd)
        if self.torque is not None and t - self.last_log >= 1.0:
            self.last_log = t
            err = 100 * (self.torque.com - self.torque.com_target)
            self.get_logger().info(
                f'torque mode: roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f} '
                f'contacts={self.torque.contacts(s)} CoM error=({err[0]:+.1f}, {err[1]:+.1f}, '
                f'{err[2]:+.1f}) cm')


def main(args=None):
    rclpy.init(args=args)
    node = TorqueStandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
