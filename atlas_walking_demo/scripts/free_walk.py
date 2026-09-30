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
Scripted free-standing (no harness) walk: a number of steps, then stand.

usage: ros2 run atlas_walking_demo free_walk.py --ros-args -p steps:=10
  [-p step_length:=0.15] [-p side_speed:=0.0] [-p turn_speed:=0.0]
Runs balance_controller.FreeWalkController at 200 Hz of simulation time:
takes over AtlasPlugin's setpoint, crouches, settles, walks `steps` steps
(the last one bringing the feet together) and keeps balancing. If Atlas
falls it gets back up (auto_recover) and walks the steps again.
For keyboard control use walk_keyboard.py --ros-args -p harness:=false.
"""

from atlas_msgs.msg import AtlasCommand, AtlasState
import balance_controller as bc
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
import zmp_walk as zw

RECOVER_WAIT = 2.0  # s lying still before getting up


class FreeWalkNode(Node):
    """Walk a fixed number of steps free-standing; see the module docstring."""

    def __init__(self):
        super().__init__('atlas_free_walk',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.steps = self.declare_parameter('steps', 6).value
        step_length = self.declare_parameter('step_length', zw.STEP_LENGTH).value
        side_speed = self.declare_parameter('side_speed', 0.0).value
        turn_speed = self.declare_parameter('turn_speed', 0.0).value
        self.auto_recover = self.declare_parameter('auto_recover', True).value
        zw.SINGLE_SUPPORT = self.declare_parameter('single_support', zw.SINGLE_SUPPORT).value
        zw.DOUBLE_SUPPORT = self.declare_parameter('double_support', zw.DOUBLE_SUPPORT).value
        zw.ZMP_Y_INSET = self.declare_parameter('zmp_y_inset', zw.ZMP_Y_INSET).value
        self.params = {name: self.declare_parameter(name, value).value
                       for name, value in bc.DEFAULT_PARAMS.items()}
        t_step = zw.SINGLE_SUPPORT + zw.DOUBLE_SUPPORT
        self.velocity = (step_length / t_step, side_speed, turn_speed)
        self.controller = None
        self.state = None
        self.last_log = -1.0
        self.fell_at = None
        self.stopped = False
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.mode_pub = self.create_publisher(String, 'atlas/mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'atlas/cmd_vel', 10)
        self.create_subscription(AtlasState, 'atlas/atlas_state', self._on_state, 10)
        self.create_timer(zw.DT, self._tick)

    def _on_state(self, msg):
        self.state = msg
        if self.controller is None and len(msg.position) == len(bc.N):
            self.controller = bc.FreeWalkController(msg, self.params)
            self.controller.set_velocity(*self.velocity)
            self.get_logger().info(
                f'Took over; crouching, then {self.steps} steps '
                f'(vx={self.velocity[0]:.3f} m/s, vy={self.velocity[1]:.3f} m/s, '
                f'wz={self.velocity[2]:.3f} rad/s).')

    def _tick(self):
        c = self.controller
        if c is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        was_fallen = c.status == 'fallen'
        out = c.update(self.state, t)
        if c.status == 'fallen':
            if not was_fallen:
                self.fell_at = t
                self.get_logger().error(f'Fell ({c.fall_info}); '
                                        f'{"getting up" if self.auto_recover else "stopped"}.')
            elif self.auto_recover and t - self.fell_at >= RECOVER_WAIT:
                c.recover()
                self.stopped = False
                c.set_velocity(*self.velocity)
            return
        if not self.stopped and c.walker.planned_count >= self.steps - 1:
            self.stopped = True
            c.stop()
        while c.messages:
            self.get_logger().warn(c.messages.pop(0))
        if out.mode:
            self.get_logger().info(f'atlas/mode: {out.mode}')
            self.mode_pub.publish(String(data=out.mode))
        if out.harness_vz:
            twist = Twist()
            twist.linear.z = out.harness_vz
            self.cmd_vel_pub.publish(twist)
        msg = AtlasCommand()
        msg.position, msg.effort = out.position, out.effort
        msg.k_effort = [255] * len(bc.N)
        msg.kp_position, msg.kd_position = out.kp, out.kd
        self.pub.publish(msg)
        self._log(t)

    def _log(self, t):
        c = self.controller
        if t - self.last_log < 0.5:
            return
        self.last_log = t
        roll, pitch = bc.rpy(self.state.orientation)
        s, info = self.state, c.info
        err = c.com_err or (0.0, 0.0)
        self.get_logger().info(
            f'{c.status:10s} {info["phase"]:8s} pitch={57.3 * pitch:+5.1f} '
            f'roll={57.3 * roll:+5.1f} Fz L/R={-s.l_foot.force.z:5.0f}/{-s.r_foot.force.z:5.0f} '
            f'com=({info["com"][0]:+.2f},{info["com"][1]:+.2f}) '
            f'comerr=({100 * err[0]:+.1f},{100 * err[1]:+.1f})cm '
            f'steps={c.walker.planned_count}')
        if self.stopped and c.status == 'ready' and not c.walking and not getattr(
                self, 'announced', False):
            self.announced = True
            self.get_logger().info('Walk finished; standing.')


def main(args=None):
    rclpy.init(args=args)
    node = FreeWalkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
