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

"""Joystick teleop for DRCVehicleROSPlugin and VRCPlugin's enter/exit-car cheat."""

import math

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Int8

AXIS_HAND_BRAKE = 0
AXIS_BRAKE_PEDAL = 1
AXIS_DIRECTION = 2
AXIS_GAS_PEDAL = 3
AXIS_HAND_WHEEL = 4

BUTTON_ENTER_CAR = 0
BUTTON_EXIT_CAR = 1


class DrcVehicleTeleop(Node):
    """Drives DRCVehicleROSPlugin's per-model cmd topics from a joystick."""

    def __init__(self):
        super().__init__('drc_vehicle_teleop')
        # DRCVehicleROSPlugin advertises every cmd/state topic under the
        # spawned vehicle model's own gz-sim name (Model(_entity).Name()),
        # not a fixed "drc_vehicle" prefix -- the original script hardcoded
        # "drc_vehicle/...", which never matched any world this migration
        # actually ships (atlas.world's vehicle model is "golf_cart").
        model = self.declare_parameter('vehicle_model_name', 'golf_cart').value

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        # VRCPlugin's enter/exit-car topics are fixed, world-scoped names,
        # not per-vehicle-model -- unlike the pedal/wheel topics below.
        self.robot_enter_car = self.create_publisher(Pose, 'drc_world/robot_enter_car', 10)
        self.robot_exit_car = self.create_publisher(Pose, 'drc_world/robot_exit_car', 10)
        self.brake_pedal = self.create_publisher(Float64, f'{model}/brake_pedal/cmd', 10)
        self.gas_pedal = self.create_publisher(Float64, f'{model}/gas_pedal/cmd', 10)
        self.hand_brake = self.create_publisher(Float64, f'{model}/hand_brake/cmd', 10)
        self.hand_wheel = self.create_publisher(Float64, f'{model}/hand_wheel/cmd', 10)
        self.direction = self.create_publisher(Int8, f'{model}/direction/cmd', 10)

    def joy_cb(self, msg):
        if msg.buttons[BUTTON_ENTER_CAR] == 1:
            self.robot_enter_car.publish(Pose())
        elif msg.buttons[BUTTON_EXIT_CAR] == 1:
            self.robot_exit_car.publish(Pose())
        else:
            self.hand_brake.publish(Float64(data=msg.axes[AXIS_HAND_BRAKE]))
            self.brake_pedal.publish(Float64(data=msg.axes[AXIS_BRAKE_PEDAL]))
            self.gas_pedal.publish(Float64(data=msg.axes[AXIS_GAS_PEDAL]))
            direction = -1 if msg.axes[AXIS_DIRECTION] < 0.5 else 1
            self.direction.publish(Int8(data=direction))
            hand_wheel = (msg.axes[AXIS_HAND_WHEEL] - 0.5) * math.pi
            self.hand_wheel.publish(Float64(data=hand_wheel))


def main(args=None):
    rclpy.init(args=args)
    node = DrcVehicleTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
