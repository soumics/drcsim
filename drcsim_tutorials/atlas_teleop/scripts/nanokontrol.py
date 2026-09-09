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
Joystick input driver for the Korg nanoKONTROL MIDI controller.

Ported mechanically from the original drcsim tutorial of the same name --
requires the `pygame` package and physical nanoKONTROL hardware to run at
all; neither is available to verify this port beyond the rospy->rclpy and
Python 2->3 syntax translation. Original author: Austin Hendrix.
"""

import sys

import pygame
import pygame.midi

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from sensor_msgs.msg import Joy

CONTROL_AXES = [{
    # mode 1, sliders
    2: 0, 3: 1, 4: 2, 5: 3, 6: 4, 8: 5, 9: 6, 12: 7, 13: 8,
    # mode 1, knobs
    14: 9, 15: 10, 16: 11, 17: 12, 18: 13, 19: 14, 20: 15, 21: 16, 22: 17,
}, {
    # mode 2, sliders
    42: 0, 43: 1, 50: 2, 51: 3, 52: 4, 53: 5, 54: 6, 55: 7, 56: 8,
    # mode 2, knobs
    57: 9, 58: 10, 59: 11, 60: 12, 61: 13, 62: 14, 63: 15, 65: 16, 66: 17,
}, {
    # mode 3, sliders
    85: 0, 86: 1, 87: 2, 88: 3, 89: 4, 90: 5, 91: 6, 92: 7, 93: 8,
    # mode 3, knobs
    94: 9, 95: 10, 96: 11, 97: 12, 102: 13, 103: 14, 104: 15, 105: 16, 106: 17,
}, {
    # mode 4, sliders
    7: 0, 263: 1, 519: 2, 775: 3, 1031: 4, 1287: 5, 1543: 6, 1799: 7, 2055: 8,
    # mode 4, knobs
    10: 9, 266: 10, 522: 11, 778: 12, 1034: 13, 1290: 14, 1546: 15, 1802: 16,
    2058: 17,
}]

CONTROL_BUTTONS = [
    # mode 1: up/down x9, then rew/play/ff/repeat/stop/rec
    [23, 33, 24, 34, 25, 35, 26, 36, 27, 37, 28, 38, 29, 39, 30, 40, 31, 41,
     47, 45, 48, 49, 46, 44],
    # mode 2
    [67, 76, 68, 77, 69, 78, 70, 79, 71, 80, 72, 81, 73, 82, 74, 83, 75, 84,
     47, 45, 48, 49, 46, 44],
    # mode 3
    [107, 116, 108, 117, 109, 118, 110, 119, 111, 120, 112, 121, 113, 122,
     114, 123, 115, 124, 47, 45, 48, 49, 46, 44],
    # mode 4
    [16, 17, 272, 273, 528, 529, 784, 785, 1040, 1041, 1296, 1297, 1552,
     1553, 1808, 1809, 2064, 2065, 47, 45, 48, 49, 46, 44],
]


def main(args=None):
    pygame.midi.init()
    devices = pygame.midi.get_count()
    if devices < 1:
        print('No MIDI devices detected')
        sys.exit(1)
    print(f'Found {devices} MIDI devices')

    if len(sys.argv) > 1:
        input_dev = int(sys.argv[1])
    else:
        print('no input device supplied. will try to use default device.')
        input_dev = pygame.midi.get_default_input_id()
        if input_dev == -1:
            print('No default MIDI input device')
            sys.exit(1)
    print(f'Using input device {input_dev}')

    controller = pygame.midi.Input(input_dev)

    rclpy.init(args=args)
    node = Node('kontrol')
    pub = node.create_publisher(Joy, 'joy', 10)

    joy_msg = Joy()
    joy_msg.axes = [0.0] * 18
    joy_msg.buttons = [0] * 25
    mode = None
    changed = False

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            while controller.poll():
                for event in controller.read(1):
                    control = event[0]
                    if (control[0] & 0xF0) == 176:
                        control_id = control[1] | ((control[0] & 0x0F) << 8)

                        if mode is None:
                            candidate = None
                            for index, control_axis in enumerate(CONTROL_AXES):
                                if control_id in control_axis:
                                    if candidate is not None:
                                        candidate = None
                                        break
                                    candidate = index
                            for index, control_button in enumerate(CONTROL_BUTTONS):
                                if control_id in control_button:
                                    if candidate is not None:
                                        candidate = None
                                        break
                                    candidate = index
                            mode = candidate
                            if mode is None:
                                continue

                        if control_id in CONTROL_AXES[mode]:
                            control_val = min(1.0, max(0.0, float(control[2]) / 127.0))
                            axis = CONTROL_AXES[mode][control_id]
                            joy_msg.axes[axis] = control_val
                            changed = True

                        if control_id in CONTROL_BUTTONS[mode]:
                            button = CONTROL_BUTTONS[mode].index(control_id)
                            joy_msg.buttons[button] = 1 if control[2] != 0 else 0
                            changed = True
                    elif control[0] == 79:
                        mode = control[1]
                        joy_msg.buttons[24] = mode
                        changed = True

            if changed:
                joy_msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(joy_msg)
                changed = False

            node.get_clock().sleep_for(Duration(seconds=0.01))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
