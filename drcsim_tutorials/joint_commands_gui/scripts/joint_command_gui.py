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
Per-joint slider GUI publishing osrf_msgs/JointCommands to Atlas.

Ported from the original drcsim joint_commands_gui tutorial, rewritten
against three real problems found in the original while porting it (see
CLAUDE.md): it used wxWidgets (not available in a stock ROS 2 Jazzy
desktop-full image -- this uses Tkinter, stdlib, instead), it filtered
joints by a stale "atlas::"-prefixed, 28-joint (v3/v4 Atlas) name list
(updated to atlas_v5's real, unprefixed, 30-joint names), and its
`source_list`/`FollowJointTrajectoryActionGoal` remote-control code path
referenced a message type it never imported -- dead on arrival in the
original, dropped here rather than ported.
"""

from math import pi
import threading
import tkinter as tk
import xml.dom.minidom

from osrf_msgs.msg import JointCommands
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

# The exact 30-joint order AtlasPlugin::Configure() builds internally --
# see atlas_teleop.py's ATLAS_JOINT_NAMES for the same list with a fuller
# cross-reference comment. Only joints in both this list and the received
# robot_description get a slider; this is how the original's "ignore the
# finger and multisense joints" filtering was implemented, just against
# up-to-date names.
ATLAS_JOINT_NAMES = [
    'back_bkz', 'back_bky', 'back_bkx', 'neck_ry',
    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
    'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx',
    'l_arm_wry2',
    'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx',
    'r_arm_wry2',
]

SLIDER_STEPS = 10000


def parse_free_joints(robot_description_xml, allowed_names):
    """
    Return {name: {min, max, zero}} for non-fixed joints in allowed_names.

    Pure function (no ROS/GUI), split out for unit testing -- see
    test/test_parse_free_joints.py.
    """
    robot = xml.dom.minidom.parseString(robot_description_xml).getElementsByTagName(
        'robot')[0]
    free_joints = {}
    for child in robot.childNodes:
        if child.nodeType == child.TEXT_NODE or child.localName != 'joint':
            continue
        jtype = child.getAttribute('type')
        if jtype == 'fixed':
            continue
        name = child.getAttribute('name')
        if name not in allowed_names:
            continue
        if jtype == 'continuous':
            minval, maxval = -pi, pi
        else:
            limit = child.getElementsByTagName('limit')[0]
            minval = float(limit.getAttribute('lower'))
            maxval = float(limit.getAttribute('upper'))
        zeroval = 0.0 if minval <= 0 <= maxval else (maxval + minval) / 2
        free_joints[name] = {'min': minval, 'max': maxval, 'zero': zeroval}
    return free_joints


class JointCommandGuiNode(Node):
    """
    Publishes JointCommands from either slider values or a fixed pose.

    Real per-joint gains already live in AtlasPlugin, loaded from
    atlas_v5_gains.yaml at launch time -- unlike the original, which read
    gains via the ROS 1 global parameter server (rospy.get_param(
    'atlas_controller/gains/...'), which has no ROS 2 equivalent since
    every node's parameters are private to it), this leaves kp/ki/kd/
    effort-limit fields empty. AtlasPlugin only ever applies those fields
    when their array length matches its own joint count (see
    AtlasPlugin::SetAtlasCommand), so leaving them empty means the real,
    already-loaded gains are simply left alone -- simpler and more
    correct than trying to duplicate them here.
    """

    def __init__(self):
        super().__init__('joint_command_gui')
        self.rate_hz = self.declare_parameter('rate', 10.0).value
        self.use_gui = self.declare_parameter('use_gui', True).value

        self.free_joints = {}
        self.joint_order = []
        # Slider-set target position per joint; read by the publish timer,
        # written only from the Tk mainloop thread -- see the module
        # docstring's note on why no lock is used for this.
        self.values = {}

        self.pub = self.create_publisher(JointCommands, '/atlas/joint_commands', 10)
        transient_local_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.description_sub = self.create_subscription(
            String, 'robot_description', self._on_robot_description, transient_local_qos)
        self.gui = None
        self.timer = None

    def _on_robot_description(self, msg):
        if self.joint_order:
            return  # already initialized from a first robot_description
        self.free_joints = parse_free_joints(msg.data, ATLAS_JOINT_NAMES)
        # Keep ATLAS_JOINT_NAMES's canonical order, not URDF file order.
        self.joint_order = [n for n in ATLAS_JOINT_NAMES if n in self.free_joints]
        self.values = {n: self.free_joints[n]['zero'] for n in self.joint_order}
        self.get_logger().info(
            f'Got robot_description: {len(self.joint_order)} joint sliders.')

        if self.use_gui:
            self.gui = JointCommandGuiWindow(self)
            threading.Thread(target=self.gui.mainloop, daemon=True).start()

        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish_command)

    def _publish_command(self):
        if not self.joint_order:
            return
        command = JointCommands()
        command.name = list(self.joint_order)
        command.position = [self.values[n] for n in self.joint_order]
        command.k_effort = [255] * len(self.joint_order)
        self.pub.publish(command)

    def set_value(self, name, value):
        self.values[name] = value

    def center_all(self):
        for name in self.joint_order:
            self.values[name] = self.free_joints[name]['zero']
        if self.gui is not None:
            self.gui.refresh_from_node()


class JointCommandGuiWindow(tk.Tk):
    """One label+slider row per joint, plus a Center button."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.title('Joint Command GUI')
        self.sliders = {}

        for name in node.joint_order:
            joint = node.free_joints[name]
            if joint['min'] == joint['max']:
                continue
            row = tk.Frame(self)
            row.pack(fill=tk.X, padx=4, pady=1)
            tk.Label(row, text=name, width=16, anchor='w').pack(side=tk.LEFT)
            value_label = tk.Label(row, text=f'{joint["zero"]:.2f}', width=8)
            value_label.pack(side=tk.RIGHT)
            slider = tk.Scale(
                self, from_=joint['min'], to=joint['max'], resolution=(
                    (joint['max'] - joint['min']) / SLIDER_STEPS),
                orient=tk.HORIZONTAL,
                command=lambda v, n=name, lbl=value_label: self._on_slider(n, v, lbl))
            slider.set(joint['zero'])
            slider.pack(fill=tk.X, padx=4)
            self.sliders[name] = (slider, value_label)

        tk.Button(self, text='Center', command=self._on_center).pack(fill=tk.X, pady=4)

    def _on_slider(self, name, value_str, value_label):
        value = float(value_str)
        self.node.set_value(name, value)
        value_label.config(text=f'{value:.2f}')

    def _on_center(self):
        self.node.center_all()

    def refresh_from_node(self):
        for name, (slider, value_label) in self.sliders.items():
            value = self.node.values[name]
            slider.set(value)
            value_label.config(text=f'{value:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandGuiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
