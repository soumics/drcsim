/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Diagnostic/demo tool: drives every Atlas joint through an arbitrary
// sine-wave trajectory via the lower-level osrf_msgs/JointCommands
// topic (AtlasPlugin::SetJointCommands), rather than atlas_command.
// Per-joint PID gains are read from the same "atlas_controller.gains.
// <joint>.{p,i,d,i_clamp}" parameters AtlasPlugin itself loads (see
// LoadPIDGainsFromParameter()) -- pass a matching params file on the
// command line (--ros-args --params-file ...) for these to be anything
// other than zero.
//
// AtlasPlugin::SetJointCommands matches incoming arrays purely by
// length, not by the message's `name` field (see its size-equality
// guards), so the joint *order* below must match AtlasPlugin's own
// (see AtlasPlugin::Load's jointNames construction) even though the
// names themselves are only used here, for the parameter lookup.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <osrf_msgs/msg/joint_commands.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<osrf_msgs::msg::JointCommands>::SharedPtr g_pubJointCommands;
osrf_msgs::msg::JointCommands g_jc;

void SetJointStates(const sensor_msgs::msg::JointState::SharedPtr _js)
{
  static const rclcpp::Time startTime = g_node->now();

  // for testing round trip time
  g_jc.header.stamp = _js->header.stamp;

  // assign arbitrary joint angle targets
  const double elapsed = (g_node->now() - startTime).seconds();
  for (unsigned int i = 0; i < g_jc.name.size(); ++i) {
    g_jc.position[i] = 3.2 * std::sin(elapsed);
  }

  g_pubJointCommands->publish(g_jc);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("pub_joint_commands");

  const int atlasVersion = g_node->declare_parameter("atlas_version", 5);
  const int atlasSubVersion = g_node->declare_parameter("atlas_sub_version", 0);
  const bool hasWry2 = (atlasVersion == 4 && atlasSubVersion == 0) || atlasVersion > 4;

  // Must match AtlasPlugin::Load's joint order (current canonical names;
  // AtlasPlugin itself falls back to older per-joint names if these
  // aren't found on the model, but that fallback is irrelevant here
  // since these names are only used for the parameter lookup below).
  g_jc.name = {
    "back_bkz", "back_bky", "back_bkx", "neck_ry",
    "l_leg_hpz", "l_leg_hpx", "l_leg_hpy", "l_leg_kny", "l_leg_aky", "l_leg_akx",
    "r_leg_hpz", "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky", "r_leg_akx",
    "l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx"};
  if (hasWry2) {
    g_jc.name.push_back("l_arm_wry2");
  }
  g_jc.name.insert(
    g_jc.name.end(),
    {"r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx"});
  if (hasWry2) {
    g_jc.name.push_back("r_arm_wry2");
  }

  const unsigned int n = g_jc.name.size();
  g_jc.position.assign(n, 0.0);
  g_jc.velocity.assign(n, 0.0);
  g_jc.effort.assign(n, 0.0);
  g_jc.kp_position.assign(n, 0.0);
  g_jc.ki_position.assign(n, 0.0);
  g_jc.kd_position.assign(n, 0.0);
  g_jc.kp_velocity.assign(n, 0.0);
  g_jc.i_effort_min.assign(n, 0.0);
  g_jc.i_effort_max.assign(n, 0.0);

  for (unsigned int i = 0; i < n; ++i) {
    const std::string prefix = "atlas_controller.gains." + g_jc.name[i] + ".";
    g_jc.kp_position[i] = g_node->declare_parameter(prefix + "p", 0.0);
    g_jc.ki_position[i] = g_node->declare_parameter(prefix + "i", 0.0);
    g_jc.kd_position[i] = g_node->declare_parameter(prefix + "d", 0.0);
    const double iClamp = g_node->declare_parameter(prefix + "i_clamp", 0.0);
    g_jc.i_effort_min[i] = -iClamp;
    g_jc.i_effort_max[i] = iClamp;
  }

  // This wait is needed to ensure this node has gotten a simulation time
  // update (requires use_sim_time:=true and a /clock bridge).
  while (rclcpp::ok() && g_node->now().seconds() <= 0) {
  }

  auto subJointStates = g_node->create_subscription<sensor_msgs::msg::JointState>(
    "/atlas/joint_states", rclcpp::QoS(1), SetJointStates);

  g_pubJointCommands = g_node->create_publisher<osrf_msgs::msg::JointCommands>(
    "/atlas/joint_commands", rclcpp::QoS(1).transient_local());

  rclcpp::spin(g_node);
  rclcpp::shutdown();

  return 0;
}
