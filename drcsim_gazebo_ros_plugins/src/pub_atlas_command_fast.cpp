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

// Diagnostic/demo tool: same sine-wave joint trajectory as
// pub_atlas_command.cpp, but computed and published synchronously inside
// the AtlasState subscription callback itself instead of from a separate
// worker thread -- a tighter, lower-latency response loop, useful for
// round-trip-time testing against AtlasPlugin's atlas_command/atlas_state
// topics.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <atlas_msgs/msg/atlas_command.hpp>
#include <atlas_msgs/msg/atlas_state.hpp>

namespace
{
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<atlas_msgs::msg::AtlasCommand>::SharedPtr g_pubAtlasCommand;
atlas_msgs::msg::AtlasCommand g_ac;
atlas_msgs::msg::AtlasState g_as;
std::mutex g_mutex;
rclcpp::Time g_t0;
unsigned int g_numJoints = 30;

void UpdateControl()
{
  {
    // for testing round trip time
    std::lock_guard<std::mutex> lock(g_mutex);
    g_ac.header.stamp = g_as.header.stamp;
  }

  // simulate working
  std::this_thread::sleep_for(std::chrono::microseconds(500));

  // assign arbitrary joint angle targets
  const double elapsed = (g_node->now() - g_t0).seconds();
  for (unsigned int i = 0; i < g_numJoints; ++i) {
    g_ac.position[i] = 3.2 * std::sin(elapsed);
    g_ac.k_effort[i] = 255;
  }

  // Let AtlasPlugin know that a response over /atlas/atlas_command is
  // expected every 2ms, and to wait for AtlasCommand if none has been
  // received yet. Use up the delay budget if wait is needed.
  g_ac.desired_controller_period_ms = 2;

  g_pubAtlasCommand->publish(g_ac);
}

void SetAtlasState(const atlas_msgs::msg::AtlasState::SharedPtr _as)
{
  static const rclcpp::Time startTime = g_node->now();
  g_t0 = startTime;

  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_as = *_as;
  }
  UpdateControl();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("pub_atlas_command_fast");

  // This wait is needed to ensure this node has gotten a simulation time
  // update (requires use_sim_time:=true and a /clock bridge).
  g_t0 = g_node->now();
  while (rclcpp::ok() && g_node->now().seconds() <= 0) {
  }

  g_ac.position.assign(g_numJoints, 0.0);
  g_ac.k_effort.assign(g_numJoints, 255);

  auto subAtlasState = g_node->create_subscription<atlas_msgs::msg::AtlasState>(
    "/atlas/atlas_state", rclcpp::QoS(100), SetAtlasState);

  g_pubAtlasCommand = g_node->create_publisher<atlas_msgs::msg::AtlasCommand>(
    "/atlas/atlas_command", rclcpp::QoS(100).transient_local());

  rclcpp::spin(g_node);
  rclcpp::shutdown();

  return 0;
}
