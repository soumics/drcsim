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
// sine-wave trajectory via a dedicated worker thread, decoupled from the
// AtlasState subscription callback -- useful for exercising
// AtlasPlugin's atlas_command/atlas_state round trip under a simulated
// "controller does its own thing on its own schedule" workload. See
// pub_atlas_command_fast.cpp for the alternative (compute-and-publish
// synchronously inside the subscription callback) used for round-trip
// latency testing instead.

#include <chrono>
#include <cmath>
#include <cstdlib>
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
unsigned int g_seed = 0;

void SetAtlasState(const atlas_msgs::msg::AtlasState::SharedPtr _as)
{
  static const rclcpp::Time startTime = g_node->now();
  g_t0 = startTime;

  std::lock_guard<std::mutex> lock(g_mutex);
  g_as = *_as;
}

void Work()
{
  while (rclcpp::ok()) {
    {
      // for testing round trip time
      std::lock_guard<std::mutex> lock(g_mutex);
      g_ac.header.stamp = g_as.header.stamp;
    }

    // simulate working
    std::this_thread::sleep_for(std::chrono::microseconds(2000));

    // assign arbitrary joint angle targets
    const double elapsed = (g_node->now() - g_t0).seconds();
    for (unsigned int i = 0; i < g_numJoints; ++i) {
      const double randNorm =
        2.0 * static_cast<double>(rand_r(&g_seed)) / static_cast<double>(RAND_MAX) - 1.0;
      g_ac.position[i] = 3.2 * std::sin(randNorm + elapsed);
      g_ac.k_effort[i] = 255;
    }

    // Let AtlasPlugin know that a response over /atlas/atlas_command is
    // expected every 5ms, and to wait for AtlasCommand if none has been
    // received yet. Use up the delay budget if wait is needed.
    g_ac.desired_controller_period_ms = 5;

    g_pubAtlasCommand->publish(g_ac);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("pub_atlas_command");

  const int atlasVersion = g_node->declare_parameter("atlas_version", 5);
  const int atlasSubVersion = g_node->declare_parameter("atlas_sub_version", 0);

  // Atlas version 4.0 has no wry2 joints.
  if ((atlasVersion == 4 && atlasSubVersion == 0) || atlasVersion > 4) {
    g_numJoints = 30;
  } else {
    g_numJoints = 28;
  }

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

  // simulated worker thread
  std::thread workThread(Work);

  rclcpp::spin(g_node);
  workThread.join();
  rclcpp::shutdown();

  return 0;
}
