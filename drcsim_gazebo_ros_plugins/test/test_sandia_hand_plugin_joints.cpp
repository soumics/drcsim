/*
 * Copyright 2026 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/sim/TestFixture.hh>

#include <rclcpp/rclcpp.hpp>

#include <osrf_msgs/msg/joint_commands.hpp>
#include <sandia_hand_msgs/msg/raw_tactile.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Point gz-sim at the just-built plugin .so before any TestFixture is
// constructed, so the test doesn't depend on this package's environment
// hook having been sourced (it hasn't -- this runs straight out of the
// build tree, before install).
struct PluginPathSetter
{
  PluginPathSetter()
  {
    setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", PLUGIN_BUILD_DIR, 1);
  }
};
static PluginPathSetter g_pluginPathSetter;

static const char kJointCommandsTopic[] = "/sandia_hands/l_hand/joint_commands";
static const char kJointStatesTopic[] = "/sandia_hands/l_hand/joint_states";
static const char kTactileTopic[] = "/sandia_hands/l_hand/tactile_raw";
constexpr unsigned int kNumJoints = 12;

TEST(SandiaHandPluginJointsTest, DrivesJointAndReportsTactileContact)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/sandia_hand_full_test.sdf");

  fixture.Finalize();
  // Let the plugin's Load() run (creating its ROS interface) before we try
  // to talk to it.
  fixture.Server()->Run(true /*blocking*/, 50 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");

  std::mutex stateMutex;
  sensor_msgs::msg::JointState lastJointStates;
  bool sawJointStates = false;
  sandia_hand_msgs::msg::RawTactile lastTactile;
  bool sawTactile = false;

  auto jointStatesSub = testNode->create_subscription<sensor_msgs::msg::JointState>(
    kJointStatesTopic, 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(stateMutex);
      lastJointStates = *msg;
      sawJointStates = true;
    });
  auto tactileSub = testNode->create_subscription<sandia_hand_msgs::msg::RawTactile>(
    kTactileTopic, 10,
    [&](const sandia_hand_msgs::msg::RawTactile::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(stateMutex);
      lastTactile = *msg;
      sawTactile = true;
    });
  auto commandsPub = testNode->create_publisher<osrf_msgs::msg::JointCommands>(
    kJointCommandsTopic, 10);

  // Wait for the plugin's subscription to this topic to be discovered
  // before publishing -- otherwise a best-effort-QoS publish could go out
  // before anyone is listening.
  for (int attempt = 0;
    attempt < 100 && testNode->count_subscribers(kJointCommandsTopic) == 0;
    ++attempt)
  {
    rclcpp::spin_some(testNode);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(testNode->count_subscribers(kJointCommandsTopic), 0u);

  // Drive left_f0_j0 (index 0 in the plugin's joint list) toward 1.0 rad
  // with a real PID gain -- SetJointCommands() only overwrites the fields
  // that arrive at the exact expected size, so every array below has to
  // be the full 12 joints even though only index 0 is actually driven.
  osrf_msgs::msg::JointCommands cmd;
  cmd.position.assign(kNumJoints, 0.0);
  cmd.velocity.assign(kNumJoints, 0.0);
  cmd.effort.assign(kNumJoints, 0.0);
  cmd.kp_position.assign(kNumJoints, 0.0);
  cmd.ki_position.assign(kNumJoints, 0.0);
  cmd.kd_position.assign(kNumJoints, 0.0);
  cmd.kp_velocity.assign(kNumJoints, 0.0);
  cmd.i_effort_min.assign(kNumJoints, 0.0);
  cmd.i_effort_max.assign(kNumJoints, 0.0);
  cmd.position[0] = 1.0;
  cmd.kp_position[0] = 5.0;
  cmd.kd_position[0] = 0.2;

  for (int i = 0; i < 5; ++i) {
    commandsPub->publish(cmd);
    rclcpp::spin_some(testNode);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Give the PID plenty of sim steps to move the joint, and the tactile
  // pipeline plenty of steps to process the palm/obstacle contact that's
  // been overlapping since the very first step.
  fixture.Server()->Run(true /*blocking*/, 3000 /*iterations*/, false /*paused*/);

  double joint0Position = 0.0;
  bool tactileChanged = false;
  for (int attempt = 0; attempt < 100; ++attempt) {
    rclcpp::spin_some(testNode);
    {
      std::lock_guard<std::mutex> lock(stateMutex);
      if (sawJointStates && lastJointStates.position.size() > 0) {
        joint0Position = lastJointStates.position[0];
      }
      if (sawTactile) {
        for (const uint16_t value : lastTactile.palm) {
          if (value != 26500) {
            tactileChanged = true;
          }
        }
      }
    }
    if (sawJointStates && sawTactile && std::abs(joint0Position) > 0.05 &&
      tactileChanged)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(sawJointStates);
  ASSERT_TRUE(sawTactile);
  // The commanded PID force should have moved the joint measurably off
  // its starting position of 0 -- proves SetJointCommands(), the PID
  // computation, and Joint::SetForce() all actually worked end to end.
  EXPECT_GT(std::abs(joint0Position), 0.05);
  // The palm collision was configured to overlap the static obstacle
  // from the first sim step -- proves the force-created
  // ContactSensorData components and FillTactileData() actually worked.
  EXPECT_TRUE(tactileChanged);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
