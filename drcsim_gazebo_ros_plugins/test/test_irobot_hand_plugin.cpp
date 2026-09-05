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
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <gz/sim/TestFixture.hh>

#include <rclcpp/rclcpp.hpp>

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

TEST(IRobotHandPluginTest, MissingJointsDoesNotCrashAndStandsUpNoRosInterface)
{
  // Unlike SandiaHandPlugin, this plugin has no stumps fallback: if even one
  // of the 23 expected joints is missing, FindJoints() aborts the whole
  // load. This asserts that failure path is graceful (no crash) and leaves
  // validConfig false (no ROS publishers/subscriptions ever stand up).
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/irobot_hand_missing_joints_test.sdf");

  fixture.Finalize();
  fixture.Server()->Run(true /*blocking*/, 200 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(testNode->count_publishers("/irobot_hands/l_hand/joint_states"), 0u);
  EXPECT_EQ(testNode->count_publishers("/left_hand/sensors/raw"), 0u);
}

TEST(IRobotHandPluginTest, StandsUpRosInterfaceWithAllJointsPresent)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/irobot_hand_full_test.sdf");

  fixture.Finalize();
  // The plugin's own rclcpp executor spins on its own thread throughout
  // this call, in real wall-clock time, independent of the sim-time steps
  // being run here -- by the time Run() returns there's been plenty of
  // real time for ROS graph discovery to complete.
  fixture.Server()->Run(true /*blocking*/, 200 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");

  bool sawJointStatesPublisher = false;
  bool sawHandleSensorsPublisher = false;
  for (int attempt = 0;
    attempt < 50 && !(sawJointStatesPublisher && sawHandleSensorsPublisher);
    ++attempt)
  {
    sawJointStatesPublisher =
      testNode->count_publishers("/irobot_hands/l_hand/joint_states") > 0;
    sawHandleSensorsPublisher =
      testNode->count_publishers("/left_hand/sensors/raw") > 0;
    if (!(sawJointStatesPublisher && sawHandleSensorsPublisher)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  EXPECT_TRUE(sawJointStatesPublisher);
  EXPECT_TRUE(sawHandleSensorsPublisher);

  bool sawControlSubscription = false;
  for (int attempt = 0; attempt < 50 && !sawControlSubscription; ++attempt) {
    sawControlSubscription =
      testNode->count_subscribers("/left_hand/control") > 0;
    if (!sawControlSubscription) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawControlSubscription);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
