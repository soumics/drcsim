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
#include <string>
#include <thread>

#include <gz/sim/TestFixture.hh>

#include <rclcpp/rclcpp.hpp>

#include <atlas_msgs/msg/s_model_robot_output.hpp>
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

TEST(RobotiqHandPluginTest, MissingJointsDoesNotCrashAndStandsUpNoRosInterface)
{
  // FindJoints() aborts the whole load the first time an expected joint is
  // missing (no stumps fallback, same as IRobotHandPlugin). This asserts
  // that failure path is graceful and leaves validConfig false.
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/robotiq_hand_missing_joints_test.sdf");

  fixture.Finalize();
  fixture.Server()->Run(true /*blocking*/, 200 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(testNode->count_publishers("/left_hand/state"), 0u);
  EXPECT_EQ(testNode->count_publishers("/robotiq_hands/left_hand/joint_states"), 0u);
}

TEST(RobotiqHandPluginTest, MovesFingerTowardCommandedPosition)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/robotiq_hand_full_test.sdf");

  fixture.Finalize();
  fixture.Server()->Run(true /*blocking*/, 200 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");

  bool sawHandleStatePublisher = false;
  bool sawJointStatesPublisher = false;
  for (int attempt = 0;
    attempt < 50 && !(sawHandleStatePublisher && sawJointStatesPublisher);
    ++attempt)
  {
    sawHandleStatePublisher = testNode->count_publishers("/left_hand/state") > 0;
    sawJointStatesPublisher =
      testNode->count_publishers("/robotiq_hands/left_hand/joint_states") > 0;
    if (!(sawHandleStatePublisher && sawJointStatesPublisher)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawHandleStatePublisher);
  EXPECT_TRUE(sawJointStatesPublisher);

  double firstJointPosition = 0.0;
  bool gotJointState = false;
  auto jointStatesSub = testNode->create_subscription<sensor_msgs::msg::JointState>(
    "/robotiq_hands/left_hand/joint_states", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr _msg)
    {
      if (!_msg->position.empty()) {
        firstJointPosition = _msg->position[0];
        gotJointState = true;
      }
    });

  auto commandPub = testNode->create_publisher<atlas_msgs::msg::SModelRobotOutput>(
    "/left_hand/command", 10);

  // Wait for DDS discovery to actually match this publisher with the
  // plugin's subscription before spending loop iterations on it below --
  // otherwise the first several dozen milliseconds of published commands
  // are silently dropped (no match yet) and the joint spends most of the
  // loop below sitting at handState == Disabled (zero commanded force).
  bool sawCommandSubscriber = false;
  for (int attempt = 0; attempt < 100 && !sawCommandSubscriber; ++attempt) {
    sawCommandSubscriber = testNode->count_subscribers("/left_hand/command") > 0;
    if (!sawCommandSubscriber) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawCommandSubscriber);

  // Wide grasping mode drives joint index 0 (l_palm_finger_1_joint) straight
  // to its upper position limit regardless of r_pra -- unlike the three
  // underactuated curl joints, this one is both the informative (PID
  // feedback) and the actuated joint (see the class-level design note on
  // the two joint vectors), so it moves correctly even in this test's
  // simplified, non-coupled finger linkage.
  atlas_msgs::msg::SModelRobotOutput command;
  command.r_act = 1;
  command.r_mod = 2;  // Wide.
  command.r_gto = 1;
  for (int i = 0; i < 1000; ++i) {
    commandPub->publish(command);
    fixture.Server()->Run(true /*blocking*/, 1 /*iterations*/, false /*paused*/);
    rclcpp::spin_some(testNode);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  EXPECT_TRUE(gotJointState);
  EXPECT_GT(std::abs(firstJointPosition), 0.05);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
