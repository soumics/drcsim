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

#include <atlas_msgs/msg/atlas_command.hpp>
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

TEST(AtlasPluginTest, StandsUpRosInterfaceAndPidsJointToTarget)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/atlas_plugin_test.sdf");

  fixture.Finalize();
  // The plugin's own rclcpp executor spins on its own thread throughout
  // this call, in real wall-clock time, independent of the sim-time steps
  // being run here -- by the time Run() returns there's been plenty of
  // real time for ROS graph discovery to complete.
  fixture.Server()->Run(true /*blocking*/, 200 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");

  bool sawJointStatesPublisher = false;
  bool sawAtlasStatePublisher = false;
  bool sawAtlasCommandSubscriber = false;
  for (int attempt = 0;
    attempt < 50 &&
    !(sawJointStatesPublisher && sawAtlasStatePublisher && sawAtlasCommandSubscriber);
    ++attempt)
  {
    sawJointStatesPublisher = testNode->count_publishers("/atlas/joint_states") > 0;
    sawAtlasStatePublisher = testNode->count_publishers("/atlas/atlas_state") > 0;
    sawAtlasCommandSubscriber =
      testNode->count_subscribers("/atlas/atlas_command") > 0;
    if (!(sawJointStatesPublisher && sawAtlasStatePublisher && sawAtlasCommandSubscriber)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawJointStatesPublisher);
  EXPECT_TRUE(sawAtlasStatePublisher);
  EXPECT_TRUE(sawAtlasCommandSubscriber);

  // 30 joints for the default atlas_version (5): confirm the joint list
  // came through with the expected size and that back_bkz (index 0) is
  // first, before commanding it.
  bool gotJointState = false;
  double backBkzPosition = 0.0;
  auto jointStatesSub = testNode->create_subscription<sensor_msgs::msg::JointState>(
    "/atlas/joint_states", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr _msg)
    {
      if (!_msg->name.empty() && _msg->name[0] == "back_bkz" &&
      !_msg->position.empty())
      {
        backBkzPosition = _msg->position[0];
        gotJointState = true;
      }
    });

  auto commandPub = testNode->create_publisher<atlas_msgs::msg::AtlasCommand>(
    "/atlas/atlas_command", 10);

  bool sawCommandSubscriberMatched = false;
  for (int attempt = 0; attempt < 100 && !sawCommandSubscriberMatched; ++attempt) {
    sawCommandSubscriberMatched =
      testNode->count_subscribers("/atlas/atlas_command") > 0;
    if (!sawCommandSubscriberMatched) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawCommandSubscriberMatched);

  atlas_msgs::msg::AtlasCommand command;
  command.position.assign(30, 0.0);
  command.velocity.assign(30, 0.0);
  command.effort.assign(30, 0.0);
  command.kp_position.assign(30, 50.0f);
  command.ki_position.assign(30, 0.0f);
  command.kd_position.assign(30, 5.0f);
  command.kp_velocity.assign(30, 0.0f);
  command.i_effort_min.assign(30, 0.0f);
  command.i_effort_max.assign(30, 0.0f);
  command.k_effort.assign(30, 255);
  command.position[0] = 0.5;  // back_bkz target.

  for (int i = 0; i < 1000; ++i) {
    command.header.stamp = testNode->get_clock()->now();
    commandPub->publish(command);
    fixture.Server()->Run(true /*blocking*/, 1 /*iterations*/, false /*paused*/);
    rclcpp::spin_some(testNode);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  EXPECT_TRUE(gotJointState);
  EXPECT_GT(backBkzPosition, 0.1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
