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

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

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

TEST(MultiSenseSLPluginTest, StandsUpRosInterfaceAndSpinsSpindle)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/multisense_sl_test.sdf");

  fixture.Finalize();
  // The plugin's own rclcpp executor spins on its own thread throughout
  // this call, in real wall-clock time, independent of the sim-time steps
  // being run here -- by the time Run() returns there's been plenty of
  // real time for ROS graph discovery to complete.
  fixture.Server()->Run(true /*blocking*/, 200 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");

  // Default atlas_version (5) selects the "/multisense" namespace, not the
  // legacy "/multisense_sl" one.
  bool sawJointStatesPublisher = false;
  bool sawImuPublisher = false;
  for (int attempt = 0;
    attempt < 50 && !(sawJointStatesPublisher && sawImuPublisher);
    ++attempt)
  {
    sawJointStatesPublisher =
      testNode->count_publishers("/multisense/joint_states") > 0;
    sawImuPublisher = testNode->count_publishers("/multisense/imu") > 0;
    if (!(sawJointStatesPublisher && sawImuPublisher)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawJointStatesPublisher);
  EXPECT_TRUE(sawImuPublisher);

  double spindleVelocity = 0.0;
  bool gotJointState = false;
  auto jointStatesSub = testNode->create_subscription<sensor_msgs::msg::JointState>(
    "/multisense/joint_states", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr _msg)
    {
      if (!_msg->velocity.empty()) {
        spindleVelocity = _msg->velocity[0];
        gotJointState = true;
      }
    });

  auto speedPub = testNode->create_publisher<std_msgs::msg::Float64>(
    "/multisense/set_spindle_speed", 10);

  bool sawSpeedSubscriber = false;
  for (int attempt = 0; attempt < 100 && !sawSpeedSubscriber; ++attempt) {
    sawSpeedSubscriber =
      testNode->count_subscribers("/multisense/set_spindle_speed") > 0;
    if (!sawSpeedSubscriber) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawSpeedSubscriber);

  std_msgs::msg::Float64 speedMsg;
  speedMsg.data = 5.0;  // rad/s, well within the default 0-50 RPM PID range.
  for (int i = 0; i < 1000; ++i) {
    speedPub->publish(speedMsg);
    fixture.Server()->Run(true /*blocking*/, 1 /*iterations*/, false /*paused*/);
    rclcpp::spin_some(testNode);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  EXPECT_TRUE(gotJointState);
  EXPECT_GT(std::abs(spindleVelocity), 0.5);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
