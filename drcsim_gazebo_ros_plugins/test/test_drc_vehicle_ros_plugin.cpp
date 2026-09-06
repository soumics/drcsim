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

#include <std_msgs/msg/float64.hpp>

// Point gz-sim at the just-built plugin .so, and enable VRC cheats (this
// plugin's whole ROS interface is gated behind that env var, matching the
// original), before any TestFixture is constructed.
struct PluginPathSetter
{
  PluginPathSetter()
  {
    setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", PLUGIN_BUILD_DIR, 1);
    setenv("VRC_CHEATS_ENABLED", "1", 1);
  }
};
static PluginPathSetter g_pluginPathSetter;

TEST(DRCVehicleROSPluginTest, GasPedalCommandMovesStateTowardFull)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/drc_vehicle_ros_plugin_test.sdf");

  fixture.Finalize();
  // The plugin's own rclcpp executor spins on its own thread throughout
  // this call, in real wall-clock time, independent of the sim-time steps
  // being run here -- by the time Run() returns there's been plenty of
  // real time for ROS graph discovery to complete.
  fixture.Server()->Run(true /*blocking*/, 200 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");

  bool sawStatePublisher = false;
  bool sawCmdSubscriber = false;
  for (int attempt = 0;
    attempt < 50 && !(sawStatePublisher && sawCmdSubscriber);
    ++attempt)
  {
    sawStatePublisher =
      testNode->count_publishers("/vehicle_test_model/gas_pedal/state") > 0;
    sawCmdSubscriber =
      testNode->count_subscribers("/vehicle_test_model/gas_pedal/cmd") > 0;
    if (!(sawStatePublisher && sawCmdSubscriber)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawStatePublisher);
  EXPECT_TRUE(sawCmdSubscriber);

  double lastGasPercent = -1.0;
  auto stateSub = testNode->create_subscription<std_msgs::msg::Float64>(
    "/vehicle_test_model/gas_pedal/state", 10,
    [&](const std_msgs::msg::Float64::SharedPtr _msg)
    {
      lastGasPercent = _msg->data;
    });

  auto cmdPub = testNode->create_publisher<std_msgs::msg::Float64>(
    "/vehicle_test_model/gas_pedal/cmd", 10);

  std_msgs::msg::Float64 fullGas;
  fullGas.data = 1.0;

  for (int i = 0; i < 500; ++i) {
    cmdPub->publish(fullGas);
    fixture.Server()->Run(true /*blocking*/, 1 /*iterations*/, false /*paused*/);
    rclcpp::spin_some(testNode);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  // The gas pedal's PID should have driven it close to its upper (fully
  // pressed) limit, reported as gas_pedal/state approaching 1.0.
  EXPECT_GT(lastGasPercent, 0.8);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
