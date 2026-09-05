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

#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <rclcpp/rclcpp.hpp>

// Point gz-sim at the just-built plugin .so, and enable VRCPlugin's cheats
// (off by default, gated by this env var, matching the original), before
// any TestFixture is constructed.
struct PluginPathSetter
{
  PluginPathSetter()
  {
    setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", PLUGIN_BUILD_DIR, 1);
    setenv("VRC_CHEATS_ENABLED", "1", 1);
  }
};
static PluginPathSetter g_pluginPathSetter;

TEST(VRCPluginTest, StandsUpRosInterfaceAndAutoPinsOnStartup)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/vrc_plugin_test.sdf");

  // Read utorso's world Z directly off the ECM every step, rather than via
  // the fake-ASIS ROS topic -- that topic only starts publishing once the
  // startup state machine reaches Robot::INITIALIZED, which (with the
  // default 5-second harness duration) is well past the point where the
  // robot is unpinned again and no longer expected to be near its spawn
  // height at all.
  double lastUtorsoZ = 0.0;
  bool sawUtorso = false;
  fixture.OnPostUpdate(
    [&](const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager & _ecm)
    {
      gz::sim::World world(gz::sim::worldEntity(_ecm));
      const gz::sim::Entity atlasModel = world.ModelByName(_ecm, "atlas");
      if (atlasModel == gz::sim::kNullEntity) {
        return;
      }
      const gz::sim::Entity utorso =
        gz::sim::Model(atlasModel).LinkByName(_ecm, "utorso");
      if (utorso == gz::sim::kNullEntity) {
        return;
      }
      lastUtorsoZ = gz::sim::worldPose(utorso, _ecm).Pos().Z();
      sawUtorso = true;
    });

  fixture.Finalize();
  // The plugin's own rclcpp executor spins on its own thread throughout
  // this call, in real wall-clock time, independent of the sim-time steps
  // being run here -- by the time Run() returns there's been plenty of
  // real time for both ROS graph discovery and the startup state machine
  // to walk from NONE through to the default "pinned" startup mode taking
  // hold (atlas.startup_mode defaults to empty, which the state machine
  // treats as the non-bdi_stand/"pinned" path), while staying well under
  // the 5-second default harness duration that would auto-unpin it again.
  fixture.Server()->Run(true /*blocking*/, 300 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");

  bool sawAtlasCommandPublisher = false;
  bool sawCmdVelSubscriber = false;
  for (int attempt = 0;
    attempt < 50 && !(sawAtlasCommandPublisher && sawCmdVelSubscriber);
    ++attempt)
  {
    sawAtlasCommandPublisher =
      testNode->count_publishers("/atlas/atlas_command") > 0;
    sawCmdVelSubscriber = testNode->count_subscribers("/atlas/cmd_vel") > 0;
    if (!(sawAtlasCommandPublisher && sawCmdVelSubscriber)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawAtlasCommandPublisher);
  EXPECT_TRUE(sawCmdVelSubscriber);

  // The default startup path pins the robot (gravity compensated + a real
  // world-pin joint) before it ever reaches "nominal" -- utorso started at
  // world Z=1 and, with the counter-gravity force and pin joint both
  // working, should still be almost exactly there 300ms later. Without
  // them (e.g. if AddJoint()/ApplyGravityCompensation() silently did
  // nothing), a 50kg free body would already have fallen several
  // centimeters under gravity in that time.
  EXPECT_TRUE(sawUtorso);
  EXPECT_NEAR(lastUtorsoZ, 1.0, 0.01);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
