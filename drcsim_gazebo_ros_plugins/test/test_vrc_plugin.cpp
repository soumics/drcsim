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
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <rclcpp/rclcpp.hpp>

// Point gz-sim at the just-built plugin .so, enable VRCPlugin's cheats (off
// by default, gated by this env var, matching the original), and shorten
// atlas.time_to_unpin (default 5s -- too slow for a unit test) via the same
// DRCSIM_ROS_PARAMS_FILE mechanism a real launch file uses (see
// RosNodeOptions.hpp), all before any TestFixture is constructed. 1.0s
// safely exceeds StandsUpRosInterfaceAndAutoPinsOnStartup's 300-iteration
// (300ms) window below, so it stays unaffected by this global override.
struct PluginPathSetter
{
  PluginPathSetter()
  {
    setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", PLUGIN_BUILD_DIR, 1);
    setenv("VRC_CHEATS_ENABLED", "1", 1);

    char paramsPath[] = "/tmp/drcsim_vrc_plugin_test_params.XXXXXX";
    const int fd = mkstemp(paramsPath);
    if (fd >= 0) {
      close(fd);
      std::ofstream paramsFile(paramsPath);
      paramsFile << "vrc_plugin:\n  ros__parameters:\n"
        "    atlas.time_to_unpin: 1.0\n";
      paramsFile.close();
      setenv("DRCSIM_ROS_PARAMS_FILE", paramsPath, 1);
    }
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

TEST(VRCPluginTest, UnpinActuallyReleasesUtorsoToFallUnderGravity)
{
  // Regression test for a bug that survived unnoticed through three
  // different pin-to-world mechanisms (see VRCPlugin.hpp's class-level
  // design note): each one held the pin correctly, but silently failed to
  // ever truly release the link again once unpinned -- invisible to the
  // test above, which only ever checks that the pin holds, never that
  // unpinning actually works. This asserts the release itself.
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/vrc_plugin_test.sdf");

  double lastUtorsoZ = 0.0;
  double zJustAfterUnpin = -1.0;
  bool sawUtorso = false;
  fixture.OnPostUpdate(
    [&](const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
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

      // atlas.time_to_unpin is overridden to 1.0s (see PluginPathSetter);
      // capture a baseline Z shortly after that point, once, as the
      // "released, but gravity hasn't had time to move it much yet" mark.
      const double simTimeSec = std::chrono::duration<double>(_info.simTime).count();
      if (zJustAfterUnpin < 0.0 && simTimeSec >= 1.2) {
        zJustAfterUnpin = lastUtorsoZ;
      }
    });

  fixture.Finalize();
  // 2.5s of sim time at the test world's 1ms step size: well past the 1.0s
  // auto-unpin point, with 1.3+ more seconds for gravity to visibly move a
  // freely-falling body (there is no ground plane in this test world, and
  // nothing else supports utorso once its own pin/gravity-compensation
  // release, so a still-frozen link -- the exact bug this guards against
  // -- would keep lastUtorsoZ pinned at its spawn height instead).
  fixture.Server()->Run(true /*blocking*/, 2500 /*iterations*/, false /*paused*/);

  EXPECT_TRUE(sawUtorso);
  ASSERT_GE(zJustAfterUnpin, 0.0) << "never observed the post-unpin moment";
  EXPECT_LT(lastUtorsoZ, zJustAfterUnpin - 0.05);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
