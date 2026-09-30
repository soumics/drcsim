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

#include <cmath>
#include <cstdlib>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

// Point gz-sim at the just-built plugin .so (see test_drc_building_plugin.cc).
struct PluginPathSetter
{
  PluginPathSetter()
  {
    setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", PLUGIN_BUILD_DIR, 1);
  }
};
static PluginPathSetter g_pluginPathSetter;

TEST(FollowCameraPluginTest, FollowsPositionAndHeadingButNeverRollOrPitch)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/follow_camera_test.sdf");

  gz::math::Pose3d standingCamera;
  gz::math::Pose3d lyingCamera;
  fixture.OnPostUpdate(
    [&](const gz::sim::UpdateInfo &,
    const gz::sim::EntityComponentManager & _ecm)
    {
      gz::sim::World world(gz::sim::worldEntity(_ecm));
      standingCamera = gz::sim::worldPose(
        world.ModelByName(_ecm, "camera_standing"), _ecm);
      lyingCamera = gz::sim::worldPose(
        world.ModelByName(_ecm, "camera_lying"), _ecm);
    }).Finalize();
  fixture.Server()->Run(true, 200, false);

  // Standing target at (1, 2, 0.9), heading 0.5 rad, rolled and pitched:
  // the camera sits at the offset in the target's heading frame, level.
  const gz::math::Pose3d expected =
    gz::math::Pose3d(1, 2, 0.9, 0, 0, 0.5) * gz::math::Pose3d(3, -2, 0.3, 0, 0.06, 2.4);
  EXPECT_NEAR(standingCamera.Pos().X(), expected.Pos().X(), 1e-3);
  EXPECT_NEAR(standingCamera.Pos().Y(), expected.Pos().Y(), 1e-3);
  EXPECT_NEAR(standingCamera.Pos().Z(), expected.Pos().Z(), 1e-3);
  EXPECT_NEAR(standingCamera.Rot().Roll(), 0.0, 1e-3);
  EXPECT_NEAR(standingCamera.Rot().Pitch(), 0.06, 1e-3);
  EXPECT_NEAR(standingCamera.Rot().Yaw(), expected.Rot().Yaw(), 1e-3);

  // Lying on its back: no heading to follow, so the camera keeps heading 0
  // and stays level -- it does not look at the sky.
  EXPECT_NEAR(lyingCamera.Pos().X(), 0.0, 1e-3);
  EXPECT_NEAR(lyingCamera.Pos().Y(), 0.0, 1e-3);
  EXPECT_NEAR(lyingCamera.Rot().Roll(), 0.0, 1e-3);
  EXPECT_NEAR(lyingCamera.Rot().Pitch(), 0.0, 1e-3);
}
