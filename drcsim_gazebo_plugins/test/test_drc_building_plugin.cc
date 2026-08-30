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

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

// Point gz-sim at the just-built plugin .so before any TestFixture is
// constructed, so the test doesn't depend on this package's environment
// hook having been sourced (it hasn't -- this runs straight out of the
// build tree, before install).
namespace
{
  struct PluginPathSetter
  {
    PluginPathSetter()
  {
    setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", PLUGIN_BUILD_DIR, 1);
    }
  };
  PluginPathSetter g_pluginPathSetter;
}  // namespace

TEST(DRCBuildingPluginTest, DoorHoldsNearZeroUnderDefaultCommand)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/door_test.sdf");

  double doorPosition = 0.0;
  bool sawDoorJoint = false;

  fixture.OnPostUpdate(
    [&](const gz::sim::UpdateInfo &,
    const gz::sim::EntityComponentManager & _ecm)
  {
    gz::sim::World world(gz::sim::worldEntity(_ecm));
    gz::sim::Entity modelEntity =
    world.ModelByName(_ecm, "door_test_model");
    ASSERT_NE(modelEntity, gz::sim::kNullEntity);

    gz::sim::Model model(modelEntity);
    gz::sim::Entity doorJoint = model.JointByName(_ecm, "door_joint");
    ASSERT_NE(doorJoint, gz::sim::kNullEntity);

    auto position = gz::sim::Joint(doorJoint).Position(_ecm);
    if (position && !position->empty()) {
      doorPosition = (*position)[0];
      sawDoorJoint = true;
    }
      });

  fixture.Finalize();
  fixture.Server()->Run(true /*blocking*/, 1000 /*iterations*/,
      false /*paused*/);

  ASSERT_TRUE(sawDoorJoint);
  EXPECT_TRUE(std::isfinite(doorPosition));
  // doorCmd defaults to 0 and the door starts at 0; the PID controller
  // should hold it close to zero, not let it drift or blow up.
  EXPECT_NEAR(doorPosition, 0.0, 0.1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
