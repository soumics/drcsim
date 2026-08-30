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
#include <vector>

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

double JointPosition(const gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _joint)
{
  auto position = gz::sim::Joint(_joint).Position(_ecm);
  return (position && !position->empty()) ? (*position)[0]
                                           : std::nan("");
}

}  // namespace

// Smoke test: with all default (zero/neutral) commands, the vehicle's
// joints should settle near their neutral positions and never diverge --
// this exercises Configure() successfully finding all 11 joints, reading
// their SDF limits, computing wheel radius/track width/wheelbase from
// collision geometry, and PreUpdate() running its PID control loop
// without producing NaNs or wild oscillation.
TEST(DRCVehiclePluginTest, SettlesNearNeutralUnderDefaultCommand)
{
  gz::sim::TestFixture fixture(
      std::string(TEST_WORLD_DIR) + "/vehicle_test.sdf");

  bool sawJoints = false;
  double steerPosition = 0.0;
  double gasPosition = 0.0;
  double flWheelPosition = 0.0;

  fixture.OnPostUpdate(
      [&](const gz::sim::UpdateInfo &,
          const gz::sim::EntityComponentManager &_ecm)
      {
        gz::sim::World world(gz::sim::worldEntity(_ecm));
        gz::sim::Entity modelEntity =
            world.ModelByName(_ecm, "vehicle_test_model");
        ASSERT_NE(modelEntity, gz::sim::kNullEntity);
        gz::sim::Model model(modelEntity);

        gz::sim::Entity steerJoint =
            model.JointByName(_ecm, "steering_wheel_joint");
        gz::sim::Entity gasJoint =
            model.JointByName(_ecm, "gas_pedal_joint");
        gz::sim::Entity flWheelJoint =
            model.JointByName(_ecm, "fl_wheel_joint");
        ASSERT_NE(steerJoint, gz::sim::kNullEntity);
        ASSERT_NE(gasJoint, gz::sim::kNullEntity);
        ASSERT_NE(flWheelJoint, gz::sim::kNullEntity);

        steerPosition = JointPosition(_ecm, steerJoint);
        gasPosition = JointPosition(_ecm, gasJoint);
        flWheelPosition = JointPosition(_ecm, flWheelJoint);
        sawJoints = true;
      });

  fixture.Finalize();
  fixture.Server()->Run(true /*blocking*/, 2000 /*iterations*/,
      false /*paused*/);

  ASSERT_TRUE(sawJoints);
  EXPECT_TRUE(std::isfinite(steerPosition));
  EXPECT_TRUE(std::isfinite(gasPosition));
  EXPECT_TRUE(std::isfinite(flWheelPosition));

  // Hand wheel PID should hold the steering wheel near its zero command.
  EXPECT_NEAR(steerPosition, 0.0, 0.2);
  // Gas pedal PID should hold the pedal near its low (released) limit.
  EXPECT_NEAR(gasPosition, 0.0, 0.02);
  // With ~0% gas and the vehicle unconstrained (no ground contact) in
  // this minimal world, the front-left wheel shouldn't be spinning up.
  EXPECT_LT(std::fabs(flWheelPosition), 5.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
