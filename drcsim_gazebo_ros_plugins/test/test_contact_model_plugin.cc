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

#include <cstdlib>
#include <string>

#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>

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

// Factored out to file scope: uncrustify misparses this exact templated
// EntityComponentManager::Component<T>() call as a comparison expression
// (wants spaces around < and >, which cpplint then rejects) when it's
// nested several braces deep inside a lambda; at file scope it parses fine.
static const gz::sim::components::ContactSensorData * GetContactSensorData(
  const gz::sim::EntityComponentManager & _ecm,
  gz::sim::Entity _collisionEntity)
{
  const auto * contacts =
    _ecm.Component<gz::sim::components::ContactSensorData>(_collisionEntity);
  return contacts;
}

TEST(ContactModelPluginTest, DetectsContactOnConfiguredCollision)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/contact_test.sdf");

  bool sawContactComponent = false;
  bool sawNonEmptyContact = false;

  fixture.OnPostUpdate(
    [&](const gz::sim::UpdateInfo &,
    const gz::sim::EntityComponentManager & _ecm)
  {
    gz::sim::World world(gz::sim::worldEntity(_ecm));
    gz::sim::Entity modelEntity =
    world.ModelByName(_ecm, "contact_test_model");
    if (modelEntity == gz::sim::kNullEntity) {
      return;
    }

    gz::sim::Entity linkEntity =
    gz::sim::Model(modelEntity).LinkByName(_ecm, "box_link");
    if (linkEntity == gz::sim::kNullEntity) {
      return;
    }

    auto collisionEntities = _ecm.ChildrenByComponents(
      linkEntity, gz::sim::components::Collision());
    if (collisionEntities.empty()) {
      return;
    }

    const auto * contacts = GetContactSensorData(_ecm, collisionEntities[0]);
    if (contacts) {
      sawContactComponent = true;
      if (!contacts->Data().contact().empty()) {
        sawNonEmptyContact = true;
      }
    }
      });

  fixture.Finalize();
  fixture.Server()->Run(true /*blocking*/, 1000 /*iterations*/,
      false /*paused*/);

  // The plugin should have force-created a ContactSensorData component on
  // the configured collision (box_link::box_collision), matching the
  // original Classic ContactManager::CreateFilter behavior.
  ASSERT_TRUE(sawContactComponent);
  // The two boxes are configured to overlap from the very first step, so
  // the physics engine should have reported at least one contact.
  EXPECT_TRUE(sawNonEmptyContact);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
