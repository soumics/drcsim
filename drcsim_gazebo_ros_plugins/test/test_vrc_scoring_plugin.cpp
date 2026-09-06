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

#include <atlas_msgs/msg/vrc_score.hpp>

// Point gz-sim at the just-built plugin .so before any TestFixture is
// constructed, so the test doesn't depend on this package's environment
// hook having been sourced -- this runs straight out of the build tree,
// before install.
struct PluginPathSetter
{
  PluginPathSetter()
  {
    setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", PLUGIN_BUILD_DIR, 1);
  }
};
static PluginPathSetter g_pluginPathSetter;

TEST(VRCScoringPluginTest, ScoresGateCrossings)
{
  gz::sim::TestFixture fixture(
    std::string(TEST_WORLD_DIR) + "/vrc_scoring_plugin_test.sdf");

  // Drive the "atlas" model in a straight line through both gates by hand
  // -- gravity is zeroed out in the test world (see vrc_scoring_plugin_test
  // .sdf), so a direct SetWorldPoseCmd every tick is the only thing moving
  // it, with no real dynamics to fight.
  double currentX = -1.0;
  gz::sim::Entity atlasModelEntity = gz::sim::kNullEntity;
  fixture.OnPreUpdate(
    [&](const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & _ecm)
    {
      if (atlasModelEntity == gz::sim::kNullEntity) {
        gz::sim::World world(gz::sim::worldEntity(_ecm));
        atlasModelEntity = world.ModelByName(_ecm, "atlas");
        if (atlasModelEntity == gz::sim::kNullEntity) {
          return;
        }
      }
      currentX += 0.01;
      gz::sim::Model(atlasModelEntity).SetWorldPoseCmd(
        _ecm, gz::math::Pose3d(currentX, 0, 0, 0, 0, 0));
    });

  fixture.Finalize();
  // Atlas is present in the world from the start, so VRCScoringPlugin's
  // Configure() finds it immediately (see FindAtlas) -- 500 ticks at
  // 0.01/tick walks it from x=-1.0 to x=4.0, straight through gate_1
  // (x=0) and gate_2 (x=2).
  fixture.Server()->Run(true /*blocking*/, 500 /*iterations*/, false /*paused*/);

  rclcpp::Node::SharedPtr testNode = std::make_shared<rclcpp::Node>("test_observer");

  bool sawScorePublisher = false;
  for (int attempt = 0; attempt < 50 && !sawScorePublisher; ++attempt) {
    sawScorePublisher = testNode->count_publishers("/vrc_score") > 0;
    if (!sawScorePublisher) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(sawScorePublisher);

  int lastCompletionScore = -1;
  auto scoreSub = testNode->create_subscription<atlas_msgs::msg::VRCScore>(
    "vrc_score", rclcpp::QoS(1).transient_local(),
    [&](const atlas_msgs::msg::VRCScore::SharedPtr _msg)
    {
      lastCompletionScore = _msg->completion_score;
    });

  // The score topic is latched (transient_local), so the subscription
  // above should pick up the most recent score once discovery completes
  // -- both gates should already have been crossed by the 500 ticks run
  // above; keep spinning (and stepping, harmlessly, since there are no
  // more gates left to cross) until it arrives or we time out.
  for (int attempt = 0; attempt < 100 && lastCompletionScore < 2; ++attempt) {
    fixture.Server()->Run(true /*blocking*/, 1 /*iterations*/, false /*paused*/);
    rclcpp::spin_some(testNode);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(lastCompletionScore, 2);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
