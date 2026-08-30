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

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "osrf_msgs/msg/joint_commands.hpp"

namespace
{

template<typename MessageT>
MessageT roundtrip(const MessageT & msg)
{
  rclcpp::Serialization<MessageT> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg, &serialized);

  MessageT out;
  serializer.deserialize_message(&serialized, &out);
  return out;
}

}  // namespace

TEST(OsrfMsgs, JointCommandsRoundtrip)
{
  osrf_msgs::msg::JointCommands msg;
  msg.header.frame_id = "atlas";
  msg.name = {"back_bkz", "back_bky"};
  msg.position = {0.1, 0.2};
  msg.velocity = {0.0, 0.0};
  msg.effort = {1.0, 2.0};
  msg.kp_position = {100.0, 200.0};
  msg.ki_position = {0.0, 0.0};
  msg.kd_position = {10.0, 20.0};
  msg.kp_velocity = {0.0, 0.0};
  msg.i_effort_min = {-1.0, -1.0};
  msg.i_effort_max = {1.0, 1.0};

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "atlas");
  ASSERT_EQ(out.name.size(), 2u);
  EXPECT_EQ(out.name[0], "back_bkz");
  EXPECT_EQ(out.name[1], "back_bky");
  ASSERT_EQ(out.position.size(), 2u);
  EXPECT_DOUBLE_EQ(out.position[1], 0.2);
  ASSERT_EQ(out.kp_position.size(), 2u);
  EXPECT_DOUBLE_EQ(out.kp_position[0], 100.0);
  ASSERT_EQ(out.i_effort_max.size(), 2u);
  EXPECT_DOUBLE_EQ(out.i_effort_max[0], 1.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
