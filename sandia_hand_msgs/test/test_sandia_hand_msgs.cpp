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

#include "sandia_hand_msgs/msg/cal_finger_state.hpp"
#include "sandia_hand_msgs/msg/parameter.hpp"
#include "sandia_hand_msgs/msg/raw_finger_commands.hpp"
#include "sandia_hand_msgs/msg/raw_finger_inertial.hpp"
#include "sandia_hand_msgs/msg/raw_finger_state.hpp"
#include "sandia_hand_msgs/msg/raw_mobo_state.hpp"
#include "sandia_hand_msgs/msg/raw_palm_state.hpp"
#include "sandia_hand_msgs/msg/raw_tactile.hpp"
#include "sandia_hand_msgs/msg/relative_joint_commands.hpp"
#include "sandia_hand_msgs/msg/simple_grasp.hpp"
#include "sandia_hand_msgs/srv/get_parameters.hpp"
#include "sandia_hand_msgs/srv/set_finger_home.hpp"
#include "sandia_hand_msgs/srv/set_joint_limit_policy.hpp"
#include "sandia_hand_msgs/srv/set_parameters.hpp"
#include "sandia_hand_msgs/srv/simple_grasp_srv.hpp"
#include "sandia_hand_msgs/srv/simple_grasp_with_slew.hpp"

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

TEST(SandiaHandMsgs, CalFingerStateRoundtrip)
{
  sandia_hand_msgs::msg::CalFingerState msg;
  msg.fmcb_time = 1.5;
  msg.pp_tactile = {1, 2, 3, 4, 5, 6};
  msg.hall_tgt = {10, 20, 30};
  msg.fmcb_effort = {-1, 0, 1};

  const auto out = roundtrip(msg);
  EXPECT_DOUBLE_EQ(out.fmcb_time, 1.5);
  EXPECT_FLOAT_EQ(out.pp_tactile[5], 6.0f);
  EXPECT_EQ(out.hall_tgt[1], 20);
  EXPECT_EQ(out.fmcb_effort[0], -1);
}

TEST(SandiaHandMsgs, ParameterRoundtripAndConstants)
{
  EXPECT_EQ(sandia_hand_msgs::msg::Parameter::INTEGER, 1);
  EXPECT_EQ(sandia_hand_msgs::msg::Parameter::FLOAT, 2);

  sandia_hand_msgs::msg::Parameter msg;
  msg.name = "kp";
  msg.val_type = sandia_hand_msgs::msg::Parameter::FLOAT;
  msg.f_val = 3.25f;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.name, "kp");
  EXPECT_EQ(out.val_type, sandia_hand_msgs::msg::Parameter::FLOAT);
  EXPECT_FLOAT_EQ(out.f_val, 3.25f);
}

TEST(SandiaHandMsgs, RawFingerCommandsRoundtrip)
{
  sandia_hand_msgs::msg::RawFingerCommands msg;
  msg.motor_targets = {100, -100, 0};

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.motor_targets[0], 100);
  EXPECT_EQ(out.motor_targets[1], -100);
}

TEST(SandiaHandMsgs, RawFingerInertialRoundtrip)
{
  sandia_hand_msgs::msg::RawFingerInertial msg;
  msg.mm_accel = {1, 2, 3};
  msg.dp_mag = {4, 5, 6};

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.mm_accel[2], 3);
  EXPECT_EQ(out.dp_mag[0], 4);
}

TEST(SandiaHandMsgs, RawFingerStateRoundtrip)
{
  sandia_hand_msgs::msg::RawFingerState msg;
  msg.fmcb_time = 123u;
  msg.pp_tactile = {1, 2, 3, 4, 5, 6};
  msg.hall_pos = {7, 8, 9};

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.fmcb_time, 123u);
  EXPECT_EQ(out.pp_tactile[0], 1);
  EXPECT_EQ(out.hall_pos[2], 9);
}

TEST(SandiaHandMsgs, RawMoboStateRoundtrip)
{
  sandia_hand_msgs::msg::RawMoboState msg;
  msg.mobo_time = 42u;
  msg.finger_currents = {0.1f, 0.2f, 0.3f, 0.4f};
  msg.mobo_max_effort = 200;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.mobo_time, 42u);
  EXPECT_FLOAT_EQ(out.finger_currents[3], 0.4f);
  EXPECT_EQ(out.mobo_max_effort, 200);
}

TEST(SandiaHandMsgs, RawPalmStateRoundtrip)
{
  sandia_hand_msgs::msg::RawPalmState msg;
  msg.palm_time = 7u;
  msg.palm_accel = {1, 2, 3};
  msg.palm_tactile[0] = 555;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.palm_time, 7u);
  EXPECT_EQ(out.palm_accel[1], 2);
  EXPECT_EQ(out.palm_tactile[0], 555);
}

TEST(SandiaHandMsgs, RawTactileRoundtrip)
{
  sandia_hand_msgs::msg::RawTactile msg;
  msg.header.frame_id = "sandia_hand";
  msg.f0 = {1, 2, 3};
  msg.palm = {9, 8, 7};

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "sandia_hand");
  ASSERT_EQ(out.f0.size(), 3u);
  EXPECT_EQ(out.f0[2], 3);
  ASSERT_EQ(out.palm.size(), 3u);
  EXPECT_EQ(out.palm[0], 9);
}

TEST(SandiaHandMsgs, RelativeJointCommandsRoundtrip)
{
  sandia_hand_msgs::msg::RelativeJointCommands msg;
  msg.header.frame_id = "sandia_hand";
  msg.position[0] = 0.5f;
  msg.max_effort[0] = 100;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "sandia_hand");
  EXPECT_FLOAT_EQ(out.position[0], 0.5f);
  EXPECT_EQ(out.max_effort[0], 100);
}

TEST(SandiaHandMsgs, SimpleGraspRoundtrip)
{
  sandia_hand_msgs::msg::SimpleGrasp msg;
  msg.name = "power_grasp";
  msg.closed_amount = 0.75;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.name, "power_grasp");
  EXPECT_DOUBLE_EQ(out.closed_amount, 0.75);
}

TEST(SandiaHandMsgs, GetParametersServiceRoundtrip)
{
  sandia_hand_msgs::srv::GetParameters::Response resp;
  sandia_hand_msgs::msg::Parameter p;
  p.name = "kd";
  resp.parameters.push_back(p);

  const auto out = roundtrip(resp);
  ASSERT_EQ(out.parameters.size(), 1u);
  EXPECT_EQ(out.parameters[0].name, "kd");
}

TEST(SandiaHandMsgs, SetFingerHomeServiceRoundtrip)
{
  sandia_hand_msgs::srv::SetFingerHome::Request req;
  req.finger_idx = 2;

  const auto out = roundtrip(req);
  EXPECT_EQ(out.finger_idx, 2);
}

TEST(SandiaHandMsgs, SetJointLimitPolicyServiceRoundtrip)
{
  sandia_hand_msgs::srv::SetJointLimitPolicy::Request req;
  req.policy = "strict";

  const auto out = roundtrip(req);
  EXPECT_EQ(out.policy, "strict");
}

TEST(SandiaHandMsgs, SetParametersServiceRoundtrip)
{
  sandia_hand_msgs::srv::SetParameters::Request req;
  sandia_hand_msgs::msg::Parameter p;
  p.name = "ki";
  req.parameters.push_back(p);

  const auto out = roundtrip(req);
  ASSERT_EQ(out.parameters.size(), 1u);
  EXPECT_EQ(out.parameters[0].name, "ki");
}

TEST(SandiaHandMsgs, SimpleGraspSrvServiceRoundtrip)
{
  sandia_hand_msgs::srv::SimpleGraspSrv::Request req;
  req.grasp.name = "pinch";
  req.grasp.closed_amount = 0.5;

  const auto out = roundtrip(req);
  EXPECT_EQ(out.grasp.name, "pinch");
  EXPECT_DOUBLE_EQ(out.grasp.closed_amount, 0.5);
}

TEST(SandiaHandMsgs, SimpleGraspWithSlewServiceRoundtrip)
{
  sandia_hand_msgs::srv::SimpleGraspWithSlew::Request req;
  req.grasp.name = "pinch";
  req.slew_duration = 2.0f;

  const auto out = roundtrip(req);
  EXPECT_EQ(out.grasp.name, "pinch");
  EXPECT_FLOAT_EQ(out.slew_duration, 2.0f);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
