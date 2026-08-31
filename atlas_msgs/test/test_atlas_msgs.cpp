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

#include "atlas_msgs/msg/atlas_behavior_feedback.hpp"
#include "atlas_msgs/msg/atlas_behavior_manipulate_params.hpp"
#include "atlas_msgs/msg/atlas_behavior_pelvis_servo_params.hpp"
#include "atlas_msgs/msg/atlas_behavior_stand_params.hpp"
#include "atlas_msgs/msg/atlas_behavior_step_data.hpp"
#include "atlas_msgs/msg/atlas_behavior_step_params.hpp"
#include "atlas_msgs/msg/atlas_behavior_walk_params.hpp"
#include "atlas_msgs/msg/atlas_command.hpp"
#include "atlas_msgs/msg/atlas_position_data.hpp"
#include "atlas_msgs/msg/atlas_sim_interface_command.hpp"
#include "atlas_msgs/msg/atlas_sim_interface_state.hpp"
#include "atlas_msgs/msg/atlas_state.hpp"
#include "atlas_msgs/msg/controller_statistics.hpp"
#include "atlas_msgs/msg/force_torque_sensors.hpp"
#include "atlas_msgs/msg/s_model_robot_input.hpp"
#include "atlas_msgs/msg/s_model_robot_output.hpp"
#include "atlas_msgs/msg/synchronization_statistics.hpp"
#include "atlas_msgs/msg/test.hpp"
#include "atlas_msgs/msg/vrc_score.hpp"
#include "atlas_msgs/srv/atlas_filters.hpp"
#include "atlas_msgs/srv/get_joint_damping.hpp"
#include "atlas_msgs/srv/reset_controls.hpp"
#include "atlas_msgs/srv/set_joint_damping.hpp"
#include "atlas_msgs/action/walk_demo.hpp"

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

TEST(AtlasMsgs, AtlasCommandRoundtrip)
{
  atlas_msgs::msg::AtlasCommand msg;
  msg.header.frame_id = "atlas";
  msg.position = {0.1, 0.2, 0.3};
  msg.k_effort = {0, 128, 255};
  msg.desired_controller_period_ms = 1;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "atlas");
  ASSERT_EQ(out.position.size(), 3u);
  EXPECT_DOUBLE_EQ(out.position[1], 0.2);
  EXPECT_EQ(out.k_effort[2], 255);
  EXPECT_EQ(out.desired_controller_period_ms, 1);
}

TEST(AtlasMsgs, AtlasStateRoundtrip)
{
  atlas_msgs::msg::AtlasState msg;
  msg.header.frame_id = "atlas";
  msg.position = {1.0f, 2.0f};
  msg.l_foot.force.z = 100.0;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "atlas");
  EXPECT_FLOAT_EQ(out.position[0], 1.0f);
  EXPECT_DOUBLE_EQ(out.l_foot.force.z, 100.0);
  EXPECT_EQ(atlas_msgs::msg::AtlasState::L_LEG_HPY, 6);
}

TEST(AtlasMsgs, ForceTorqueSensorsRoundtrip)
{
  atlas_msgs::msg::ForceTorqueSensors msg;
  msg.header.frame_id = "atlas";
  msg.r_hand.torque.x = 1.5;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "atlas");
  EXPECT_DOUBLE_EQ(out.r_hand.torque.x, 1.5);
}

TEST(AtlasMsgs, ControllerStatisticsRoundtrip)
{
  atlas_msgs::msg::ControllerStatistics msg;
  msg.header.frame_id = "atlas";
  msg.command_age = 0.01;
  msg.command_age_mean = 0.02;

  const auto out = roundtrip(msg);
  EXPECT_DOUBLE_EQ(out.command_age, 0.01);
  EXPECT_DOUBLE_EQ(out.command_age_mean, 0.02);
}

TEST(AtlasMsgs, SynchronizationStatisticsRoundtrip)
{
  atlas_msgs::msg::SynchronizationStatistics msg;
  msg.delay_in_step = 0.001;
  msg.delay_in_window = 0.05;
  msg.delay_window_remain = 0.1;

  const auto out = roundtrip(msg);
  EXPECT_DOUBLE_EQ(out.delay_in_step, 0.001);
  EXPECT_DOUBLE_EQ(out.delay_window_remain, 0.1);
}

TEST(AtlasMsgs, TestMsgRoundtrip)
{
  atlas_msgs::msg::Test msg;
  msg.header.frame_id = "atlas";
  msg.damping = {1.0f, 2.0f};
  msg.k_effort = {255};

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "atlas");
  EXPECT_FLOAT_EQ(out.damping[1], 2.0f);
  EXPECT_EQ(out.k_effort[0], 255);
}

TEST(AtlasMsgs, VRCScoreRoundtripAndConstants)
{
  EXPECT_EQ(atlas_msgs::msg::VRCScore::TASK_DRIVING, 1u);
  EXPECT_EQ(atlas_msgs::msg::VRCScore::TASK_MANIPULATION, 3u);

  atlas_msgs::msg::VRCScore msg;
  msg.completion_score = 4;
  msg.falls = 1;
  msg.message = "gate passed";
  msg.task_type = atlas_msgs::msg::VRCScore::TASK_WALKING;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.completion_score, 4);
  EXPECT_EQ(out.message, "gate passed");
  EXPECT_EQ(out.task_type, atlas_msgs::msg::VRCScore::TASK_WALKING);
}

TEST(AtlasMsgs, SModelRobotInputRoundtrip)
{
  atlas_msgs::msg::SModelRobotInput msg;
  msg.g_act = 1;
  msg.g_sta = 3;
  msg.g_poa = 200;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.g_act, 1);
  EXPECT_EQ(out.g_sta, 3);
  EXPECT_EQ(out.g_poa, 200);
}

TEST(AtlasMsgs, SModelRobotOutputRoundtrip)
{
  atlas_msgs::msg::SModelRobotOutput msg;
  msg.r_act = 1;
  msg.r_gto = 1;
  msg.r_pra = 255;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.r_act, 1);
  EXPECT_EQ(out.r_gto, 1);
  EXPECT_EQ(out.r_pra, 255);
}

TEST(AtlasMsgs, AtlasBehaviorStepDataRoundtrip)
{
  atlas_msgs::msg::AtlasBehaviorStepData msg;
  msg.step_index = 1;
  msg.foot_index = 0;
  msg.duration = 0.63;
  msg.pose.position.z = 0.1;
  msg.swing_height = 0.05;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.step_index, 1u);
  EXPECT_DOUBLE_EQ(out.duration, 0.63);
  EXPECT_DOUBLE_EQ(out.pose.position.z, 0.1);
}

TEST(AtlasMsgs, AtlasBehaviorWalkParamsRoundtrip)
{
  atlas_msgs::msg::AtlasBehaviorWalkParams msg;
  msg.use_demo_walk = true;
  msg.step_queue[0].step_index = 1;
  msg.step_queue[3].step_index = 4;

  const auto out = roundtrip(msg);
  EXPECT_TRUE(out.use_demo_walk);
  ASSERT_EQ(out.step_queue.size(), 4u);
  EXPECT_EQ(out.step_queue[0].step_index, 1u);
  EXPECT_EQ(out.step_queue[3].step_index, 4u);
}

TEST(AtlasMsgs, AtlasBehaviorStepParamsRoundtrip)
{
  atlas_msgs::msg::AtlasBehaviorStepParams msg;
  msg.use_demo_walk = false;
  msg.desired_step.step_index = 2;

  const auto out = roundtrip(msg);
  EXPECT_FALSE(out.use_demo_walk);
  EXPECT_EQ(out.desired_step.step_index, 2u);
}

TEST(AtlasMsgs, AtlasBehaviorManipulateParamsRoundtrip)
{
  atlas_msgs::msg::AtlasBehaviorManipulateParams msg;
  msg.use_desired = true;
  msg.use_demo_mode = false;
  msg.desired.pelvis_height = 0.9;
  msg.desired.pelvis_yaw = 0.1;

  const auto out = roundtrip(msg);
  EXPECT_TRUE(out.use_desired);
  EXPECT_DOUBLE_EQ(out.desired.pelvis_height, 0.9);
}

TEST(AtlasMsgs, AtlasBehaviorFeedbackRoundtripAndConstants)
{
  EXPECT_EQ(atlas_msgs::msg::AtlasBehaviorFeedback::STATUS_TRANSITION_SUCCESS, 2u);
  EXPECT_EQ(atlas_msgs::msg::AtlasBehaviorFeedback::STATUS_ERROR_FALLING, 256u);

  atlas_msgs::msg::AtlasBehaviorFeedback msg;
  msg.status_flags = atlas_msgs::msg::AtlasBehaviorFeedback::STATUS_TRANSITION_SUCCESS;
  msg.trans_from_behavior_index = 1;
  msg.trans_to_behavior_index = 2;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.status_flags, atlas_msgs::msg::AtlasBehaviorFeedback::STATUS_TRANSITION_SUCCESS);
  EXPECT_EQ(out.trans_to_behavior_index, 2);
}

TEST(AtlasMsgs, AtlasSimInterfaceCommandRoundtripAndConstants)
{
  EXPECT_EQ(atlas_msgs::msg::AtlasSimInterfaceCommand::WALK, 4);
  EXPECT_EQ(atlas_msgs::msg::AtlasSimInterfaceCommand::USER, 7);

  atlas_msgs::msg::AtlasSimInterfaceCommand msg;
  msg.header.frame_id = "atlas";
  msg.behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::WALK;
  msg.walk_params.use_demo_walk = true;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "atlas");
  EXPECT_EQ(out.behavior, atlas_msgs::msg::AtlasSimInterfaceCommand::WALK);
  EXPECT_TRUE(out.walk_params.use_demo_walk);
}

TEST(AtlasMsgs, AtlasSimInterfaceStateRoundtripAndConstants)
{
  EXPECT_EQ(atlas_msgs::msg::AtlasSimInterfaceState::NO_ERRORS, 0);
  EXPECT_EQ(atlas_msgs::msg::AtlasSimInterfaceState::ERROR_NO_SUCH_BEHAVIOR, -6);

  atlas_msgs::msg::AtlasSimInterfaceState msg;
  msg.header.frame_id = "atlas";
  msg.error_code = atlas_msgs::msg::AtlasSimInterfaceState::NO_ERRORS;
  msg.foot_pos_est[0].position.z = 0.05;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "atlas");
  EXPECT_EQ(out.error_code, atlas_msgs::msg::AtlasSimInterfaceState::NO_ERRORS);
  ASSERT_EQ(out.foot_pos_est.size(), 2u);
  EXPECT_DOUBLE_EQ(out.foot_pos_est[0].position.z, 0.05);
}

TEST(AtlasMsgs, AtlasFiltersServiceRoundtrip)
{
  atlas_msgs::srv::AtlasFilters::Request req;
  req.coef_a = {1.0, 0.5};
  req.filter_velocity = true;

  const auto out = roundtrip(req);
  ASSERT_EQ(out.coef_a.size(), 2u);
  EXPECT_DOUBLE_EQ(out.coef_a[1], 0.5);
  EXPECT_TRUE(out.filter_velocity);

  atlas_msgs::srv::AtlasFilters::Response resp;
  resp.success = true;
  resp.status_message = "ok";

  const auto out_resp = roundtrip(resp);
  EXPECT_TRUE(out_resp.success);
  EXPECT_EQ(out_resp.status_message, "ok");
}

TEST(AtlasMsgs, GetJointDampingServiceRoundtrip)
{
  atlas_msgs::srv::GetJointDamping::Response resp;
  resp.damping_coefficients[0] = 1.0;
  resp.damping_coefficients_min[0] = 0.0;
  resp.damping_coefficients_max[0] = 5.0;
  resp.success = true;

  const auto out = roundtrip(resp);
  EXPECT_DOUBLE_EQ(out.damping_coefficients[0], 1.0);
  EXPECT_DOUBLE_EQ(out.damping_coefficients_max[0], 5.0);
  EXPECT_TRUE(out.success);
}

TEST(AtlasMsgs, ResetControlsServiceRoundtrip)
{
  atlas_msgs::srv::ResetControls::Request req;
  req.reset_bdi_controller = true;
  req.reload_pid_from_ros = false;
  req.atlas_command.desired_controller_period_ms = 1;

  const auto out = roundtrip(req);
  EXPECT_TRUE(out.reset_bdi_controller);
  EXPECT_FALSE(out.reload_pid_from_ros);
  EXPECT_EQ(out.atlas_command.desired_controller_period_ms, 1);
}

TEST(AtlasMsgs, SetJointDampingServiceRoundtrip)
{
  atlas_msgs::srv::SetJointDamping::Request req;
  req.damping_coefficients[5] = 2.5;

  const auto out = roundtrip(req);
  EXPECT_DOUBLE_EQ(out.damping_coefficients[5], 2.5);
}

TEST(AtlasMsgs, WalkDemoActionGoalRoundtrip)
{
  atlas_msgs::action::WalkDemo::Goal goal;
  goal.header.frame_id = "atlas";
  goal.behavior = atlas_msgs::action::WalkDemo::Goal::WALK;
  goal.steps.resize(1);
  goal.steps[0].step_index = 1;

  const auto out = roundtrip(goal);
  EXPECT_EQ(out.header.frame_id, "atlas");
  EXPECT_EQ(out.behavior, atlas_msgs::action::WalkDemo::Goal::WALK);
  ASSERT_EQ(out.steps.size(), 1u);
  EXPECT_EQ(out.steps[0].step_index, 1u);
}

TEST(AtlasMsgs, WalkDemoActionResultAndFeedbackRoundtrip)
{
  atlas_msgs::action::WalkDemo::Result result;
  result.success = true;
  result.end_state.error_code = atlas_msgs::msg::AtlasSimInterfaceState::NO_ERRORS;

  const auto out_result = roundtrip(result);
  EXPECT_TRUE(out_result.success);
  EXPECT_EQ(out_result.end_state.error_code, atlas_msgs::msg::AtlasSimInterfaceState::NO_ERRORS);

  atlas_msgs::action::WalkDemo::Feedback feedback;
  feedback.state.error_code = atlas_msgs::msg::AtlasSimInterfaceState::ERROR_UNSPECIFIED;

  const auto out_feedback = roundtrip(feedback);
  EXPECT_EQ(out_feedback.state.error_code,
    atlas_msgs::msg::AtlasSimInterfaceState::ERROR_UNSPECIFIED);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
