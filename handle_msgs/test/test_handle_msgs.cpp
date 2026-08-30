#include <gtest/gtest.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "handle_msgs/msg/cable_tension.hpp"
#include "handle_msgs/msg/collision.hpp"
#include "handle_msgs/msg/finger.hpp"
#include "handle_msgs/msg/handle_collisions.hpp"
#include "handle_msgs/msg/handle_control.hpp"
#include "handle_msgs/msg/handle_sensors.hpp"
#include "handle_msgs/msg/handle_sensors_calibrated.hpp"

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

TEST(HandleMsgs, CableTensionRoundtrip)
{
  handle_msgs::msg::CableTension msg;
  msg.sensor1 = 1.5f;
  msg.sensor2 = -2.5f;

  const auto out = roundtrip(msg);
  EXPECT_FLOAT_EQ(out.sensor1, 1.5f);
  EXPECT_FLOAT_EQ(out.sensor2, -2.5f);
}

TEST(HandleMsgs, CollisionRoundtrip)
{
  handle_msgs::msg::Collision msg;
  msg.frame_id = "finger[0]/distal_link";
  msg.sensor_id = 7;
  msg.intensity = 0.42f;
  msg.x = 1.0f;
  msg.y = 2.0f;
  msg.z = 3.0f;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.frame_id, "finger[0]/distal_link");
  EXPECT_EQ(out.sensor_id, 7);
  EXPECT_FLOAT_EQ(out.intensity, 0.42f);
  EXPECT_FLOAT_EQ(out.x, 1.0f);
  EXPECT_FLOAT_EQ(out.y, 2.0f);
  EXPECT_FLOAT_EQ(out.z, 3.0f);
}

TEST(HandleMsgs, FingerRoundtrip)
{
  handle_msgs::msg::Finger msg;
  msg.proximal = {1.0f, 2.0f, 3.0f};
  msg.distal = {4.0f, 5.0f};

  const auto out = roundtrip(msg);
  ASSERT_EQ(out.proximal.size(), 3u);
  EXPECT_FLOAT_EQ(out.proximal[0], 1.0f);
  ASSERT_EQ(out.distal.size(), 2u);
  EXPECT_FLOAT_EQ(out.distal[1], 5.0f);
}

TEST(HandleMsgs, HandleCollisionsRoundtrip)
{
  handle_msgs::msg::HandleCollisions msg;
  msg.header.frame_id = "base_link";
  handle_msgs::msg::Collision c;
  c.sensor_id = 1;
  c.intensity = 9.0f;
  msg.collisions.push_back(c);

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "base_link");
  ASSERT_EQ(out.collisions.size(), 1u);
  EXPECT_EQ(out.collisions[0].sensor_id, 1);
  EXPECT_FLOAT_EQ(out.collisions[0].intensity, 9.0f);
}

TEST(HandleMsgs, HandleControlRoundtripAndConstants)
{
  EXPECT_EQ(handle_msgs::msg::HandleControl::VELOCITY, 1u);
  EXPECT_EQ(handle_msgs::msg::HandleControl::POSITION, 2u);
  EXPECT_EQ(handle_msgs::msg::HandleControl::CURRENT, 3u);
  EXPECT_EQ(handle_msgs::msg::HandleControl::VOLTAGE, 4u);
  EXPECT_EQ(handle_msgs::msg::HandleControl::ANGLE, 5u);

  handle_msgs::msg::HandleControl msg;
  msg.type[0] = handle_msgs::msg::HandleControl::POSITION;
  msg.value[0] = 1234;
  msg.valid[0] = true;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.type[0], handle_msgs::msg::HandleControl::POSITION);
  EXPECT_EQ(out.value[0], 1234);
  EXPECT_TRUE(out.valid[0]);
}

TEST(HandleMsgs, HandleSensorsRoundtrip)
{
  handle_msgs::msg::HandleSensors msg;
  msg.header.frame_id = "handle_hand";
  msg.motor_hall_encoder[0] = 3500;
  msg.air_temp = 21.5f;
  msg.finger_spread = 512;
  msg.finger_tactile[0].proximal = {1.0f, 2.0f};
  msg.proximal_acceleration[0].x = 0.1;
  msg.proximal_acceleration[0].y = 0.2;
  msg.proximal_acceleration[0].z = 0.3;
  msg.responses[0] = true;
  msg.response_history[0] = 100;
  msg.motor_error[0] = 0;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "handle_hand");
  EXPECT_EQ(out.motor_hall_encoder[0], 3500);
  EXPECT_FLOAT_EQ(out.air_temp, 21.5f);
  EXPECT_EQ(out.finger_spread, 512);
  ASSERT_EQ(out.finger_tactile[0].proximal.size(), 2u);
  EXPECT_FLOAT_EQ(out.finger_tactile[0].proximal[1], 2.0f);
  EXPECT_DOUBLE_EQ(out.proximal_acceleration[0].x, 0.1);
  EXPECT_TRUE(out.responses[0]);
  EXPECT_EQ(out.response_history[0], 100);
  EXPECT_EQ(out.motor_error[0], 0);
}

TEST(HandleMsgs, HandleSensorsCalibratedRoundtrip)
{
  handle_msgs::msg::HandleSensorsCalibrated msg;
  msg.header.frame_id = "handle_hand";
  msg.finger_spread = 0.75f;
  msg.proximal_joint_angle = {0.1f, 0.2f, 0.3f};
  msg.palm_tactile[0] = 5.0f;

  const auto out = roundtrip(msg);
  EXPECT_EQ(out.header.frame_id, "handle_hand");
  EXPECT_FLOAT_EQ(out.finger_spread, 0.75f);
  EXPECT_FLOAT_EQ(out.proximal_joint_angle[2], 0.3f);
  EXPECT_FLOAT_EQ(out.palm_tactile[0], 5.0f);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
