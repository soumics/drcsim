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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__HANDJOINTCONTROLLER_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__HANDJOINTCONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace drcsim_gazebo_ros_plugins
{

/// \brief Generic position controller for a multi-fingered hand.
///
/// New in this port (no Classic original): drives every revolute joint of
/// the model whose name starts with <joint_prefix> with a PD law toward a
/// target. Joints carrying a URDF/SDF <mimic> coupling (e.g. the SCHUNK SVH
/// hand's 20 joints driven by 9 motors) follow their leader's target times
/// the multiplier plus the offset, so only the leaders are commanded. This
/// replaces Classic's per-hand gazebo_ros_control + mimic-joint plugins
/// (which have no gz-sim equivalent here) and does not rely on the physics
/// engine supporting mimic constraints.
///
/// SDF: <joint_prefix>, <ros_namespace> (topics <ns>/command and
/// <ns>/joint_states), optional <kp> (N m/rad, default 3), <kd>
/// (N m s/rad, default 0.02), <max_effort> (N m, default 5; the joint's own
/// effort limit applies if lower).
///
/// ROS: <ns>/command (sensor_msgs/JointState; names with or without the
/// prefix, positions only) sets leader targets; <ns>/joint_states publishes
/// every hand joint at 50 Hz.
class HandJointController
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  HandJointController() = default;
  ~HandJointController() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

private:
  struct HandJoint
  {
    std::string name;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    double lower{0.0};
    double upper{0.0};
    double maxEffort{0.0};
    /// Index of the leader joint for a mimic joint, -1 for a leader.
    int leader{-1};
    double multiplier{1.0};
    double offset{0.0};
  };

  void OnCommand(const sensor_msgs::msg::JointState::SharedPtr _msg);

  gz::sim::Model model{gz::sim::kNullEntity};
  std::string prefix;
  std::string ns;
  double kp{3.0};
  double kd{0.02};
  double maxEffort{5.0};
  std::vector<HandJoint> joints;
  /// Commanded leader targets, indexed like `joints` (followers unused).
  std::vector<double> targets;
  std::mutex targetMutex;
  bool initialized{false};
  std::chrono::steady_clock::duration lastPublish{0};

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread rosSpinThread;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointStates;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subCommand;
};

}  // namespace drcsim_gazebo_ros_plugins

#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__HANDJOINTCONTROLLER_HPP_
