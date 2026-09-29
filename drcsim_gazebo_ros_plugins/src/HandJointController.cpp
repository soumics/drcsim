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
#include "drcsim_gazebo_ros_plugins/HandJointController.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Joint.hh>

#include "drcsim_gazebo_ros_plugins/RosNodeOptions.hpp"

using drcsim_gazebo_ros_plugins::HandJointController;

namespace
{
constexpr std::chrono::milliseconds kPublishPeriod{20};
}  // namespace

//////////////////////////////////////////////////
HandJointController::~HandJointController()
{
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->rosSpinThread.joinable()) {
    this->rosSpinThread.join();
  }
}

//////////////////////////////////////////////////
void HandJointController::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "HandJointController must be attached to a model." << std::endl;
    return;
  }
  if (!_sdf->HasElement("joint_prefix") || !_sdf->HasElement("ros_namespace")) {
    gzerr << "HandJointController needs <joint_prefix> and <ros_namespace>." << std::endl;
    return;
  }
  this->prefix = _sdf->Get<std::string>("joint_prefix");
  this->ns = _sdf->Get<std::string>("ros_namespace");
  this->kp = _sdf->Get<double>("kp", this->kp).first;
  this->kd = _sdf->Get<double>("kd", this->kd).first;
  this->maxEffort = _sdf->Get<double>("max_effort", this->maxEffort).first;

  // Every revolute joint under the prefix, with its limits and any mimic
  // coupling (leader resolved by name once all joints are known).
  std::unordered_map<std::string, int> indexOf;
  std::vector<std::string> leaderNames;
  for (const gz::sim::Entity jointEntity : this->model.Joints(_ecm)) {
    const auto * name = _ecm.Component<gz::sim::components::Name>(jointEntity);
    const auto * type = _ecm.Component<gz::sim::components::JointType>(jointEntity);
    const auto * axis = _ecm.Component<gz::sim::components::JointAxis>(jointEntity);
    if (!name || !type || !axis || name->Data().rfind(this->prefix, 0) != 0 ||
      type->Data() != sdf::JointType::REVOLUTE)
    {
      continue;
    }
    HandJoint joint;
    joint.name = name->Data();
    joint.entity = jointEntity;
    joint.lower = axis->Data().Lower();
    joint.upper = axis->Data().Upper();
    joint.maxEffort = std::min(this->maxEffort, std::abs(axis->Data().Effort()));
    std::string leaderName;
    if (const auto mimic = axis->Data().Mimic()) {
      leaderName = mimic->Joint();
      joint.multiplier = mimic->Multiplier();
      joint.offset = mimic->Offset();
    }
    indexOf[joint.name] = static_cast<int>(this->joints.size());
    this->joints.push_back(joint);
    leaderNames.push_back(leaderName);
    gz::sim::Joint(jointEntity).EnablePositionCheck(_ecm);
    gz::sim::Joint(jointEntity).EnableVelocityCheck(_ecm);
  }
  // Couplings may also be given in this plugin's own config as
  // <mimic joint="follower" leader="..." multiplier="..." offset="..."/>
  // (atlas.launch.py moves URDF <mimic> tags here, as DART has no mimic
  // constraints and gz-sim logs an error for each one it is handed).
  if (_sdf->HasElement("mimic")) {
    for (auto elem = std::const_pointer_cast<sdf::Element>(_sdf)->GetElement("mimic");
      elem; elem = elem->GetNextElement("mimic"))
    {
      const auto it = indexOf.find(elem->Get<std::string>("joint"));
      if (it == indexOf.end()) {
        continue;
      }
      leaderNames[it->second] = elem->Get<std::string>("leader");
      this->joints[it->second].multiplier = elem->Get<double>("multiplier", 1.0).first;
      this->joints[it->second].offset = elem->Get<double>("offset", 0.0).first;
    }
  }
  unsigned int followers = 0;
  for (size_t i = 0; i < this->joints.size(); ++i) {
    if (!leaderNames[i].empty()) {
      const auto it = indexOf.find(leaderNames[i]);
      if (it == indexOf.end()) {
        gzwarn << "HandJointController: mimic leader [" << leaderNames[i]
               << "] of [" << this->joints[i].name << "] not found; "
               << "controlling it directly." << std::endl;
      } else {
        this->joints[i].leader = it->second;
        ++followers;
      }
    }
  }
  if (this->joints.empty()) {
    gzerr << "HandJointController: no revolute joints start with ["
          << this->prefix << "]." << std::endl;
    return;
  }
  this->targets.assign(this->joints.size(), 0.0);
  gzmsg << "HandJointController [" << this->prefix << "]: " << this->joints.size()
        << " joints (" << this->joints.size() - followers << " actuated, " << followers
        << " mimic)." << std::endl;

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  std::string nodeName = "hand_joint_controller_" + this->prefix;
  nodeName.erase(
    std::remove_if(
      nodeName.begin(), nodeName.end(),
      [](char c) {return !std::isalnum(static_cast<unsigned char>(c)) && c != '_';}),
    nodeName.end());
  this->rosNode = std::make_shared<rclcpp::Node>(
    nodeName, drcsim_gazebo_ros_plugins::RosNodeOptionsFromEnv());
  this->pubJointStates = this->rosNode->create_publisher<sensor_msgs::msg::JointState>(
    this->ns + "/joint_states", 10);
  this->subCommand = this->rosNode->create_subscription<sensor_msgs::msg::JointState>(
    this->ns + "/command", 10,
    std::bind(&HandJointController::OnCommand, this, std::placeholders::_1));
  this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor->add_node(this->rosNode);
  this->rosSpinThread = std::thread([this]() {this->executor->spin();});
  this->initialized = true;
}

//////////////////////////////////////////////////
void HandJointController::OnCommand(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->targetMutex);
  for (size_t m = 0; m < _msg->name.size() && m < _msg->position.size(); ++m) {
    for (size_t i = 0; i < this->joints.size(); ++i) {
      const std::string & name = this->joints[i].name;
      if (this->joints[i].leader < 0 &&
        (name == _msg->name[m] || name == this->prefix + _msg->name[m]))
      {
        this->targets[i] = std::clamp(
          _msg->position[m], this->joints[i].lower, this->joints[i].upper);
      }
    }
  }
}

//////////////////////////////////////////////////
void HandJointController::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized || _info.paused) {
    return;
  }
  std::vector<double> leaderTargets;
  {
    std::lock_guard<std::mutex> lock(this->targetMutex);
    leaderTargets = this->targets;
  }
  sensor_msgs::msg::JointState state;
  const bool publish = _info.simTime - this->lastPublish >= kPublishPeriod;
  for (size_t i = 0; i < this->joints.size(); ++i) {
    const HandJoint & joint = this->joints[i];
    gz::sim::Joint handle(joint.entity);
    const auto position = handle.Position(_ecm);
    const auto velocity = handle.Velocity(_ecm);
    if (!position || position->empty() || !velocity || velocity->empty()) {
      continue;
    }
    double target = leaderTargets[i];
    if (joint.leader >= 0) {
      target = std::clamp(
        joint.multiplier * leaderTargets[joint.leader] + joint.offset, joint.lower, joint.upper);
    }
    const double effort = std::clamp(
      this->kp * (target - (*position)[0]) - this->kd * (*velocity)[0],
      -joint.maxEffort, joint.maxEffort);
    handle.SetForce(_ecm, {effort});
    if (publish) {
      state.name.push_back(joint.name);
      state.position.push_back((*position)[0]);
      state.velocity.push_back((*velocity)[0]);
      state.effort.push_back(effort);
    }
  }
  if (publish) {
    this->lastPublish = _info.simTime;
    state.header.stamp = rclcpp::Time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count());
    this->pubJointStates->publish(state);
  }
}

GZ_ADD_PLUGIN(
  HandJointController,
  gz::sim::System,
  HandJointController::ISystemConfigure,
  HandJointController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  HandJointController,
  "drcsim_gazebo_ros_plugins::HandJointController")
