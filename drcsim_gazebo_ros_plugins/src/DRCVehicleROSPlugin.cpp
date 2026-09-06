/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "drcsim_gazebo_ros_plugins/DRCVehicleROSPlugin.hpp"

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>

using drcsim_gazebo_ros_plugins::DRCVehicleROSPlugin;

//////////////////////////////////////////////////
DRCVehicleROSPlugin::~DRCVehicleROSPlugin()
{
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->rosSpinThread.joinable()) {
    this->rosSpinThread.join();
  }
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & _eventMgr)
{
  // By default, cheats are off. Allow override via environment variable.
  const char * cheatsEnabledString = std::getenv("VRC_CHEATS_ENABLED");
  this->cheatsEnabled =
    cheatsEnabledString && std::string(cheatsEnabledString) == "1";

  DRCVehiclePlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  if (!this->IsValidConfig()) {
    gzerr << "DRCVehicleROSPlugin: error configuring base DRCVehiclePlugin. "
          << "Please ensure that your vehicle model is correct and "
          << "up-to-date." << std::endl;
    return;
  }

  if (!this->cheatsEnabled) {
    return;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->rosNode = std::make_shared<rclcpp::Node>("drc_vehicle_ros_plugin");

  const std::string modelName = gz::sim::Model(_entity).Name(_ecm);

  this->subHandWheelCmd = this->rosNode->create_subscription<std_msgs::msg::Float64>(
    modelName + "/hand_wheel/cmd", 100,
    std::bind(&DRCVehicleROSPlugin::SetHandWheelState, this, std::placeholders::_1));
  this->subHandBrakeCmd = this->rosNode->create_subscription<std_msgs::msg::Float64>(
    modelName + "/hand_brake/cmd", 100,
    std::bind(&DRCVehicleROSPlugin::SetHandBrakePercent, this, std::placeholders::_1));
  this->subGasPedalCmd = this->rosNode->create_subscription<std_msgs::msg::Float64>(
    modelName + "/gas_pedal/cmd", 100,
    std::bind(&DRCVehicleROSPlugin::SetGasPedalPercent, this, std::placeholders::_1));
  this->subBrakePedalCmd = this->rosNode->create_subscription<std_msgs::msg::Float64>(
    modelName + "/brake_pedal/cmd", 100,
    std::bind(&DRCVehicleROSPlugin::SetBrakePedalPercent, this, std::placeholders::_1));
  this->subKeyCmd = this->rosNode->create_subscription<std_msgs::msg::Int8>(
    modelName + "/key/cmd", 100,
    std::bind(&DRCVehicleROSPlugin::SetKeyState, this, std::placeholders::_1));
  this->subDirectionCmd = this->rosNode->create_subscription<std_msgs::msg::Int8>(
    modelName + "/direction/cmd", 100,
    std::bind(&DRCVehicleROSPlugin::SetDirectionState, this, std::placeholders::_1));

  this->pubHandWheelState = this->rosNode->create_publisher<std_msgs::msg::Float64>(
    modelName + "/hand_wheel/state", 10);
  this->pubHandBrakeState = this->rosNode->create_publisher<std_msgs::msg::Float64>(
    modelName + "/hand_brake/state", 10);
  this->pubGasPedalState = this->rosNode->create_publisher<std_msgs::msg::Float64>(
    modelName + "/gas_pedal/state", 10);
  this->pubBrakePedalState = this->rosNode->create_publisher<std_msgs::msg::Float64>(
    modelName + "/brake_pedal/state", 10);
  this->pubKeyState = this->rosNode->create_publisher<std_msgs::msg::Int8>(
    modelName + "/key/state", 10);
  this->pubDirectionState = this->rosNode->create_publisher<std_msgs::msg::Int8>(
    modelName + "/direction/state", 10);

  this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor->add_node(this->rosNode);
  this->rosSpinThread = std::thread(
    [this]()
    {
      this->executor->spin();
    });
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    DRCVehiclePlugin::PreUpdate(_info, _ecm);
  }

  if (this->cheatsEnabled) {
    this->RosPublishStates(_info.simTime);
  }
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::RosPublishStates(
  const std::chrono::steady_clock::duration & _simTime)
{
  if ((_simTime - this->lastRosPublishTime) < this->rosPublishPeriod) {
    return;
  }
  this->lastRosPublishTime = _simTime;

  std_msgs::msg::Float64 msgSteer;
  msgSteer.data = this->GetHandWheelState();
  this->pubHandWheelState->publish(msgSteer);

  std_msgs::msg::Float64 msgBrake;
  msgBrake.data = this->GetBrakePedalPercent();
  this->pubBrakePedalState->publish(msgBrake);

  std_msgs::msg::Float64 msgGas;
  msgGas.data = this->GetGasPedalPercent();
  this->pubGasPedalState->publish(msgGas);

  std_msgs::msg::Float64 msgHandBrake;
  msgHandBrake.data = this->GetHandBrakePercent();
  this->pubHandBrakeState->publish(msgHandBrake);

  std_msgs::msg::Int8 msgKey;
  msgKey.data = static_cast<int8_t>(this->GetKeyState());
  this->pubKeyState->publish(msgKey);

  std_msgs::msg::Int8 msgDirection;
  msgDirection.data = static_cast<int8_t>(this->GetDirectionState());
  this->pubDirectionState->publish(msgDirection);
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetKeyState(const std_msgs::msg::Int8::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  if (_msg->data == 0) {
    this->DRCVehiclePlugin::SetKeyOff();
  } else if (_msg->data == 1) {
    this->DRCVehiclePlugin::SetKeyOn();
  } else {
    RCLCPP_ERROR(
      this->rosNode->get_logger(), "Invalid Key State: %d, expected 0 or 1",
      static_cast<int>(_msg->data));
  }
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetDirectionState(const std_msgs::msg::Int8::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  if (_msg->data == 0) {
    this->DRCVehiclePlugin::SetDirectionState(DRCVehiclePlugin::NEUTRAL);
  } else if (_msg->data == 1) {
    this->DRCVehiclePlugin::SetDirectionState(DRCVehiclePlugin::FORWARD);
  } else if (_msg->data == -1) {
    this->DRCVehiclePlugin::SetDirectionState(DRCVehiclePlugin::REVERSE);
  } else {
    RCLCPP_ERROR(
      this->rosNode->get_logger(), "Invalid Direction State: %d, expected -1, 0, or 1",
      static_cast<int>(_msg->data));
  }

  // Matches the original: the FNR switch time is re-latched unconditionally,
  // even after an invalid message.
  this->UpdateFNRSwitchTime();
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetHandBrakePercent(const std_msgs::msg::Float64::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  const double percent = std::clamp(_msg->data, 0.0, 1.0);
  double min = 0.0;
  double max = 0.0;
  this->DRCVehiclePlugin::GetHandBrakeLimits(min, max);
  this->DRCVehiclePlugin::SetHandBrakeState(min + percent * (max - min));
  this->UpdateHandBrakeTime();
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetHandWheelState(const std_msgs::msg::Float64::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->DRCVehiclePlugin::SetHandWheelState(_msg->data);
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetGasPedalPercent(const std_msgs::msg::Float64::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  const double percent = std::clamp(_msg->data, 0.0, 1.0);
  double min = 0.0;
  double max = 0.0;
  this->DRCVehiclePlugin::GetGasPedalLimits(min, max);
  this->DRCVehiclePlugin::SetGasPedalState(min + percent * (max - min));
}

//////////////////////////////////////////////////
void DRCVehicleROSPlugin::SetBrakePedalPercent(const std_msgs::msg::Float64::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  const double percent = std::clamp(_msg->data, 0.0, 1.0);
  double min = 0.0;
  double max = 0.0;
  this->DRCVehiclePlugin::GetBrakePedalLimits(min, max);
  this->DRCVehiclePlugin::SetBrakePedalState(min + percent * (max - min));
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(
  DRCVehicleROSPlugin,
  gz::sim::System,
  DRCVehicleROSPlugin::ISystemConfigure,
  DRCVehicleROSPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DRCVehicleROSPlugin, "drcsim_gazebo_ros_plugins::DRCVehicleROSPlugin")
