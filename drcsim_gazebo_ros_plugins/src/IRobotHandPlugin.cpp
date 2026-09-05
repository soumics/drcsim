/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include "drcsim_gazebo_ros_plugins/IRobotHandPlugin.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <sdf/JointAxis.hh>

using drcsim_gazebo_ros_plugins::IRobotHandPlugin;

//////////////////////////////////////////////////
IRobotHandPlugin::IRobotHandPlugin()
{
  this->errorTerms.resize(5);
  this->flexureFlexJoints.resize(kNumFingers);
  this->flexureTwistJoints.resize(kNumFingers);
  this->flexureFlexSprings.resize(kNumFingers);
  this->flexureTwistSprings.resize(kNumFingers);
}

//////////////////////////////////////////////////
IRobotHandPlugin::~IRobotHandPlugin()
{
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->rosSpinThread.joinable()) {
    this->rosSpinThread.join();
  }
}

//////////////////////////////////////////////////
void IRobotHandPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "IRobotHandPlugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->sdfConfig = _sdf;
}

//////////////////////////////////////////////////
void IRobotHandPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized && this->sdfConfig) {
    // Deferred from Configure(): a model's child joint entities are not
    // guaranteed to exist yet when Configure() runs.
    this->Load(_ecm);
    this->initialized = true;
  }

  if (this->validConfig) {
    this->UpdateStates(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void IRobotHandPlugin::Load(gz::sim::EntityComponentManager & _ecm)
{
  if (!this->sdfConfig->HasElement("side")) {
    gzerr << "Failed to determine which hand we're controlling; "
          << "aborting plugin load." << std::endl;
    return;
  }
  this->side = this->sdfConfig->Get<std::string>("side");
  if (this->side != "left" && this->side != "right") {
    gzerr << "Failed to determine which hand we're controlling; "
          << "aborting plugin load." << std::endl;
    return;
  }

  gzmsg << "IRobotHandPlugin loading for " << this->side << " hand."
        << std::endl;

  if (!this->FindJoints(_ecm)) {
    return;
  }

  this->SetJointSpringParams();

  gz::sim::Joint thumbBaseJoint(this->fingerBaseJoints[2]);
  const auto thumbAxes = thumbBaseJoint.Axis(_ecm);
  if (!thumbAxes || thumbAxes->empty()) {
    gzerr << "Could not read thumb base joint limits; aborting plugin load."
          << std::endl;
    return;
  }
  this->thumbUpperLimit = (*thumbAxes)[0].Upper();
  this->thumbLowerLimit = (*thumbAxes)[0].Lower();
  this->thumbAntagonistAngle = 0.0;

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->rosNode = std::make_shared<rclcpp::Node>(
    "irobot_hand_plugin_" + this->side);

  this->pubJointStates =
    this->rosNode->create_publisher<sensor_msgs::msg::JointState>(
    "irobot_hands/" + this->side.substr(0, 1) + "_hand/joint_states", 10);

  this->pubHandleState =
    this->rosNode->create_publisher<handle_msgs::msg::HandleSensors>(
    this->side + "_hand/sensors/raw", 100);

  this->subHandleCommand =
    this->rosNode->create_subscription<handle_msgs::msg::HandleControl>(
    this->side + "_hand/control", 100,
    std::bind(&IRobotHandPlugin::SetHandleCommand, this, std::placeholders::_1));

  this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor->add_node(this->rosNode);
  this->rosSpinThread = std::thread(
    [this]()
    {
      this->executor->spin();
    });

  this->validConfig = true;
}

//////////////////////////////////////////////////
bool IRobotHandPlugin::GetAndPushBackJoint(
  gz::sim::EntityComponentManager & _ecm,
  const std::string & _jointName,
  std::vector<gz::sim::Entity> & _joints)
{
  const gz::sim::Entity joint = this->model.JointByName(_ecm, _jointName);
  if (joint == gz::sim::kNullEntity) {
    gzerr << "Failed to find joint: " << _jointName
          << "; aborting plugin load." << std::endl;
    return false;
  }
  gz::sim::Joint jointWrapper(joint);
  jointWrapper.EnablePositionCheck(_ecm);
  jointWrapper.EnableVelocityCheck(_ecm);
  _joints.push_back(joint);
  gzmsg << "IRobotHandPlugin found joint: " << _jointName << std::endl;
  return true;
}

//////////////////////////////////////////////////
bool IRobotHandPlugin::FindJoints(gz::sim::EntityComponentManager & _ecm)
{
  char jointName[256];
  for (int f = 0; f < kNumFingers; ++f) {
    if (f == 0 || f == 1) {
      std::snprintf(jointName, sizeof(jointName),
        "%s_finger[%d]/joint_base_rotation", this->side.c_str(), f);
      if (!this->GetAndPushBackJoint(_ecm, jointName, this->fingerBaseRotationJoints)) {
        return false;
      }
    }

    std::snprintf(jointName, sizeof(jointName),
      "%s_finger[%d]/joint_base", this->side.c_str(), f);
    if (!this->GetAndPushBackJoint(_ecm, jointName, this->fingerBaseJoints)) {
      return false;
    }

    std::snprintf(jointName, sizeof(jointName),
      "%s_finger[%d]/flexible_joint_flex_from_proximal_to_1", this->side.c_str(), f);
    if (!this->GetAndPushBackJoint(_ecm, jointName, this->flexureFlexJoints[f])) {
      return false;
    }
    std::snprintf(jointName, sizeof(jointName),
      "%s_finger[%d]/flexible_joint_twist_from_proximal_to_1", this->side.c_str(), f);
    if (!this->GetAndPushBackJoint(_ecm, jointName, this->flexureTwistJoints[f])) {
      return false;
    }

    for (int l = 1; l < kNumFlexLinks; ++l) {
      std::snprintf(jointName, sizeof(jointName),
        "%s_finger[%d]/flexible_joint_flex_from_%d_to_%d",
        this->side.c_str(), f, l, l + 1);
      if (!this->GetAndPushBackJoint(_ecm, jointName, this->flexureFlexJoints[f])) {
        return false;
      }
      std::snprintf(jointName, sizeof(jointName),
        "%s_finger[%d]/flexible_joint_twist_from_%d_to_%d",
        this->side.c_str(), f, l, l + 1);
      if (!this->GetAndPushBackJoint(_ecm, jointName, this->flexureTwistJoints[f])) {
        return false;
      }
    }

    std::snprintf(jointName, sizeof(jointName),
      "%s_finger[%d]/flexible_joint_flex_from_%d_to_distal",
      this->side.c_str(), f, kNumFlexLinks);
    if (!this->GetAndPushBackJoint(_ecm, jointName, this->flexureFlexJoints[f])) {
      return false;
    }
    std::snprintf(jointName, sizeof(jointName),
      "%s_finger[%d]/flexible_joint_twist_from_%d_to_distal",
      this->side.c_str(), f, kNumFlexLinks);
    if (!this->GetAndPushBackJoint(_ecm, jointName, this->flexureTwistJoints[f])) {
      return false;
    }
  }

  gzmsg << "IRobotHandPlugin found all joints for " << this->side
        << " hand." << std::endl;

  std::size_t numJoints =
    this->fingerBaseRotationJoints.size() + this->fingerBaseJoints.size();
  for (const auto & joints : this->flexureFlexJoints) {
    numJoints += joints.size();
  }
  for (const auto & joints : this->flexureTwistJoints) {
    numJoints += joints.size();
  }
  this->jointStates.name.resize(numJoints);
  this->jointStates.position.resize(numJoints);
  this->jointStates.velocity.resize(numJoints);
  this->jointStates.effort.resize(numJoints);

  return true;
}

//////////////////////////////////////////////////
void IRobotHandPlugin::SetJointSpringParams()
{
  // Fake springiness with an explicit PD force computed every step (see the
  // class-level design note); these are the same gains the original applied
  // via Joint::SetStiffnessDamping().
  const double flexJointKp = 0.187733 * (kNumFlexLinks + 2);
  const double flexJointKd = 0.01;
  const double twistJointKp = 0.187733 * (kNumFlexLinks + 2) * 2.0;
  const double twistJointKd = 0.01;
  const double baseJointKp = 0.020068;
  const double baseJointKd = 0.1;
  const double baseRotationJointKp = 0.0;
  const double baseRotationJointKd = 1.0;

  const double baseJointPreloadTorque = 0.0259865;
  const double baseJointPreloadJointPosition = baseJointPreloadTorque / baseJointKp;

  for (int f = 0; f < kNumFingers; ++f) {
    this->flexureFlexSprings[f].assign(
      this->flexureFlexJoints[f].size(), SpringParams{flexJointKp, flexJointKd, 0.0});
    this->flexureTwistSprings[f].assign(
      this->flexureTwistJoints[f].size(), SpringParams{twistJointKp, twistJointKd, 0.0});
  }

  this->fingerBaseSprings.assign(
    this->fingerBaseJoints.size(),
    SpringParams{baseJointKp, baseJointKd, -baseJointPreloadJointPosition});

  this->fingerBaseRotationSprings.assign(
    this->fingerBaseRotationJoints.size(),
    SpringParams{baseRotationJointKp, baseRotationJointKd, 0.0});
}

//////////////////////////////////////////////////
double IRobotHandPlugin::SpringForce(
  const SpringParams & _spring, double _position, double _velocity)
{
  return -_spring.kp * (_position - _spring.preload) - _spring.kd * _velocity;
}

//////////////////////////////////////////////////
double IRobotHandPlugin::FirstOrZero(
  const std::optional<std::vector<double>> & _values)
{
  if (_values && !_values->empty()) {
    return (*_values)[0];
  }
  return 0.0;
}

//////////////////////////////////////////////////
void IRobotHandPlugin::SetHandleCommand(
  const handle_msgs::msg::HandleControl::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  // Indices: 0 index finger flex, 1 middle finger flex, 2 thumb finger flex,
  // 3 thumb finger antagonist, 4 index/middle finger spread.
  for (int i = 0; i < 5; ++i) {
    this->handleCommand.type[i] = _msg->type[i];
    this->handleCommand.value[i] = _msg->value[i];
    this->handleCommand.valid[i] = _msg->valid[i];
  }
}

//////////////////////////////////////////////////
double IRobotHandPlugin::HandleControlFlexValueToFlexJointAngle(int32_t _value)
{
  // _value is taken as motor rotation in radians. In practice, spool
  // diameter varies from 10mm to 14mm.
  const double intToMotorAngle = 1.0;
  const double motorAngle = intToMotorAngle * static_cast<double>(_value);
  const double spoolDiameter = 0.012;
  const double tendonLength = spoolDiameter * motorAngle;
  const double tendonLengthToAngle = 0.01;  // wild guess
  return tendonLengthToAngle * tendonLength;
}

//////////////////////////////////////////////////
double IRobotHandPlugin::HandleControlSpreadValueToSpreadJointAngle(int32_t _value)
{
  const double intToMotorAngle = 1.0;
  const double motorAngle = intToMotorAngle * static_cast<double>(_value);
  const double reductionRatio = 1.0 / 1000.0;
  return reductionRatio * motorAngle;
}

//////////////////////////////////////////////////
void IRobotHandPlugin::UpdateStates(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (_info.simTime <= this->lastControllerUpdateTime) {
    return;
  }

  const rclcpp::Time stamp(
    std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count());

  this->GetAndPublishHandleState(_ecm, stamp);

  const double dt = std::chrono::duration<double>(
    _info.simTime - this->lastControllerUpdateTime).count();
  this->UpdatePIDControl(_ecm, dt);

  this->lastControllerUpdateTime = _info.simTime;
}

//////////////////////////////////////////////////
void IRobotHandPlugin::GetAndPublishHandleState(
  const gz::sim::EntityComponentManager & _ecm,
  const rclcpp::Time & _stamp)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  // The real HANDLE hand's sensor suite (tactile arrays, motor telemetry,
  // ...) has no equivalent in this simulated hand -- the original left all
  // of this zeroed/empty too, it only ever populated the header stamp.
  this->handleState.header.stamp = _stamp;
  this->handleState.motor_hall_encoder = {0, 0, 0, 0};
  this->handleState.motor_winding_temp = {0.0f, 0.0f, 0.0f, 0.0f};
  this->handleState.air_temp = 0.0f;
  this->handleState.motor_velocity = {0, 0, 0, 0};
  this->handleState.motor_housing_temp = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  this->handleState.motor_current = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 3; ++i) {
    this->handleState.finger_tactile[i].proximal.clear();
    this->handleState.finger_tactile[i].distal.clear();
    this->handleState.finger_tactile_temp[i].proximal.clear();
    this->handleState.finger_tactile_temp[i].distal.clear();
  }
  this->handleState.finger_spread = 0;
  this->handleState.proximal_joint_angle = {0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    this->handleState.distal_joint_angle[i].proximal.clear();
    this->handleState.distal_joint_angle[i].distal.clear();
    this->handleState.proximal_acceleration[i].x = 0.0;
    this->handleState.proximal_acceleration[i].y = 0.0;
    this->handleState.proximal_acceleration[i].z = 0.0;
    this->handleState.distal_acceleration[i].x = 0.0;
    this->handleState.distal_acceleration[i].y = 0.0;
    this->handleState.distal_acceleration[i].z = 0.0;
  }
  for (int i = 0; i < 12; ++i) {
    this->handleState.responses[i] = false;
    this->handleState.response_history[i] = 0;
  }
  for (int i = 0; i < 5; ++i) {
    this->handleState.motor_error[i] = 0;
  }
  this->pubHandleState->publish(this->handleState);

  int njs = 0;
  this->jointStates.header.stamp = _stamp;

  for (int f = 0; f < 2; ++f) {
    gz::sim::Joint joint(this->fingerBaseRotationJoints[f]);
    this->jointStates.name[njs] = "irobot_hand::" + this->side + "_finger_base_rotation_" +
      std::to_string(f);
    this->jointStates.position[njs] = FirstOrZero(joint.Position(_ecm));
    this->jointStates.velocity[njs] = FirstOrZero(joint.Velocity(_ecm));
    ++njs;
  }

  for (int f = 0; f < kNumFingers; ++f) {
    gz::sim::Joint joint(this->fingerBaseJoints[f]);
    this->jointStates.name[njs] = "irobot_hand::" + this->side + "_finger_base_" +
      std::to_string(f);
    this->jointStates.position[njs] = FirstOrZero(joint.Position(_ecm));
    this->jointStates.velocity[njs] = FirstOrZero(joint.Velocity(_ecm));
    ++njs;
  }

  for (int f = 0; f < kNumFingers; ++f) {
    const std::size_t numFlex = this->flexureFlexJoints[f].size();
    for (std::size_t i = 0; i < numFlex; ++i) {
      gz::sim::Joint flexJoint(this->flexureFlexJoints[f][i]);
      this->jointStates.name[njs] = "irobot_hand::" + this->side + "_flex_" +
        std::to_string(f) + "_" + std::to_string(i);
      this->jointStates.position[njs] = FirstOrZero(flexJoint.Position(_ecm));
      this->jointStates.velocity[njs] = FirstOrZero(flexJoint.Velocity(_ecm));
      ++njs;

      gz::sim::Joint twistJoint(this->flexureTwistJoints[f][i]);
      this->jointStates.name[njs] = "irobot_hand::" + this->side + "_twist_" +
        std::to_string(f) + "_" + std::to_string(i);
      this->jointStates.position[njs] = FirstOrZero(twistJoint.Position(_ecm));
      this->jointStates.velocity[njs] = FirstOrZero(twistJoint.Velocity(_ecm));
      ++njs;
    }
  }

  this->pubJointStates->publish(this->jointStates);
}

//////////////////////////////////////////////////
void IRobotHandPlugin::UpdatePIDControl(
  gz::sim::EntityComponentManager & _ecm, double _dt)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  gz::sim::Joint thumbBaseJoint(this->fingerBaseJoints[2]);
  {
    // Antagonist angle is between 0 (no antagonist) and upper - lower
    // (pinned to lower position).
    this->thumbAntagonistAngle = std::max(
      0.0, std::min(
        this->thumbUpperLimit - this->thumbLowerLimit,
        HandleControlFlexValueToFlexJointAngle(this->handleCommand.value[3])));
  }
  const double thumbEffectiveUpperLimit =
    this->thumbUpperLimit - this->thumbAntagonistAngle;

  for (int j = 0; j < 5; ++j) {
    if (j == 3) {
      continue;  // Antagonist angle handled separately above.
    }

    double target;
    if (j == 4) {
      target = HandleControlSpreadValueToSpreadJointAngle(this->handleCommand.value[j]);
    } else {
      target = HandleControlFlexValueToFlexJointAngle(this->handleCommand.value[j]);
    }

    const std::size_t numFlex = this->flexureFlexJoints[j < 4 ? j : 0].size();

    double currentPos = 0.0;
    double currentVel = 0.0;
    if (j < 4) {
      gz::sim::Joint baseJoint(this->fingerBaseJoints[j]);
      double baseJointPos = FirstOrZero(baseJoint.Position(_ecm));
      double baseJointVel = FirstOrZero(baseJoint.Velocity(_ecm));
      double flexureFlexJointPos = 0.0;
      double flexureFlexJointVel = 0.0;
      for (std::size_t i = 0; i < numFlex; ++i) {
        gz::sim::Joint flexJoint(this->flexureFlexJoints[j][i]);
        flexureFlexJointPos += FirstOrZero(flexJoint.Position(_ecm));
        flexureFlexJointVel += FirstOrZero(flexJoint.Velocity(_ecm));
      }
      currentPos = baseJointPos + flexureFlexJointPos;
      currentVel = baseJointVel + flexureFlexJointVel;
    } else {
      gz::sim::Joint rot0(this->fingerBaseRotationJoints[0]);
      gz::sim::Joint rot1(this->fingerBaseRotationJoints[1]);
      currentPos = FirstOrZero(rot0.Position(_ecm)) + FirstOrZero(rot1.Position(_ecm));
      currentVel = FirstOrZero(rot0.Velocity(_ecm)) + FirstOrZero(rot1.Velocity(_ecm));
    }

    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double iEffortMin = 0.0;
    double iEffortMax = 0.0;
    double current = 0.0;
    if (this->handleCommand.type[j] == handle_msgs::msg::HandleControl::POSITION) {
      current = currentPos;
      kp = this->kpPosition[j];
      ki = this->kiPosition[j];
      kd = this->kdPosition[j];
      iEffortMin = this->iPositionEffortMin[j];
      iEffortMax = this->iPositionEffortMax[j];
    } else if (this->handleCommand.type[j] == handle_msgs::msg::HandleControl::VELOCITY) {
      current = currentVel;
      kp = this->kpVelocity[j];
      ki = this->kiVelocity[j];
      kd = this->kdVelocity[j];
      iEffortMin = this->iVelocityEffortMin[j];
      iEffortMax = this->iVelocityEffortMax[j];
    } else if (this->handleCommand.type[j] == 0) {
      continue;  // Uncontrolled.
    } else {
      RCLCPP_ERROR(
        this->rosNode->get_logger(), "Control type [%d] not available for joint %d",
        this->handleCommand.type[j], j);
      continue;
    }

    double torque;
    {
      const double qP = target - current;
      if (_dt > 0.0) {
        this->errorTerms[j].positionErrorDerivative =
          (qP - this->errorTerms[j].positionError) / _dt;
      }
      this->errorTerms[j].positionError = qP;
      this->errorTerms[j].positionErrorIntegral = std::clamp(
        this->errorTerms[j].positionErrorIntegral + _dt * qP, iEffortMin, iEffortMax);
      torque = kp * this->errorTerms[j].positionError +
        ki * this->errorTerms[j].positionErrorIntegral +
        kd * this->errorTerms[j].positionErrorDerivative;
    }

    if (j < 4) {
      // A tendon can only transmit tension, not compression.
      const double tendonTorque = std::max(0.0, torque);
      const auto basePos = thumbBaseJoint.Position(_ecm);

      if (j == 2 && basePos && !basePos->empty() && (*basePos)[0] > thumbEffectiveUpperLimit) {
        gz::sim::Joint(this->fingerBaseJoints[j]).SetForce(_ecm, {0.0});
      } else {
        gz::sim::Joint baseJoint(this->fingerBaseJoints[j]);
        SpringParams dynamicSpring = this->fingerBaseSprings[j];
        dynamicSpring.kd = 0.5 * tendonTorque;
        const double springForce = SpringForce(
          dynamicSpring, FirstOrZero(baseJoint.Position(_ecm)),
          FirstOrZero(baseJoint.Velocity(_ecm)));
        baseJoint.SetForce(_ecm, {springForce + std::max(0.0, tendonTorque / 2.0)});
      }

      for (std::size_t i = 0; i < numFlex; ++i) {
        gz::sim::Joint flexJoint(this->flexureFlexJoints[j][i]);
        SpringParams dynamicSpring = this->flexureFlexSprings[j][i];
        dynamicSpring.kd = 0.5 * tendonTorque / numFlex;
        const double springForce = SpringForce(
          dynamicSpring, FirstOrZero(flexJoint.Position(_ecm)),
          FirstOrZero(flexJoint.Velocity(_ecm)));
        flexJoint.SetForce(_ecm, {springForce + (tendonTorque / 2.0) / numFlex});
      }
    } else {
      SpringParams dynamicSpring = this->fingerBaseRotationSprings[0];
      dynamicSpring.kd = torque;
      gz::sim::Joint rot0(this->fingerBaseRotationJoints[0]);
      const double springForce = SpringForce(
        dynamicSpring, FirstOrZero(rot0.Position(_ecm)), FirstOrZero(rot0.Velocity(_ecm)));
      // Setting one joint is enough due to the gearbox linking them; joint 1
      // only ever gets its static passive spring, applied below with the
      // other passive-only joints.
      rot0.SetForce(_ecm, {springForce + torque});
    }
  }

  // Passive-only joints: the flexure twist joints, and finger base rotation
  // joint 1, never receive active tendon force -- only their spring.
  for (int f = 0; f < kNumFingers; ++f) {
    for (std::size_t i = 0; i < this->flexureTwistJoints[f].size(); ++i) {
      gz::sim::Joint twistJoint(this->flexureTwistJoints[f][i]);
      const double springForce = SpringForce(
        this->flexureTwistSprings[f][i], FirstOrZero(twistJoint.Position(_ecm)),
        FirstOrZero(twistJoint.Velocity(_ecm)));
      twistJoint.SetForce(_ecm, {springForce});
    }
  }
  gz::sim::Joint rot1(this->fingerBaseRotationJoints[1]);
  const double rot1SpringForce = SpringForce(
    this->fingerBaseRotationSprings[1], FirstOrZero(rot1.Position(_ecm)),
    FirstOrZero(rot1.Velocity(_ecm)));
  rot1.SetForce(_ecm, {rot1SpringForce});
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(IRobotHandPlugin,
              gz::sim::System,
              IRobotHandPlugin::ISystemConfigure,
              IRobotHandPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(IRobotHandPlugin,
    "drcsim_gazebo_ros_plugins::IRobotHandPlugin")
