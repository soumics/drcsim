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

#include "drcsim_gazebo_plugins/DRCVehiclePlugin.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/JointAxis.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Sphere.hh>

using drcsim_gazebo_plugins::DRCVehiclePlugin;

namespace
{
  /// \brief Clamp helper matching the old gazebo::math::clamp(value, min, max).
  double Clamp(double _value, double _min, double _max)
  {
    return std::max(_min, std::min(_value, _max));
  }
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "DRCVehiclePlugin should be attached to a model entity. "
          << "Failed to initialize.\n";
    return;
  }

  this->gasPedalJoint = this->RequireJoint(_ecm, _sdf, "gas_pedal");
  this->brakePedalJoint = this->RequireJoint(_ecm, _sdf, "brake_pedal");
  this->handWheelJoint = this->RequireJoint(_ecm, _sdf, "steering_wheel");
  this->handBrakeJoint = this->RequireJoint(_ecm, _sdf, "hand_brake");
  this->fnrSwitchJoint = this->RequireJoint(_ecm, _sdf, "fnr_switch");
  this->flWheelJoint = this->RequireJoint(_ecm, _sdf, "front_left_wheel");
  this->frWheelJoint = this->RequireJoint(_ecm, _sdf, "front_right_wheel");
  this->blWheelJoint = this->RequireJoint(_ecm, _sdf, "back_left_wheel");
  this->brWheelJoint = this->RequireJoint(_ecm, _sdf, "back_right_wheel");
  this->flWheelSteeringJoint =
      this->RequireJoint(_ecm, _sdf, "front_left_wheel_steering");
  this->frWheelSteeringJoint =
      this->RequireJoint(_ecm, _sdf, "front_right_wheel_steering");

  if (this->gasPedalJoint == gz::sim::kNullEntity ||
      this->brakePedalJoint == gz::sim::kNullEntity ||
      this->handWheelJoint == gz::sim::kNullEntity ||
      this->handBrakeJoint == gz::sim::kNullEntity ||
      this->fnrSwitchJoint == gz::sim::kNullEntity ||
      this->flWheelJoint == gz::sim::kNullEntity ||
      this->frWheelJoint == gz::sim::kNullEntity ||
      this->blWheelJoint == gz::sim::kNullEntity ||
      this->brWheelJoint == gz::sim::kNullEntity ||
      this->flWheelSteeringJoint == gz::sim::kNullEntity ||
      this->frWheelSteeringJoint == gz::sim::kNullEntity)
  {
    // RequireJoint already logged the specific failure.
    return;
  }

  // Enable position/velocity sensing -- gz-sim only populates these
  // components when a consumer opts in, for performance.
  gz::sim::Joint(this->gasPedalJoint).EnablePositionCheck(_ecm);
  gz::sim::Joint(this->brakePedalJoint).EnablePositionCheck(_ecm);
  gz::sim::Joint(this->handWheelJoint).EnablePositionCheck(_ecm);
  gz::sim::Joint(this->handBrakeJoint).EnablePositionCheck(_ecm);
  gz::sim::Joint(this->fnrSwitchJoint).EnablePositionCheck(_ecm);
  gz::sim::Joint(this->flWheelSteeringJoint).EnablePositionCheck(_ecm);
  gz::sim::Joint(this->frWheelSteeringJoint).EnablePositionCheck(_ecm);
  gz::sim::Joint(this->flWheelJoint).EnableVelocityCheck(_ecm);
  gz::sim::Joint(this->frWheelJoint).EnableVelocityCheck(_ecm);
  gz::sim::Joint(this->blWheelJoint).EnableVelocityCheck(_ecm);
  gz::sim::Joint(this->brWheelJoint).EnableVelocityCheck(_ecm);

  // Put some deadband at the end of range for gas and brake pedals,
  // hand brake and FNR switch.
  auto applyDeadband = [this, &_ecm](gz::sim::Entity _joint,
      double &_high, double &_low)
  {
    auto [lower, upper] = this->JointLimits(_ecm, _joint);
    double jointCenter = (upper + lower) / 2.0;
    _high = jointCenter +
        (1 - this->jointDeadbandPercent) * (upper - jointCenter);
    _low = jointCenter +
        (1 - this->jointDeadbandPercent) * (lower - jointCenter);
  };

  applyDeadband(this->gasPedalJoint, this->gasPedalHigh, this->gasPedalLow);
  applyDeadband(this->brakePedalJoint,
      this->brakePedalHigh, this->brakePedalLow);
  applyDeadband(this->handBrakeJoint,
      this->handBrakeHigh, this->handBrakeLow);
  this->handBrakeCmd = this->handBrakeHigh;
  applyDeadband(this->fnrSwitchJoint,
      this->fnrSwitchHigh, this->fnrSwitchLow);

  // Hand wheel and steered wheel limits are not deadbanded -- they drive
  // the steering ratio, not a percent-utilization readout.
  std::tie(this->handWheelLow, this->handWheelHigh) =
      this->JointLimits(_ecm, this->handWheelJoint);
  std::tie(this->flWheelSteeringLow, this->flWheelSteeringHigh) =
      this->JointLimits(_ecm, this->flWheelSteeringJoint);
  std::tie(this->frWheelSteeringLow, this->frWheelSteeringHigh) =
      this->JointLimits(_ecm, this->frWheelSteeringJoint);

  this->UpdateFNRSwitchTime();

  // get some vehicle parameters
  this->frontTorque = _sdf->Get<double>("front_torque", 0.0).first;
  this->backTorque = _sdf->Get<double>("back_torque", 2000.0).first;
  this->frontBrakeTorque =
      _sdf->Get<double>("front_brake_torque", 2000.0).first;
  this->backBrakeTorque =
      _sdf->Get<double>("back_brake_torque", 2000.0).first;
  this->maxSpeed = _sdf->Get<double>("max_speed", 10.0).first;
  this->maxSteer = _sdf->Get<double>("max_steer", 0.6).first;
  this->minBrakePercent = _sdf->Get<double>("min_brake_percent", 0.02).first;
  this->fLwheelSteeringPgain =
      _sdf->Get<double>("flwheel_steering_p_gain", 0.0).first;
  this->fRwheelSteeringPgain =
      _sdf->Get<double>("frwheel_steering_p_gain", 0.0).first;
  this->fLwheelSteeringIgain =
      _sdf->Get<double>("flwheel_steering_i_gain", 0.0).first;
  this->fRwheelSteeringIgain =
      _sdf->Get<double>("frwheel_steering_i_gain", 0.0).first;
  this->fLwheelSteeringDgain =
      _sdf->Get<double>("flwheel_steering_d_gain", 0.0).first;
  this->fRwheelSteeringDgain =
      _sdf->Get<double>("frwheel_steering_d_gain", 0.0).first;

  this->UpdateHandWheelRatio();

  // Update wheel radius for each wheel from its child link's collision
  // shape (assumes the wheel link has a single cylinder or sphere
  // collision, and is the joint's child link).
  this->flWheelRadius = this->WheelRadius(_ecm, this->flWheelJoint);
  this->frWheelRadius = this->WheelRadius(_ecm, this->frWheelJoint);
  this->blWheelRadius = this->WheelRadius(_ecm, this->blWheelJoint);
  this->brWheelRadius = this->WheelRadius(_ecm, this->brWheelJoint);

  // Compute wheelbase and front track width from wheel collision positions.
  gz::math::Vector3d flCenterPos = this->WheelPosition(_ecm, flWheelJoint);
  gz::math::Vector3d frCenterPos = this->WheelPosition(_ecm, frWheelJoint);
  gz::math::Vector3d blCenterPos = this->WheelPosition(_ecm, blWheelJoint);
  gz::math::Vector3d brCenterPos = this->WheelPosition(_ecm, brWheelJoint);
  this->frontTrackWidth = (flCenterPos - frCenterPos).Length();
  gz::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  gz::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
  this->wheelbaseLength = (frontAxlePos - backAxlePos).Length();

  // initialize controllers for car
  this->gasPedalPID.Init(800, 0, 0, 0, 0,
      this->pedalForce, -this->pedalForce);
  this->brakePedalPID.Init(800, 0, 0, 0, 0,
      this->pedalForce, -this->pedalForce);
  this->handWheelPID.Init(100, 0, 0, 0, 0,
      this->handWheelForce, -this->handWheelForce);
  this->handBrakePID.Init(30, 0, 0, 0, 0,
      this->handBrakeForce, -this->handBrakeForce);
  this->fnrSwitchPID.Init(30, 0, 0, 0, 0,
      this->fnrSwitchForce, -this->fnrSwitchForce);
  this->flWheelSteeringPID.Init(this->fLwheelSteeringPgain,
      this->fLwheelSteeringIgain, this->fLwheelSteeringDgain,
      0, 0, this->steeredWheelForce, -this->steeredWheelForce);
  this->frWheelSteeringPID.Init(this->fRwheelSteeringPgain,
      this->fRwheelSteeringIgain, this->fRwheelSteeringDgain,
      0, 0, this->steeredWheelForce, -this->steeredWheelForce);

  this->validConfig = true;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !this->validConfig)
    return;

  double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0)
    return;

  this->currentSimTime = _info.simTime;

  gz::sim::Joint handWheel(this->handWheelJoint);
  gz::sim::Joint handBrake(this->handBrakeJoint);
  gz::sim::Joint fnrSwitch(this->fnrSwitchJoint);
  gz::sim::Joint brakePedal(this->brakePedalJoint);
  gz::sim::Joint gasPedal(this->gasPedalJoint);
  gz::sim::Joint flWheelSteering(this->flWheelSteeringJoint);
  gz::sim::Joint frWheelSteering(this->frWheelSteeringJoint);
  gz::sim::Joint flWheel(this->flWheelJoint);
  gz::sim::Joint frWheel(this->frWheelJoint);
  gz::sim::Joint blWheel(this->blWheelJoint);
  gz::sim::Joint brWheel(this->brWheelJoint);

  this->handWheelState = handWheel.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->handBrakeState = handBrake.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->fnrSwitchState = fnrSwitch.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->brakePedalState = brakePedal.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->gasPedalState = gasPedal.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->flSteeringState = flWheelSteering.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->frSteeringState = frWheelSteering.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];

  this->flWheelState = flWheel.Velocity(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->frWheelState = frWheel.Velocity(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->blWheelState = blWheel.Velocity(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->brWheelState = brWheel.Velocity(_ecm).value_or(
      std::vector<double>{0.0})[0];

  std::chrono::duration<double> dtDuration(dt);

  // PID (position) steering
  double steerError = this->handWheelState - this->handWheelCmd;
  double steerCmd = this->handWheelPID.Update(steerError, dtDuration);
  handWheel.SetForce(_ecm, {steerCmd});

  // Bi-stable switching of hand-brake reference point. Note: handBrakeTime
  // is a "last externally commanded" timestamp -- it is only updated by
  // UpdateHandBrakeTime() (called externally, e.g. by DRCVehicleROSPlugin
  // when a ROS command arrives), not by this block itself, matching the
  // original.
  double handBrakeHysteresis = 0.2;
  double handBrakeCmdEps = 0.01;
  auto simTimeSec = std::chrono::duration<double>(this->currentSimTime).count();
  auto handBrakeTimeSec =
      std::chrono::duration<double>(this->handBrakeTime).count();
  if (this->handBrakeCmd < (this->handBrakeLow + handBrakeCmdEps) &&
      this->GetHandBrakePercent() > (0.5 + handBrakeHysteresis) &&
      (simTimeSec - handBrakeTimeSec) > 0.5)
  {
    this->handBrakeCmd = this->handBrakeHigh;
    gzlog << "Hand brake manually enabled\n";
  } else if (this->handBrakeCmd > (this->handBrakeHigh - handBrakeCmdEps) &&
      this->GetHandBrakePercent() < (0.5 - handBrakeHysteresis) &&
      (simTimeSec - handBrakeTimeSec) > 0.5)
  {
    this->handBrakeCmd = this->handBrakeLow;
    gzlog << "Hand brake manually disabled\n";
  }

  // PID (position) hand brake
  double handBrakeError = this->handBrakeState - this->handBrakeCmd;
  double handBrakePIDCmd = this->handBrakePID.Update(
      handBrakeError, dtDuration);
  handBrake.SetForce(_ecm, {handBrakePIDCmd});

  // Bi-stable switching of FNR switch reference point
  double fnrSwitchHysteresis = handBrakeHysteresis;
  double fnrSwitchCmdEps = handBrakeCmdEps;
  auto fnrSwitchTimeSec =
      std::chrono::duration<double>(this->fnrSwitchTime).count();
  if (this->fnrSwitchCmd < (this->fnrSwitchLow + fnrSwitchCmdEps) &&
      this->GetFNRSwitchPercent() > (0.5 + fnrSwitchHysteresis) &&
      (simTimeSec - fnrSwitchTimeSec) > 0.5)
  {
    this->SetDirectionState(REVERSE);
    this->UpdateFNRSwitchTime();
    gzlog << "FNR switch manually set to reverse\n";
  } else if (this->fnrSwitchCmd > (this->fnrSwitchHigh - fnrSwitchCmdEps) &&
      this->GetFNRSwitchPercent() < (0.5 - fnrSwitchHysteresis) &&
      (simTimeSec - fnrSwitchTimeSec) > 0.5)
  {
    this->SetDirectionState(FORWARD);
    this->UpdateFNRSwitchTime();
    gzlog << "FNR switch manually set to forward\n";
  }

  // PID (position) FNR switch
  double fnrSwitchError = this->fnrSwitchState - this->fnrSwitchCmd;
  double fnrSwitchPIDCmd = this->fnrSwitchPID.Update(
      fnrSwitchError, dtDuration);
  fnrSwitch.SetForce(_ecm, {fnrSwitchPIDCmd});

  // PID (position) gas pedal
  double gasError = this->gasPedalState - this->gasPedalCmd;
  double gasCmd = this->gasPedalPID.Update(gasError, dtDuration);
  gasPedal.SetForce(_ecm, {gasCmd});

  // PID (position) brake pedal
  double brakeError = this->brakePedalState - this->brakePedalCmd;
  double brakeCmd = this->brakePedalPID.Update(brakeError, dtDuration);
  brakePedal.SetForce(_ecm, {brakeCmd});

  // PID (position) steering joints based on steering position
  // Ackermann steering geometry here.
  double tanSteer = tan(this->handWheelState * this->steeringRatio);
  this->flWheelSteeringCmd = atan2(tanSteer,
      1 - this->frontTrackWidth / 2 / this->wheelbaseLength * tanSteer);
  this->frWheelSteeringCmd = atan2(tanSteer,
      1 + this->frontTrackWidth / 2 / this->wheelbaseLength * tanSteer);

  double flwsError = this->flSteeringState - this->flWheelSteeringCmd;
  double flwsCmd = this->flWheelSteeringPID.Update(flwsError, dtDuration);
  flWheelSteering.SetForce(_ecm, {flwsCmd});

  double frwsError = this->frSteeringState - this->frWheelSteeringCmd;
  double frwsCmd = this->frWheelSteeringPID.Update(frwsError, dtDuration);
  frWheelSteering.SetForce(_ecm, {frwsCmd});

  // Gas pedal torque.
  // Map gas torques to individual wheels, cutting off at max speed.
  // Apply equal torque at left and right wheels (implicit differential).
  double gasPercent = this->GetGasPedalPercent();
  double gasMultiplier = this->GetGasTorqueMultiplier();
  double flGasTorque = 0, frGasTorque = 0, blGasTorque = 0, brGasTorque = 0;
  if ((std::fabs(this->flWheelState * this->flWheelRadius) < this->maxSpeed)
    && (std::fabs(this->frWheelState * this->frWheelRadius) < this->maxSpeed))
  {
    flGasTorque = gasPercent * this->frontTorque * gasMultiplier;
    frGasTorque = gasPercent * this->frontTorque * gasMultiplier;
  }
  if ((std::fabs(this->blWheelState * this->blWheelRadius) < this->maxSpeed)
    && (std::fabs(this->brWheelState * this->brWheelRadius) < this->maxSpeed))
  {
    blGasTorque = gasPercent * this->backTorque * gasMultiplier;
    brGasTorque = gasPercent * this->backTorque * gasMultiplier;
  }

  // Brake pedal, hand-brake torque.
  double brakePercent =
      this->GetBrakePedalPercent() + this->GetHandBrakePercent();
  brakePercent = Clamp(brakePercent, this->minBrakePercent, 1.0);
  // Map brake torques to individual wheels, opposing wheel spin direction.
  // Below the smoothing speed in rad/s, reduce applied brake torque.
  double smoothingSpeed = 0.5;
  double flBrakeTorque = -brakePercent * this->frontBrakeTorque *
      Clamp(this->flWheelState / smoothingSpeed, -1.0, 1.0);
  double frBrakeTorque = -brakePercent * this->frontBrakeTorque *
      Clamp(this->frWheelState / smoothingSpeed, -1.0, 1.0);
  double blBrakeTorque = -brakePercent * this->backBrakeTorque *
      Clamp(this->blWheelState / smoothingSpeed, -1.0, 1.0);
  double brBrakeTorque = -brakePercent * this->backBrakeTorque *
      Clamp(this->brWheelState / smoothingSpeed, -1.0, 1.0);

  flWheel.SetForce(_ecm, {flGasTorque + flBrakeTorque});
  frWheel.SetForce(_ecm, {frGasTorque + frBrakeTorque});
  blWheel.SetForce(_ecm, {blGasTorque + blBrakeTorque});
  brWheel.SetForce(_ecm, {brGasTorque + brBrakeTorque});
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetVehicleState(double _handWheelPosition,
    double _gasPedalPosition, double _brakePedalPosition,
    double _handBrakePosition, KeyType _key, DirectionType _direction)
{
  // This function isn't currently looking at joint limits.
  this->handWheelCmd = _handWheelPosition;
  this->handBrakeCmd = _handBrakePosition;
  this->gasPedalCmd = _gasPedalPosition;
  this->brakePedalCmd = _brakePedalPosition;
  this->directionState = _direction;
  this->keyState = _key;
}

//////////////////////////////////////////////////
DRCVehiclePlugin::DirectionType DRCVehiclePlugin::GetDirectionState() const
{
  return this->directionState;
}

//////////////////////////////////////////////////
DRCVehiclePlugin::KeyType DRCVehiclePlugin::GetKeyState() const
{
  return this->keyState;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetDirectionState(DirectionType _direction)
{
  this->directionState = _direction;
  if (_direction == NEUTRAL && this->keyState == ON_FR)
    this->keyState = ON;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetKeyOff()
{
  this->keyState = OFF;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetKeyOn()
{
  if (this->directionState == NEUTRAL)
    this->keyState = ON;
  else
    this->keyState = ON_FR;
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasTorqueMultiplier() const
{
  if (this->keyState == ON)
  {
    if (this->directionState == FORWARD)
      return 1.0;
    else if (this->directionState == REVERSE)
      return -1.0;
  }
  return 0;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandBrakeState(double _position)
{
  this->handBrakeCmd =
      Clamp(_position, this->handBrakeLow, this->handBrakeHigh);
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandBrakeLimits(double _min, double _max)
{
  this->handBrakeHigh = _max;
  this->handBrakeLow = _min;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::GetHandBrakeLimits(double &_min, double &_max) const
{
  _max = this->handBrakeHigh;
  _min = this->handBrakeLow;
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandBrakeState() const
{
  return this->handBrakeState;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandWheelState(double _position)
{
  this->handWheelCmd =
      Clamp(_position, this->handWheelLow, this->handWheelHigh);
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandWheelLimits(double _min, double _max)
{
  this->handWheelHigh = _max;
  this->handWheelLow = _min;
  this->UpdateHandWheelRatio();
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::GetHandWheelLimits(double &_min, double &_max) const
{
  _max = this->handWheelHigh;
  _min = this->handWheelLow;
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandWheelState() const
{
  return this->handWheelState;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::UpdateHandWheelRatio()
{
  this->handWheelRange = this->handWheelHigh - this->handWheelLow;
  double high = std::min(
      this->flWheelSteeringHigh, this->frWheelSteeringHigh);
  high = std::min(high, this->maxSteer);
  double low = std::max(
      this->flWheelSteeringLow, this->frWheelSteeringLow);
  low = std::max(low, -this->maxSteer);
  double tireAngleRange = high - low;

  this->steeringRatio = tireAngleRange / this->handWheelRange;
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandWheelRatio() const
{
  return this->steeringRatio;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetSteeredWheelState(double _position)
{
  this->SetHandWheelState(_position / this->steeringRatio);
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetSteeredWheelLimits(double _min, double _max)
{
  this->flWheelSteeringHigh = _max;
  this->flWheelSteeringLow = _min;
  this->frWheelSteeringHigh = _max;
  this->frWheelSteeringLow = _min;
  this->UpdateHandWheelRatio();
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetSteeredWheelState() const
{
  return 0.5 * (this->flSteeringState + this->frSteeringState);
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetGasPedalState(double _position)
{
  this->gasPedalCmd =
      Clamp(_position, this->gasPedalLow, this->gasPedalHigh);
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetGasPedalLimits(double _min, double _max)
{
  this->gasPedalHigh = _max;
  this->gasPedalLow = _min;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::GetGasPedalLimits(double &_min, double &_max) const
{
  _max = this->gasPedalHigh;
  _min = this->gasPedalLow;
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasPedalState() const
{
  return this->gasPedalState;
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasPedalPercent() const
{
  return Clamp((this->gasPedalState - this->gasPedalLow) /
      (this->gasPedalHigh - this->gasPedalLow), 0.0, 1.0);
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetBrakePedalPercent() const
{
  return Clamp((this->brakePedalState - this->brakePedalLow) /
      (this->brakePedalHigh - this->brakePedalLow), 0.0, 1.0);
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandBrakePercent() const
{
  return Clamp((this->handBrakeState - this->handBrakeLow) /
      (this->handBrakeHigh - this->handBrakeLow), 0.0, 1.0);
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetFNRSwitchPercent() const
{
  return Clamp((this->fnrSwitchState - this->fnrSwitchLow) /
      (this->fnrSwitchHigh - this->fnrSwitchLow), 0.0, 1.0);
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::GetFNRSwitchLimits(double &_min, double &_max) const
{
  _max = this->fnrSwitchHigh;
  _min = this->fnrSwitchLow;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::UpdateHandBrakeTime()
{
  this->handBrakeTime = this->currentSimTime;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::UpdateFNRSwitchTime()
{
  // Note: the original also toggled a forward/reverse indicator visual by
  // publishing to Gazebo-Classic's internal transport; that was purely
  // cosmetic and has been dropped in this port.
  this->fnrSwitchTime = this->currentSimTime;
  switch (this->directionState)
  {
    case FORWARD:
      this->fnrSwitchCmd = this->fnrSwitchLow;
      break;
    case REVERSE:
      this->fnrSwitchCmd = this->fnrSwitchHigh;
      break;
    case NEUTRAL:
      gzdbg << "The FNR switch does not support Neutral.\n";
      break;
    default:
      gzerr << "Invalid direction state " << this->directionState << "\n";
      break;
  }
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetBrakePedalState(double _position)
{
  this->brakePedalCmd =
      Clamp(_position, this->brakePedalLow, this->brakePedalHigh);
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::SetBrakePedalLimits(double _min, double _max)
{
  this->brakePedalHigh = _max;
  this->brakePedalLow = _min;
}

//////////////////////////////////////////////////
void DRCVehiclePlugin::GetBrakePedalLimits(double &_min, double &_max) const
{
  _max = this->brakePedalHigh;
  _min = this->brakePedalLow;
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::GetBrakePedalState() const
{
  return this->brakePedalState;
}

//////////////////////////////////////////////////
gz::sim::Entity DRCVehiclePlugin::RequireJoint(
    const gz::sim::EntityComponentManager &_ecm,
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_paramName) const
{
  if (!_sdf->HasElement(_paramName))
  {
    gzerr << "<" << _paramName << "> is required, but was not found.\n";
    return gz::sim::kNullEntity;
  }
  std::string jointName = _sdf->Get<std::string>(_paramName);
  gz::sim::Entity joint = this->model.JointByName(_ecm, jointName);
  if (joint == gz::sim::kNullEntity)
  {
    gzerr << "<" << _paramName << ">" << jointName
          << "</" << _paramName << "> does not exist\n";
  }
  return joint;
}

//////////////////////////////////////////////////
std::pair<double, double> DRCVehiclePlugin::JointLimits(
    const gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _joint) const
{
  auto axisComp = _ecm.Component<gz::sim::components::JointAxis>(_joint);
  if (!axisComp)
    return {0.0, 0.0};
  return {axisComp->Data().Lower(), axisComp->Data().Upper()};
}

//////////////////////////////////////////////////
double DRCVehiclePlugin::WheelRadius(
    const gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _wheelJoint) const
{
  auto childLinkNameComp =
      _ecm.Component<gz::sim::components::ChildLinkName>(_wheelJoint);
  if (!childLinkNameComp)
    return 0.0;
  gz::sim::Entity link =
      this->model.LinkByName(_ecm, childLinkNameComp->Data());
  auto collisions = gz::sim::Link(link).Collisions(_ecm);
  if (collisions.empty())
    return 0.0;
  auto geomComp =
      _ecm.Component<gz::sim::components::Geometry>(collisions[0]);
  if (!geomComp)
    return 0.0;
  const sdf::Geometry &geom = geomComp->Data();
  if (geom.CylinderShape())
    return geom.CylinderShape()->Radius();
  if (geom.SphereShape())
    return geom.SphereShape()->Radius();
  return 0.0;
}

//////////////////////////////////////////////////
gz::math::Vector3d DRCVehiclePlugin::WheelPosition(
    const gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _wheelJoint) const
{
  auto childLinkNameComp =
      _ecm.Component<gz::sim::components::ChildLinkName>(_wheelJoint);
  if (!childLinkNameComp)
    return gz::math::Vector3d::Zero;
  gz::sim::Entity link =
      this->model.LinkByName(_ecm, childLinkNameComp->Data());
  auto collisions = gz::sim::Link(link).Collisions(_ecm);
  if (collisions.empty())
    return gz::math::Vector3d::Zero;
  return gz::sim::worldPose(collisions[0], _ecm).Pos();
}

GZ_ADD_PLUGIN(DRCVehiclePlugin,
              gz::sim::System,
              DRCVehiclePlugin::ISystemConfigure,
              DRCVehiclePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DRCVehiclePlugin,
    "drcsim_gazebo_plugins::DRCVehiclePlugin")
