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
#ifndef DRCSIM_GAZEBO_PLUGINS__DRCVEHICLEPLUGIN_HPP_
#define DRCSIM_GAZEBO_PLUGINS__DRCVEHICLEPLUGIN_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <gz/math/PID.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace drcsim_gazebo_plugins
{
/// \brief Simple ackermann-steered vehicle model: pedals, hand wheel,
/// hand brake and FNR switch are position-controlled joints; the four
/// drive wheels receive gas/brake torque directly. Ported from the
/// original Gazebo-Classic DRCVehiclePlugin (drcsim, ROS 1 era) to
/// gz-sim's System interface for Gazebo Harmonic.
///
/// Design notes vs. the original:
/// - The original ODE-specific wheel hard-lock (SetParam stop_erp/
///   stop_cfm toggled every update to rigidly freeze a wheel joint at
///   low speed under heavy braking) has no equivalent in gz-sim's
///   physics-engine-agnostic Joint API and has been dropped; braking is
///   provided entirely by the torque term already present in the
///   original code (brake torque opposing wheel angular velocity),
///   which is physics-engine portable.
/// - The public control API (SetVehicleState, SetHandWheelState, etc.)
///   is kept as ordinary C++ methods, not a transport interface: the
///   only consumer in this codebase, DRCVehicleROSPlugin, subclasses
///   this class and calls these methods directly (same pattern as the
///   original), which gz-sim's System interface supports fine via
///   normal C++ inheritance.
/// - Set*Limits methods (SetHandWheelLimits, SetGasPedalLimits, etc.)
///   are unused by any consumer in this repository. They previously
///   both re-wrote the physics engine's joint limits *and* updated this
///   plugin's internally cached limits; gz-sim has no well-supported
///   runtime joint-limit-mutation API, so these now only update the
///   plugin's internal cached limits (used for percent-utilization
///   calculations), matching what every actual caller needs.
/// - The FNR switch's cosmetic forward/reverse visual fade (published
///   over Gazebo-Classic's internal transport) has been dropped; it had
///   no functional role.
class DRCVehiclePlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  /// \enum DirectionType
  /// \brief Direction selector switch type.
  enum DirectionType
  {
    /// \brief Reverse
    REVERSE = -1,
    /// \brief Neutral
    NEUTRAL = 0,
    /// \brief Forward
    FORWARD = 1
  };

  /// \enum KeyType
  /// \brief Key switch type.
  enum KeyType
  {
    /// \brief On, but hasn't seen Neutral yet
    ON_FR = -1,
    /// \brief Off
    OFF = 0,
    /// \brief On
    ON = 1
  };

  DRCVehiclePlugin() = default;
  ~DRCVehiclePlugin() override = default;

  // Documentation inherited
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  // Documentation inherited
  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  /// \brief Sets DRC Vehicle control inputs, the vehicle internal model
  ///        will decide the overall motion of the vehicle.
  /// \param[in] _handWheelPosition steering wheel position in radians.
  /// \param[in] _gasPedalPosition gas pedal position in meters.
  /// \param[in] _brakePedalPosition brake pedal position in meters.
  /// \param[in] _handBrakePosition handbrake position in radians.
  /// \param[in] _key key state.
  /// \param[in] _direction direction state.
  void SetVehicleState(
    double _handWheelPosition,
    double _gasPedalPosition,
    double _brakePedalPosition,
    double _handBrakePosition,
    KeyType _key,
    DirectionType _direction);

  /// \brief Returns the state of the key switch.
  KeyType GetKeyState() const;
  /// \brief Sets the key switch to ON, may become ON_FR if not in NEUTRAL.
  void SetKeyOn();
  /// \brief Sets the key switch to OFF.
  void SetKeyOff();
  /// \brief Returns the state of the direction switch.
  DirectionType GetDirectionState() const;
  /// \brief Sets the state of the direction switch.
  void SetDirectionState(DirectionType _direction);
  /// \brief Set the steering wheel angle; this will also update the front
  ///        wheel steering angle.
  void SetHandWheelState(double _position);
  /// \brief Sets the plugin's cached lower/upper limits of the steering
  ///        wheel angle (radians). Does not touch the physics joint.
  void SetHandWheelLimits(double _min, double _max);
  /// \brief Returns the cached lower/upper limits of the steering wheel
  ///        angle (radians).
  void GetHandWheelLimits(double & _min, double & _max) const;
  /// \brief Returns the steering wheel angle (rad).
  double GetHandWheelState() const;
  /// \brief Returns the front wheel angle / steering wheel angle ratio.
  double GetHandWheelRatio() const;
  /// \brief Set the hand-brake angle.
  void SetHandBrakeState(double _position);
  /// \brief Sets the plugin's cached lower/upper limits of the hand
  ///        brake angle (radians). Does not touch the physics joint.
  void SetHandBrakeLimits(double _min, double _max);
  /// \brief Returns the cached lower/upper limits of the hand-brake
  ///        angle (radians).
  void GetHandBrakeLimits(double & _min, double & _max) const;
  /// \brief Returns the lower/upper limits of the FNR switch angle
  ///        (radians).
  void GetFNRSwitchLimits(double & _min, double & _max) const;
  /// \brief Returns the hand-brake angle (rad).
  double GetHandBrakeState() const;
  /// \brief Returns the percent utilization of the handbrake relative to
  ///        its cached limits.
  double GetHandBrakePercent() const;
  /// \brief Returns the percent utilization of the FNR switch relative to
  ///        its cached limits.
  double GetFNRSwitchPercent() const;
  /// \brief Record the current sim time as the last time the hand brake
  ///        was externally commanded (used to debounce the internal
  ///        bi-stable hand-brake toggle in PreUpdate).
  void UpdateHandBrakeTime();
  /// \brief Set fnrSwitchCmd from the current direction state and record
  ///        the current sim time as the last time the FNR switch changed
  ///        (used to debounce the internal bi-stable FNR toggle in
  ///        PreUpdate).
  void UpdateFNRSwitchTime();
  /// \brief Specify front wheel orientation in radians (Note: this sets
  /// the vehicle wheels as opposed to the steering wheel angle set by
  /// SetHandWheelState). Zero setting results in vehicle traveling in a
  /// straight line. Positive steering angle results in a left turn in
  /// forward motion. Setting front wheel steering angle will also update
  /// the handWheel steering angle.
  void SetSteeredWheelState(double _position);
  /// \brief Sets the plugin's cached lower/upper limits of the steered
  ///        wheel angle (radians). Does not touch the physics joint.
  void SetSteeredWheelLimits(double _min, double _max);
  /// \brief Returns the steering angle of the steered wheels (rad).
  double GetSteeredWheelState() const;
  /// \brief Specify gas pedal position in meters.
  void SetGasPedalState(double _position);
  /// \brief Sets the plugin's cached lower/upper limits of the gas pedal
  ///        position (meters). Does not touch the physics joint.
  void SetGasPedalLimits(double _min, double _max);
  /// \brief Returns the cached lower/upper limits of the gas pedal
  ///        position (meters).
  void GetGasPedalLimits(double & _min, double & _max) const;
  /// \brief Returns the gas pedal position in meters.
  double GetGasPedalState() const;
  /// \brief Returns the percent utilization of the gas pedal relative to
  ///        its cached limits.
  double GetGasPedalPercent() const;
  /// \brief Specify brake pedal position in meters.
  void SetBrakePedalState(double _position);
  /// \brief Sets the plugin's cached lower/upper limits of the brake
  ///        pedal position (meters). Does not touch the physics joint.
  void SetBrakePedalLimits(double _min, double _max);
  /// \brief Returns the cached lower/upper limits of the brake pedal
  ///        position (meters).
  void GetBrakePedalLimits(double & _min, double & _max) const;
  /// \brief Returns the brake pedal position in meters.
  double GetBrakePedalState() const;
  /// \brief Returns the percent utilization of the brake pedal relative
  ///        to its cached limits.
  double GetBrakePedalPercent() const;

  /// \brief Returns whether `Configure()` succeeded (all required joints
  ///        were resolved). `DRCVehicleROSPlugin` uses this to decide
  ///        whether to stand up its ROS interface at all, mirroring the
  ///        original's `try { Load(...) } catch { return; }`.
  bool IsValidConfig() const;

private:
  double GetGasTorqueMultiplier() const;
  /// \brief Recompute the steering wheel / tire angle ratio from the
  ///        current cached hand wheel and steered wheel limits.
  void UpdateHandWheelRatio();
  /// \brief Look up a joint entity on #model by SDF-configured joint
  ///        name, throwing (via gzerr + returning kNullEntity) on
  ///        failure. `_paramName` is the plugin's SDF parameter that
  ///        holds the joint name.
  gz::sim::Entity RequireJoint(
    const gz::sim::EntityComponentManager & _ecm,
    const std::shared_ptr<const sdf::Element> & _sdf,
    const std::string & _paramName) const;
  /// \brief Radius of the first collision shape on a wheel's child link,
  ///        assuming a cylinder or sphere collision shape.
  double WheelRadius(
    const gz::sim::EntityComponentManager & _ecm,
    gz::sim::Entity _wheelJoint) const;
  /// \brief World position of the first collision shape on a wheel's
  ///        child link.
  gz::math::Vector3d WheelPosition(
    const gz::sim::EntityComponentManager & _ecm,
    gz::sim::Entity _wheelJoint) const;
  /// \brief Read a joint's SDF-configured (lower, upper) position limits.
  ///        Returns (0, 0) if the joint has no JointAxis component.
  std::pair<double, double> JointLimits(
    const gz::sim::EntityComponentManager & _ecm,
    gz::sim::Entity _joint) const;

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity gasPedalJoint{gz::sim::kNullEntity};
  gz::sim::Entity brakePedalJoint{gz::sim::kNullEntity};
  gz::sim::Entity handWheelJoint{gz::sim::kNullEntity};
  gz::sim::Entity handBrakeJoint{gz::sim::kNullEntity};
  gz::sim::Entity fnrSwitchJoint{gz::sim::kNullEntity};
  gz::sim::Entity flWheelJoint{gz::sim::kNullEntity};
  gz::sim::Entity frWheelJoint{gz::sim::kNullEntity};
  gz::sim::Entity blWheelJoint{gz::sim::kNullEntity};
  gz::sim::Entity brWheelJoint{gz::sim::kNullEntity};
  gz::sim::Entity flWheelSteeringJoint{gz::sim::kNullEntity};
  gz::sim::Entity frWheelSteeringJoint{gz::sim::kNullEntity};
  bool validConfig{false};
  /// \brief Current simulation time, refreshed at the top of every
  ///        PreUpdate; used by UpdateHandBrakeTime/UpdateFNRSwitchTime.
  std::chrono::steady_clock::duration currentSimTime{0};
  /// \brief The gas/brake pedals and handbrake apply torque to the wheels
  ///        based on their joint position as a percentage of the total
  ///        range of travel. The constant jointDeadbandPercent adds a small
  ///        deadband between the actual joint limits and the 0% and 100%
  ///        values reported by Get[GasPedal|BrakePedal|HandBrake]Percent()
  const double jointDeadbandPercent{0.02};

  // SDF parameters
  double frontTorque{0.0};
  double backTorque{2000.0};
  double frontBrakeTorque{2000.0};
  double backBrakeTorque{2000.0};
  double maxSpeed{10.0};
  double maxSteer{0.6};
  /// \brief Minimum braking percentage, used to approximate
  ///        rolling resistance and engine braking.
  double minBrakePercent{0.02};

  double steeringRatio{1.0};
  double pedalForce{10.0};
  double handWheelForce{1.0};
  double handBrakeForce{10.0};
  double fnrSwitchForce{0.2};
  double steeredWheelForce{5000.0};

  double gasPedalCmd{0.0};
  double brakePedalCmd{0.0};
  double handWheelCmd{0.0};
  double handBrakeCmd{0.0};
  double fnrSwitchCmd{0.0};
  double flWheelSteeringCmd{0.0};
  double frWheelSteeringCmd{0.0};

  gz::math::PID gasPedalPID;
  gz::math::PID brakePedalPID;
  gz::math::PID handWheelPID;
  gz::math::PID handBrakePID;
  gz::math::PID fnrSwitchPID;
  gz::math::PID flWheelSteeringPID;
  gz::math::PID frWheelSteeringPID;

  /// \brief Time since this vehicle was last put into or taken out of
  ///        hand-brake-engaged state, used to debounce manual toggling.
  std::chrono::steady_clock::duration handBrakeTime{0};
  /// \brief Time since the FNR switch was last toggled, used to debounce
  ///        manual toggling.
  std::chrono::steady_clock::duration fnrSwitchTime{0};

  /// joint information from model
  double gasPedalHigh{0.0};
  double gasPedalLow{0.0};
  double brakePedalHigh{0.0};
  double brakePedalLow{0.0};
  double handWheelHigh{0.0};
  double handWheelLow{0.0};
  double handWheelRange{1.0};
  double handBrakeHigh{0.0};
  double handBrakeLow{0.0};
  double fnrSwitchHigh{0.0};
  double fnrSwitchLow{0.0};
  double flWheelSteeringHigh{0.0};
  double flWheelSteeringLow{0.0};
  double frWheelSteeringHigh{0.0};
  double frWheelSteeringLow{0.0};
  double flWheelRadius{0.1};
  double frWheelRadius{0.1};
  double blWheelRadius{0.1};
  double brWheelRadius{0.1};
  double wheelbaseLength{1.0};
  double frontTrackWidth{1.0};

  /// state of vehicle
  KeyType keyState{ON};
  DirectionType directionState{FORWARD};
  double handWheelState{0.0};
  double handBrakeState{0.0};
  double fnrSwitchState{0.0};
  double flSteeringState{0.0};
  double frSteeringState{0.0};
  double gasPedalState{0.0};
  double brakePedalState{0.0};
  double flWheelState{0.0};
  double frWheelState{0.0};
  double blWheelState{0.0};
  double brWheelState{0.0};

  /// PID gains for the front-left/front-right steering joints
  double fLwheelSteeringPgain{0.0};
  double fRwheelSteeringPgain{0.0};
  double fLwheelSteeringIgain{0.0};
  double fRwheelSteeringIgain{0.0};
  double fLwheelSteeringDgain{0.0};
  double fRwheelSteeringDgain{0.0};
};
}  // namespace drcsim_gazebo_plugins
#endif  // DRCSIM_GAZEBO_PLUGINS__DRCVEHICLEPLUGIN_HPP_
