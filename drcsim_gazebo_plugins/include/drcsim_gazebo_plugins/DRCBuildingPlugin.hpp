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
#ifndef DRCSIM_GAZEBO_PLUGINS__DRCBUILDINGPLUGIN_HPP_
#define DRCSIM_GAZEBO_PLUGINS__DRCBUILDINGPLUGIN_HPP_

#include <memory>
#include <string>

#include <gz/math/PID.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace drcsim_gazebo_plugins
{
/// \brief Position-controls a door and its handle joint. Ported from the
/// original Gazebo-Classic DRCBuildingPlugin (drcsim, ROS 1 era) to
/// gz-sim's System interface for Gazebo Harmonic.
///
/// Design note vs. the original: the original additionally simulated a
/// door latch/lock by dynamically toggling the door joint's physics
/// limits (SetHighStop/SetLowStop) and hard-resetting its position to
/// zero once the handle and door were both near-zero. gz-sim has no
/// well-supported runtime joint-limit-mutation API (same issue as
/// DRCVehiclePlugin's wheel braking), so that toggle has been dropped;
/// the PID controller alone holds the door near zero when doorCmd is 0.
class DRCBuildingPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  DRCBuildingPlugin() = default;
  ~DRCBuildingPlugin() override = default;

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

  /// \brief Sets DRC Building door position (rad) given door Joint name.
  ///   - zero angle means door is closed
  ///   - door hinge axis points upwards, which means
  ///     negative angle swings door counter-clockwise if view
  ///     from above.
  void SetDoorState(double _angle);
  /// \brief Returns DRC Building door position (rad).
  double GetDoorState() const;

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity doorJoint{gz::sim::kNullEntity};
  gz::sim::Entity handleJoint{gz::sim::kNullEntity};
  bool validConfig{false};

  gz::math::PID doorPID;
  double doorState{0.0};
  double doorCmd{0.0};
  gz::math::PID handlePID;
  double handleState{0.0};
  double handleCmd{0.0};
};
}  // namespace drcsim_gazebo_plugins
#endif  // DRCSIM_GAZEBO_PLUGINS__DRCBUILDINGPLUGIN_HPP_
