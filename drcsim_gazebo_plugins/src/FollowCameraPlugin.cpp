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

#include "drcsim_gazebo_plugins/FollowCameraPlugin.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

namespace drcsim_gazebo_plugins
{

static double WrapAngle(double _angle)
{
  return std::atan2(std::sin(_angle), std::cos(_angle));
}

//////////////////////////////////////////////////
void FollowCameraPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "FollowCameraPlugin must be attached to a model.\n";
    return;
  }
  this->targetModelName = _sdf->Get<std::string>("target_model", this->targetModelName).first;
  this->targetLinkName = _sdf->Get<std::string>("target_link", this->targetLinkName).first;
  this->offset = _sdf->Get<gz::math::Pose3d>("offset", this->offset).first;
  this->timeConstant = _sdf->Get<double>("time_constant", this->timeConstant).first;
  this->followHeading = _sdf->Get<bool>("follow_heading", this->followHeading).first;
}

//////////////////////////////////////////////////
void FollowCameraPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (_info.paused || !this->model.Valid(_ecm)) {
    return;
  }
  // The target may be spawned after this model (VRCPlugin spawns Atlas).
  if (this->targetLink == gz::sim::kNullEntity) {
    gz::sim::World world(gz::sim::worldEntity(_ecm));
    const gz::sim::Model target(world.ModelByName(_ecm, this->targetModelName));
    if (!target.Valid(_ecm)) {
      return;
    }
    this->targetLink = target.LinkByName(_ecm, this->targetLinkName);
    if (this->targetLink == gz::sim::kNullEntity) {
      return;
    }
  }

  const gz::math::Pose3d pose = gz::sim::worldPose(this->targetLink, _ecm);
  const gz::math::Vector3d forward = pose.Rot().RotateVector(gz::math::Vector3d::UnitX);
  const gz::math::Vector2d measured(pose.Pos().X(), pose.Pos().Y());
  // Heading from the forward axis projected on the ground; held while it
  // points mostly up or down (a robot lying on its back or front).
  const bool headingValid =
    this->followHeading && std::hypot(forward.X(), forward.Y()) > 0.5;
  const double measuredYaw = std::atan2(forward.Y(), forward.X());

  if (!this->initialized) {
    this->position = measured;
    this->height = pose.Pos().Z();
    this->yaw = headingValid ? measuredYaw : 0.0;
    this->initialized = true;
  } else {
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const double alpha = 1.0 - std::exp(-dt / std::max(1e-3, this->timeConstant));
    this->position += alpha * (measured - this->position);
    if (headingValid) {
      this->yaw += alpha * WrapAngle(measuredYaw - this->yaw);
    }
  }

  const gz::math::Pose3d frame(
    this->position.X(), this->position.Y(), this->height, 0.0, 0.0, this->yaw);
  this->model.SetWorldPoseCmd(_ecm, frame * this->offset);
}

}  // namespace drcsim_gazebo_plugins

using drcsim_gazebo_plugins::FollowCameraPlugin;

GZ_ADD_PLUGIN(FollowCameraPlugin,
              gz::sim::System,
              FollowCameraPlugin::ISystemConfigure,
              FollowCameraPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(FollowCameraPlugin,
    "drcsim_gazebo_plugins::FollowCameraPlugin")
