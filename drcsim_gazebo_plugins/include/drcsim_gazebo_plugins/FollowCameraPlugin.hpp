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
#ifndef DRCSIM_GAZEBO_PLUGINS__FOLLOWCAMERAPLUGIN_HPP_
#define DRCSIM_GAZEBO_PLUGINS__FOLLOWCAMERAPLUGIN_HPP_

#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace drcsim_gazebo_plugins
{
/// \brief Moves its own (gravity-free, collision-free) model to follow a
/// target link smoothly, like a camera operator: position and heading only.
///
/// Attach to a model holding a camera sensor. Every step the model is
/// placed at `offset` in a frame at the target's low-pass-filtered (x, y),
/// the target's height when first seen, and its filtered heading -- never
/// its roll or pitch. A camera rigidly attached to a walking robot's pelvis
/// rolls with every step (4 m out, one degree of sway is 7 cm) and points
/// at the sky when the robot falls; this one doesn't. While the target's
/// forward axis points mostly up or down (lying on its back or front) the
/// heading is held.
///
/// SDF parameters:
///   <target_model>  model name (default "atlas")
///   <target_link>   link name (default "pelvis")
///   <offset>        pose in the follow frame (default "3.3 -2.8 0.3 0 0.06 2.44")
///   <time_constant> low-pass time constant in seconds (default 1.0)
///   <follow_heading> false keeps the heading fixed at 0 (default true)
class FollowCameraPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  FollowCameraPlugin() = default;
  ~FollowCameraPlugin() override = default;

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

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity targetLink{gz::sim::kNullEntity};
  std::string targetModelName{"atlas"};
  std::string targetLinkName{"pelvis"};
  gz::math::Pose3d offset{3.3, -2.8, 0.3, 0.0, 0.06, 2.44};
  double timeConstant{1.0};
  bool followHeading{true};

  bool initialized{false};
  gz::math::Vector2d position;
  double height{0.0};
  double yaw{0.0};
};
}  // namespace drcsim_gazebo_plugins
#endif  // DRCSIM_GAZEBO_PLUGINS__FOLLOWCAMERAPLUGIN_HPP_
