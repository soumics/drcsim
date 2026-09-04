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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__CONTACTMODELPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__CONTACTMODELPLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Publishes gz::msgs::Contacts for an arbitrary, named set of a
/// model's collisions. Ported from the original Gazebo-Classic
/// ContactModelPlugin (drcsim, ROS 1 era), which used
/// physics::ContactManager::CreateFilter on a set of collision names.
///
/// gz-sim has no equivalent arbitrary-collision-set filter API; instead,
/// contact data becomes available on a collision entity once it carries a
/// components::ContactSensorData component (normally added when an SDF
/// <sensor type="contact"> exists on it). This plugin reproduces the
/// original's "monitor these collisions without a dedicated sensor" behavior
/// by force-creating that component on each matched collision itself --
/// the same technique gz-sim's own example TouchPlugin uses.
///
/// SDF interface (unchanged from the original):
///   <plugin filename="libContactModelPlugin.so" name="...">
///     <contact>
///       <collision>link_name::collision_name</collision>  (repeatable)
///       <topic>optional_topic_name</topic>
///     </contact>
///   </plugin>
class ContactModelPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  ContactModelPlugin() = default;
  ~ContactModelPlugin() override = default;

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

  // Documentation inherited
  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  /// \brief Parses the <contact> SDF element and matches its <collision>
  /// names against this model's own collision entities, once they exist.
  void Load(gz::sim::EntityComponentManager & _ecm);

  gz::sim::Model model{gz::sim::kNullEntity};
  std::shared_ptr<const sdf::Element> sdfConfig;
  bool initialized{false};
  bool validConfig{false};

  std::vector<std::string> collisionNames;
  std::vector<gz::sim::Entity> collisionEntities;

  gz::transport::Node node;
  gz::transport::Node::Publisher contactsPub;
};
}  // namespace drcsim_gazebo_ros_plugins
#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__CONTACTMODELPLUGIN_HPP_
