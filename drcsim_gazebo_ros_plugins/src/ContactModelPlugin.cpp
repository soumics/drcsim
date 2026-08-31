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
#include "drcsim_gazebo_ros_plugins/ContactModelPlugin.hpp"

#include <string>
#include <vector>

#include <gz/msgs/contacts.pb.h>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/TopicUtils.hh>

using drcsim_gazebo_ros_plugins::ContactModelPlugin;

//////////////////////////////////////////////////
void ContactModelPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "ContactModelPlugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->sdfConfig = _sdf;
}

//////////////////////////////////////////////////
void ContactModelPlugin::Load(gz::sim::EntityComponentManager & _ecm)
{
  if (!this->sdfConfig->HasElement("contact")) {
    return;
  }
  auto contactElem = this->sdfConfig->FindElement("contact");

  for (auto collisionElem = contactElem->FindElement("collision");
    collisionElem;
    collisionElem = collisionElem->GetNextElement("collision"))
  {
    this->collisionNames.push_back(collisionElem->Get<std::string>());
  }

  std::string topicName;
  if (contactElem->HasElement("topic")) {
    topicName = contactElem->Get<std::string>("topic");
  } else {
    topicName = "/model/" + this->model.Name(_ecm) + "/contacts";
  }
  topicName = gz::transport::TopicUtils::AsValidTopic(topicName);
  if (topicName.empty()) {
    gzerr << "ContactModelPlugin: invalid topic name, plugin disabled."
          << std::endl;
    return;
  }
  this->contactsPub = this->node.Advertise<gz::msgs::Contacts>(topicName);

  auto linkEntities =
    _ecm.ChildrenByComponents(this->model.Entity(), gz::sim::components::Link());
  for (const gz::sim::Entity linkEntity : linkEntities) {
    const std::string & linkName =
      _ecm.Component<gz::sim::components::Name>(linkEntity)->Data();

    auto collisionEntities = _ecm.ChildrenByComponents(
      linkEntity, gz::sim::components::Collision());
    for (const gz::sim::Entity collisionEntity : collisionEntities) {
      const std::string & collisionName =
        _ecm.Component<gz::sim::components::Name>(collisionEntity)->Data();
      const std::string scopedName = linkName + "::" + collisionName;

      bool wanted = false;
      for (const std::string & name : this->collisionNames) {
        if (name == scopedName) {
          wanted = true;
          break;
        }
      }
      if (!wanted) {
        continue;
      }

      if (!_ecm.EntityHasComponentType(
          collisionEntity, gz::sim::components::ContactSensorData::typeId))
      {
        _ecm.CreateComponent(
          collisionEntity, gz::sim::components::ContactSensorData());
      }
      this->collisionEntities.push_back(collisionEntity);
    }
  }

  this->validConfig = !this->collisionEntities.empty();
  if (!this->validConfig) {
    gzerr << "ContactModelPlugin: none of the configured <collision> names "
          << "matched a collision on model [" << this->model.Name(_ecm)
          << "]." << std::endl;
  }
}

//////////////////////////////////////////////////
void ContactModelPlugin::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized && this->sdfConfig) {
    // Deferred from Configure(): a model's child link/collision entities
    // are not guaranteed to exist yet when Configure() runs.
    this->Load(_ecm);
    this->initialized = true;
  }
}

//////////////////////////////////////////////////
void ContactModelPlugin::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  if (!this->validConfig || _info.paused) {
    return;
  }

  gz::msgs::Contacts contactsMsg;
  for (const gz::sim::Entity collisionEntity : this->collisionEntities) {
    const auto * contacts =
      _ecm.Component<gz::sim::components::ContactSensorData>(collisionEntity);
    if (!contacts) {
      continue;
    }
    for (const auto & contact : contacts->Data().contact()) {
      contactsMsg.add_contact()->CopyFrom(contact);
    }
  }

  this->contactsPub.Publish(contactsMsg);
}

GZ_ADD_PLUGIN(ContactModelPlugin,
              gz::sim::System,
              ContactModelPlugin::ISystemConfigure,
              ContactModelPlugin::ISystemPreUpdate,
              ContactModelPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(ContactModelPlugin,
    "drcsim_gazebo_ros_plugins::ContactModelPlugin")
