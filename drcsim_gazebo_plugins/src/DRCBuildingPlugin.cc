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

#include "drcsim_gazebo_plugins/DRCBuildingPlugin.hh"

#include <chrono>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>

using namespace drcsim_gazebo_plugins;

//////////////////////////////////////////////////
void DRCBuildingPlugin::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "DRCBuildingPlugin should be attached to a model entity. "
          << "Failed to initialize.\n";
    return;
  }

  if (!_sdf->HasElement("door_joint"))
  {
    gzerr << "<door_joint> is required, but was not found.\n";
    return;
  }
  std::string doorJointName = _sdf->Get<std::string>("door_joint");
  this->doorJoint = this->model.JointByName(_ecm, doorJointName);
  if (this->doorJoint == gz::sim::kNullEntity)
  {
    gzerr << "<door_joint>" << doorJointName
          << "</door_joint> does not exist\n";
    return;
  }

  if (!_sdf->HasElement("handle_joint"))
  {
    gzerr << "<handle_joint> is required, but was not found.\n";
    return;
  }
  std::string handleJointName = _sdf->Get<std::string>("handle_joint");
  this->handleJoint = this->model.JointByName(_ecm, handleJointName);
  if (this->handleJoint == gz::sim::kNullEntity)
  {
    gzerr << "<handle_joint>" << handleJointName
          << "</handle_joint> does not exist\n";
    return;
  }

  gz::sim::Joint(this->doorJoint).EnablePositionCheck(_ecm);
  gz::sim::Joint(this->handleJoint).EnablePositionCheck(_ecm);

  this->doorPID.Init(200, 1, 20, 10, -10, 50, -50);
  this->handlePID.Init(80, 1, 1, 3, -3, 5, -5);

  this->validConfig = true;
}

//////////////////////////////////////////////////
void DRCBuildingPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !this->validConfig)
    return;

  std::chrono::duration<double> dt(_info.dt);
  if (dt.count() <= 0)
    return;

  gz::sim::Joint doorJointWrapper(this->doorJoint);
  gz::sim::Joint handleJointWrapper(this->handleJoint);

  this->doorState = doorJointWrapper.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];
  this->handleState = handleJointWrapper.Position(_ecm).value_or(
      std::vector<double>{0.0})[0];

  // PID (position) door
  double doorError = this->doorState - this->doorCmd;
  double doorCmdForce = this->doorPID.Update(doorError, dt);
  doorJointWrapper.SetForce(_ecm, {doorCmdForce});

  // PID (position) handle
  double handleError = this->handleState - this->handleCmd;
  double handleCmdForce = this->handlePID.Update(handleError, dt);
  handleJointWrapper.SetForce(_ecm, {handleCmdForce});
}

//////////////////////////////////////////////////
void DRCBuildingPlugin::SetDoorState(double _angle)
{
  this->doorCmd = _angle;
}

//////////////////////////////////////////////////
double DRCBuildingPlugin::GetDoorState() const
{
  return this->doorState;
}

GZ_ADD_PLUGIN(DRCBuildingPlugin,
              gz::sim::System,
              DRCBuildingPlugin::ISystemConfigure,
              DRCBuildingPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DRCBuildingPlugin,
    "drcsim_gazebo_plugins::DRCBuildingPlugin")
