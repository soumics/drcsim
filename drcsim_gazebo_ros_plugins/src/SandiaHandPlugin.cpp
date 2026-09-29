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
#include "drcsim_gazebo_ros_plugins/SandiaHandPlugin.hpp"

#include <gz/msgs/contacts.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>

#include "drcsim_gazebo_ros_plugins/RosNodeOptions.hpp"

using drcsim_gazebo_ros_plugins::SandiaHandPlugin;

//////////////////////////////////////////////////
SandiaHandPlugin::SandiaHandPlugin()
{
  this->jointDampingMin.assign(kNumJoints, 1.0);
  this->jointDampingMax.assign(kNumJoints, 30.0);
  this->jointDampingCurrent.assign(kNumJoints, 1.0);
}

//////////////////////////////////////////////////
SandiaHandPlugin::~SandiaHandPlugin()
{
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->rosSpinThread.joinable()) {
    this->rosSpinThread.join();
  }
}

//////////////////////////////////////////////////
void SandiaHandPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "SandiaHandPlugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->sdfConfig = _sdf;
}

//////////////////////////////////////////////////
void SandiaHandPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized && this->sdfConfig) {
    // Deferred from Configure(): a model's child link/collision/joint
    // entities are not guaranteed to exist yet when Configure() runs.
    this->Load(_ecm);
    this->initialized = true;
  }

  if (this->validConfig) {
    this->UpdateStates(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void SandiaHandPlugin::Load(gz::sim::EntityComponentManager & _ecm)
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

  gzmsg << "SandiaHandPlugin loading for " << this->side << " hand."
        << std::endl;

  this->imuLinkName = this->side.substr(0, 1) + "_hand";

  this->jointNames.clear();
  for (unsigned int f = 0; f < 4; ++f) {
    for (unsigned int j = 0; j < 3; ++j) {
      this->jointNames.push_back(
        this->side + "_f" + std::to_string(f) + "_j" + std::to_string(j));
    }
  }

  this->jointEntities.resize(this->jointNames.size(), gz::sim::kNullEntity);
  unsigned int jointCount = 0;
  for (unsigned int i = 0; i < this->jointNames.size(); ++i) {
    this->jointEntities[i] = this->model.JointByName(_ecm, this->jointNames[i]);
    if (this->jointEntities[i] != gz::sim::kNullEntity) {
      gz::sim::Joint(this->jointEntities[i]).EnablePositionCheck(_ecm);
      gz::sim::Joint(this->jointEntities[i]).EnableVelocityCheck(_ecm);
      ++jointCount;
    }
  }
  if (jointCount == 0) {
    this->hasStumps = true;
    gzmsg << "No sandia hand joints found, loading as stumps" << std::endl;
  } else if (jointCount != this->jointEntities.size()) {
    gzerr << "Error loading sandia hand joints, plugin not loaded"
          << std::endl;
    return;
  }

  this->errorTerms.resize(this->jointNames.size());
  this->jointStates.name = this->jointNames;
  this->jointStates.position.assign(this->jointNames.size(), 0.0);
  this->jointStates.velocity.assign(this->jointNames.size(), 0.0);
  this->jointStates.effort.assign(this->jointNames.size(), 0.0);

  this->jointCommands.name = this->jointNames;
  this->jointCommands.position.assign(this->jointNames.size(), 0.0);
  this->jointCommands.velocity.assign(this->jointNames.size(), 0.0);
  this->jointCommands.effort.assign(this->jointNames.size(), 0.0);
  this->jointCommands.kp_position.assign(this->jointNames.size(), 0.0);
  this->jointCommands.ki_position.assign(this->jointNames.size(), 0.0);
  this->jointCommands.kd_position.assign(this->jointNames.size(), 0.0);
  this->jointCommands.kp_velocity.assign(this->jointNames.size(), 0.0);
  this->jointCommands.i_effort_min.assign(this->jointNames.size(), 0.0);
  this->jointCommands.i_effort_max.assign(this->jointNames.size(), 0.0);

  // Get IMU link and enable the velocity/acceleration checks its world-frame
  // state readout needs (see the class-level design note on IMU handling).
  this->imuLinkEntity = this->model.LinkByName(_ecm, this->imuLinkName);
  if (this->imuLinkEntity == gz::sim::kNullEntity) {
    gzerr << this->imuLinkName << " not found" << std::endl;
  } else {
    gz::sim::Link imuLink(this->imuLinkEntity);
    imuLink.EnableVelocityChecks(_ecm);
    imuLink.EnableAccelerationChecks(_ecm);
  }

  // Tactile sensor output range, approximated from the physical hand.
  this->maxTactileOut = 33500;
  this->minTactileOut = 26500;
  this->tactileFingerArraySize = 18;
  this->tactilePalmArraySize = 32;
  this->tactile.f0.assign(this->tactileFingerArraySize, this->minTactileOut);
  this->tactile.f1.assign(this->tactileFingerArraySize, this->minTactileOut);
  this->tactile.f2.assign(this->tactileFingerArraySize, this->minTactileOut);
  this->tactile.f3.assign(this->tactileFingerArraySize, this->minTactileOut);
  this->tactile.palm.assign(this->tactilePalmArraySize, this->minTactileOut);

  if (!this->hasStumps) {
    // Sandia hand tactile dimensions taken from spec and adapted to fit on
    // our sandia hand model.
    this->palmColWidth[0] = 0.01495;
    this->palmColLength[0] = 0.02341;
    this->palmColWidth[1] = 0.01495;
    this->palmColLength[1] = 0.02341;
    this->palmColWidth[2] = 0.01495;
    this->palmColLength[2] = 0.02341;
    this->palmColWidth[3] = 0.04304;
    this->palmColLength[3] = 0.05271;
    this->palmColWidth[4] = 0.08004;
    this->palmColLength[4] = 0.01170;

    this->palmHorSize[0] = 2;
    this->palmVerSize[0] = 3;
    this->palmHorSize[1] = 2;
    this->palmVerSize[1] = 3;
    this->palmHorSize[2] = 2;
    this->palmVerSize[2] = 3;
    this->palmHorSize[3] = 2;
    this->palmVerSize[3] = 5;
    this->palmHorSize[4] = 5;
    this->palmVerSize[4] = 2;

    this->fingerColLength[0] = 0.01;
    this->fingerColWidth[0] = 0.0158;
    this->fingerColLength[1] = 0.0271;
    this->fingerColWidth[1] = 0.0134;

    this->fingerHorSize[0] = 3;
    this->fingerVerSize[0] = 2;
    this->fingerHorSize[1] = 3;
    this->fingerVerSize[1] = 4;

    // Force-create ContactSensorData on this hand's own finger/palm
    // collisions (same technique as ContactModelPlugin) and precompute
    // each one's tactile-array bucket, keyed by entity id -- contacts are
    // matched back to these by gz::msgs::Entity id, not name matching.
    auto linkEntities = _ecm.ChildrenByComponents(
      this->model.Entity(), gz::sim::components::Link());
    for (const gz::sim::Entity linkEntity : linkEntities) {
      auto collisionEntities = _ecm.ChildrenByComponents(
        linkEntity, gz::sim::components::Collision());
      for (const gz::sim::Entity collisionEntity : collisionEntities) {
        const std::string & collisionName =
          _ecm.Component<gz::sim::components::Name>(collisionEntity)->Data();

        bool isPalm = collisionName.find("palm") != std::string::npos;
        bool isFinger = collisionName.find(this->side + "_f") !=
          std::string::npos;
        if (!isPalm && !isFinger) {
          continue;
        }

        TactileCollisionInfo info;
        info.isPalm = isPalm;
        if (isPalm) {
          if (collisionName.find("_3") != std::string::npos) {
            info.palmIdx = 0;
          } else if (collisionName.find("_4") != std::string::npos) {
            info.palmIdx = 1;
          } else if (collisionName.find("_5") != std::string::npos) {
            info.palmIdx = 2;
          } else if (collisionName.find("_1") != std::string::npos) {
            info.palmIdx = 3;
          } else {
            info.palmIdx = 4;
          }
        } else {
          if (collisionName.find("f0") != std::string::npos) {
            info.fingerIdx = 0;
          } else if (collisionName.find("f1") != std::string::npos) {
            info.fingerIdx = 1;
          } else if (collisionName.find("f2") != std::string::npos) {
            info.fingerIdx = 2;
          } else if (collisionName.find("f3") != std::string::npos) {
            info.fingerIdx = 3;
          }

          if (collisionName.find("_1") != std::string::npos) {
            info.fingerColIdx = 0;
          } else if (collisionName.find("_2") != std::string::npos) {
            info.fingerColIdx = 1;
          }
        }

        if (!_ecm.EntityHasComponentType(
            collisionEntity, gz::sim::components::ContactSensorData::typeId))
        {
          _ecm.CreateComponent(
            collisionEntity, gz::sim::components::ContactSensorData());
        }
        this->tactileCollisions[collisionEntity] = info;
      }
    }
  }

  // ROS integration: multiple ROS-coupled plugins share one process and one
  // global rclcpp context -- only initialize it if nothing has already,
  // and never tear it down from this instance's destructor.
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  // RosNodeOptionsFromEnv() carries the launch file's params (the finger
  // gains below); without it every gain stayed 0 and the fingers were limp.
  this->rosNode = std::make_shared<rclcpp::Node>(
    "sandia_hand_plugin", "/sandia_hands/" + this->imuLinkName,
    drcsim_gazebo_ros_plugins::RosNodeOptionsFromEnv());

  const int numFingers = 4;
  const int numFingerJoints = 3;
  for (int finger = 0; finger < numFingers; ++finger) {
    for (int joint = 0; joint < numFingerJoints; ++joint) {
      const std::string prefix =
        "gains.f" + std::to_string(finger) + "_j" + std::to_string(joint) + ".";
      const double p = this->rosNode->declare_parameter(prefix + "p", 0.0);
      const double i = this->rosNode->declare_parameter(prefix + "i", 0.0);
      const double d = this->rosNode->declare_parameter(prefix + "d", 0.0);
      const double iClamp =
        this->rosNode->declare_parameter(prefix + "i_clamp", 0.0);
      const int jointIdx = finger * numFingerJoints + joint;
      this->jointCommands.kp_position[jointIdx] = p;
      this->jointCommands.ki_position[jointIdx] = i;
      this->jointCommands.kd_position[jointIdx] = d;
      this->jointCommands.i_effort_min[jointIdx] = -iClamp;
      this->jointCommands.i_effort_max[jointIdx] = iClamp;
    }
  }

  this->pubJointStates =
    this->rosNode->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", 10);
  this->pubImu = this->rosNode->create_publisher<sensor_msgs::msg::Imu>(
    "imu", 10);
  this->pubTactile =
    this->rosNode->create_publisher<sandia_hand_msgs::msg::RawTactile>(
    "tactile_raw", 10);

  this->subJointCommands =
    this->rosNode->create_subscription<osrf_msgs::msg::JointCommands>(
    "joint_commands", 100,
    std::bind(&SandiaHandPlugin::SetJointCommands, this, std::placeholders::_1));

  this->setJointDampingService =
    this->rosNode->create_service<atlas_msgs::srv::SetJointDamping>(
    "set_joint_damping",
    std::bind(
      &SandiaHandPlugin::SetJointDamping, this,
      std::placeholders::_1, std::placeholders::_2));
  this->getJointDampingService =
    this->rosNode->create_service<atlas_msgs::srv::GetJointDamping>(
    "get_joint_damping",
    std::bind(
      &SandiaHandPlugin::GetJointDamping, this,
      std::placeholders::_1, std::placeholders::_2));

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
double SandiaHandPlugin::FirstOrZero(
  const std::optional<std::vector<double>> & _values)
{
  if (_values && !_values->empty()) {
    return (*_values)[0];
  }
  return 0.0;
}

//////////////////////////////////////////////////
void SandiaHandPlugin::CopyVectorIfValid(
  const std::vector<double> & _from, std::vector<double> & _to)
{
  if (_from.empty() || _from.size() != _to.size()) {
    return;
  }
  _to = _from;
}

//////////////////////////////////////////////////
void SandiaHandPlugin::SetJointCommands(
  const osrf_msgs::msg::JointCommands::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->commandMutex);
  // This implementation does not check the ordering of the joints. They
  // must agree with the structure initialized in Load().
  CopyVectorIfValid(_msg->position, this->jointCommands.position);
  CopyVectorIfValid(_msg->velocity, this->jointCommands.velocity);
  CopyVectorIfValid(_msg->effort, this->jointCommands.effort);
  CopyVectorIfValid(_msg->kp_position, this->jointCommands.kp_position);
  CopyVectorIfValid(_msg->ki_position, this->jointCommands.ki_position);
  CopyVectorIfValid(_msg->kd_position, this->jointCommands.kd_position);
  CopyVectorIfValid(_msg->kp_velocity, this->jointCommands.kp_velocity);
  CopyVectorIfValid(_msg->i_effort_min, this->jointCommands.i_effort_min);
  CopyVectorIfValid(_msg->i_effort_max, this->jointCommands.i_effort_max);
}

//////////////////////////////////////////////////
void SandiaHandPlugin::SetJointDamping(
  const std::shared_ptr<atlas_msgs::srv::SetJointDamping::Request> _req,
  std::shared_ptr<atlas_msgs::srv::SetJointDamping::Response> _res)
{
  _res->success = true;
  std::string statusMessage;
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    for (unsigned int i = 0; i < this->jointNames.size(); ++i) {
      const double requested = _req->damping_coefficients[i];
      const double clamped = std::clamp(
        requested, this->jointDampingMin[i], this->jointDampingMax[i]);
      // gz-sim has no well-supported runtime joint-damping mutation API
      // (same situation as DRCVehiclePlugin's dropped Set*Limits) -- only
      // the cached value is updated, not physics.
      this->jointDampingCurrent[i] = clamped;
      if (std::abs(clamped - requested) > 1e-9) {
        statusMessage += "requested joint damping for joint [" +
          this->jointNames[i] + "] of [" + std::to_string(requested) +
          "] is truncated to [" + std::to_string(clamped) + "].\n";
        _res->success = false;
      }
    }
  }
  if (!statusMessage.empty()) {
    RCLCPP_WARN(this->rosNode->get_logger(), "%s", statusMessage.c_str());
  }
  _res->status_message = statusMessage;
}

//////////////////////////////////////////////////
void SandiaHandPlugin::GetJointDamping(
  const std::shared_ptr<atlas_msgs::srv::GetJointDamping::Request>,
  std::shared_ptr<atlas_msgs::srv::GetJointDamping::Response> _res)
{
  _res->success = true;
  _res->status_message = "success";
  std::lock_guard<std::mutex> lock(this->commandMutex);
  for (unsigned int i = 0; i < this->jointNames.size(); ++i) {
    _res->damping_coefficients[i] = this->jointDampingCurrent[i];
    _res->damping_coefficients_max[i] = this->jointDampingMax[i];
    _res->damping_coefficients_min[i] = this->jointDampingMin[i];
  }
}

//////////////////////////////////////////////////
void SandiaHandPlugin::UpdateStates(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (_info.simTime <= this->lastControllerUpdateTime) {
    return;
  }

  // IMU: read the link's raw physics state and rotate world-frame vectors
  // into the link's own frame (see the class-level design note).
  if (this->imuLinkEntity != gz::sim::kNullEntity) {
    gz::sim::Link imuLink(this->imuLinkEntity);
    auto worldPose = imuLink.WorldPose(_ecm);
    auto angularVel = imuLink.WorldAngularVelocity(_ecm);
    auto linearAcc = imuLink.WorldLinearAcceleration(_ecm);
    if (worldPose && angularVel && linearAcc) {
      const gz::math::Vector3d bodyAngularVel =
        worldPose->Rot().RotateVectorReverse(*angularVel);
      const gz::math::Vector3d bodyLinearAcc =
        worldPose->Rot().RotateVectorReverse(*linearAcc);

      sensor_msgs::msg::Imu imuMsg;
      imuMsg.header.frame_id = this->imuLinkName;
      imuMsg.header.stamp = rclcpp::Time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          _info.simTime).count());

      imuMsg.angular_velocity.x = bodyAngularVel.X();
      imuMsg.angular_velocity.y = bodyAngularVel.Y();
      imuMsg.angular_velocity.z = bodyAngularVel.Z();

      imuMsg.linear_acceleration.x = bodyLinearAcc.X();
      imuMsg.linear_acceleration.y = bodyLinearAcc.Y();
      imuMsg.linear_acceleration.z = bodyLinearAcc.Z();

      imuMsg.orientation.x = worldPose->Rot().X();
      imuMsg.orientation.y = worldPose->Rot().Y();
      imuMsg.orientation.z = worldPose->Rot().Z();
      imuMsg.orientation.w = worldPose->Rot().W();

      this->pubImu->publish(imuMsg);
    }
  }

  const auto stamp = rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count());

  if (!this->hasStumps) {
    for (unsigned int i = 0; i < this->jointEntities.size(); ++i) {
      gz::sim::Joint joint(this->jointEntities[i]);
      this->jointStates.position[i] = FirstOrZero(joint.Position(_ecm));
      this->jointStates.velocity[i] = FirstOrZero(joint.Velocity(_ecm));
    }
  }
  this->jointStates.header.stamp = stamp;

  const double dt = std::chrono::duration<double>(
    _info.simTime - this->lastControllerUpdateTime).count();

  if (!this->hasStumps) {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    for (unsigned int i = 0; i < this->jointEntities.size(); ++i) {
      const double position = this->jointStates.position[i];
      const double velocity = this->jointStates.velocity[i];

      const double qP = this->jointCommands.position[i] - position;
      if (dt > 0.0) {
        this->errorTerms[i].positionErrorDerivative =
          (qP - this->errorTerms[i].positionError) / dt;
      }
      this->errorTerms[i].positionError = qP;
      this->errorTerms[i].velocityError =
        this->jointCommands.velocity[i] - velocity;
      this->errorTerms[i].positionErrorIntegral = std::clamp(
        this->errorTerms[i].positionErrorIntegral + dt * qP,
        this->jointCommands.i_effort_min[i],
        this->jointCommands.i_effort_max[i]);

      const double force =
        this->jointCommands.kp_position[i] * this->errorTerms[i].positionError +
        this->jointCommands.kp_velocity[i] * this->errorTerms[i].velocityError +
        this->jointCommands.ki_position[i] *
        this->errorTerms[i].positionErrorIntegral +
        this->jointCommands.kd_position[i] *
        this->errorTerms[i].positionErrorDerivative +
        this->jointCommands.effort[i];

      this->jointStates.effort[i] = force;
      gz::sim::Joint(this->jointEntities[i]).SetForce(_ecm, {force});
    }
  }
  this->pubJointStates->publish(this->jointStates);

  // Tactile data: only do the (nontrivial) contact-processing work if
  // someone is actually listening.
  if (this->pubTactile->get_subscription_count() > 0 && !this->hasStumps) {
    this->tactile.f0.assign(this->tactileFingerArraySize, this->minTactileOut);
    this->tactile.f1.assign(this->tactileFingerArraySize, this->minTactileOut);
    this->tactile.f2.assign(this->tactileFingerArraySize, this->minTactileOut);
    this->tactile.f3.assign(this->tactileFingerArraySize, this->minTactileOut);
    this->tactile.palm.assign(this->tactilePalmArraySize, this->minTactileOut);

    std::vector<gz::msgs::Contact> contacts;
    for (const auto & entry : this->tactileCollisions) {
      const auto * contactData =
        _ecm.Component<gz::sim::components::ContactSensorData>(entry.first);
      if (!contactData) {
        continue;
      }
      for (const auto & contact : contactData->Data().contact()) {
        contacts.push_back(contact);
      }
    }

    this->tactile.header.stamp = stamp;
    this->FillTactileData(_ecm, contacts, this->tactile);
    this->pubTactile->publish(this->tactile);
  }

  this->lastControllerUpdateTime = _info.simTime;
}

//////////////////////////////////////////////////
void SandiaHandPlugin::FillTactileData(
  const gz::sim::EntityComponentManager & _ecm,
  const std::vector<gz::msgs::Contact> & _contacts,
  sandia_hand_msgs::msg::RawTactile & _tactileMsg)
{
  // The method of generating tactile sensor output is specific to the
  // current sandia hand collisions. This is because the collisions do not
  // really match the actual sandia hand so it was not possible to directly
  // use the position of the sensors from the spec sheet. The best we could
  // do is approximate the locations of tactile sensors on these collisions.
  // Idea: divide each collision into smaller regions, identify the region
  // the contact point lies in, and set the corresponding (closest) tactile
  // sensor's output value.
  for (const gz::msgs::Contact & contact : _contacts) {
    const auto entry1 = this->tactileCollisions.find(
      static_cast<gz::sim::Entity>(contact.collision1().id()));
    const auto entry2 = this->tactileCollisions.find(
      static_cast<gz::sim::Entity>(contact.collision2().id()));

    gz::sim::Entity collisionEntity{gz::sim::kNullEntity};
    TactileCollisionInfo info;
    bool isBody1 = true;
    if (entry1 != this->tactileCollisions.end()) {
      collisionEntity = entry1->first;
      info = entry1->second;
      isBody1 = true;
    } else if (entry2 != this->tactileCollisions.end()) {
      collisionEntity = entry2->first;
      info = entry2->second;
      isBody1 = false;
    } else {
      continue;
    }

    const gz::math::Pose3d colWorldPose =
      gz::sim::worldPose(collisionEntity, _ecm);

    for (int j = 0; j < contact.position_size(); ++j) {
      gz::math::Vector3d pos = gz::msgs::Convert(contact.position(j));
      gz::math::Vector3d force = isBody1 ?
        gz::msgs::Convert(contact.wrench(j).body_1_wrench().force()) :
        gz::msgs::Convert(contact.wrench(j).body_2_wrench().force());

      // Scaling formula taken from Gazebo Classic's ContactVisual class.
      const int tactileOutput = static_cast<int>(
        (2.0 * (this->maxTactileOut - this->minTactileOut)) /
        (1 + std::exp(-force.SquaredLength() / 100)) -
        (this->maxTactileOut - 2 * this->minTactileOut));

      // Transform into the collision's own frame.
      pos = colWorldPose.Rot().RotateVectorReverse(pos - colWorldPose.Pos());

      double vPosInCol = 0.0;
      double hPosInCol = 0.0;
      int ai = 0;
      int aj = 0;
      int aIndex = -1;

      if (info.isPalm) {
        switch (info.palmIdx) {
          case 0:
            // Index finger palm sensors: 3; 8 9; 13
            if (pos.Z() > 0) {
              vPosInCol = std::clamp(
                pos.X() / this->palmColLength[info.palmIdx], 0.0, 1.0);
              hPosInCol = std::clamp(
                (-pos.Y() + this->palmColWidth[info.palmIdx] / 2.0) /
                this->palmColWidth[0], 0.0, 1.0);

              ai = this->palmVerSize[info.palmIdx] -
                std::ceil(vPosInCol * this->palmVerSize[info.palmIdx]) - 1;
              aj = std::ceil(hPosInCol * this->palmHorSize[info.palmIdx]) - 1;
              ai = std::max(ai, 0);
              aj = std::max(aj, 0);
              aIndex = 2;
              if (ai == 1) {
                aIndex = (aj == 0) ? 7 : 8;
              } else if (ai == 2) {
                aIndex = 12;
              }
              _tactileMsg.palm[aIndex] = tactileOutput;
            }
            break;
          case 1:
            // Middle finger palm sensors: 2; 6 7; 11 12
            if (pos.Z() > 0) {
              vPosInCol = std::clamp(
                pos.X() / this->palmColLength[info.palmIdx], 0.0, 1.0);
              hPosInCol = std::clamp(
                (-pos.Y() + this->palmColWidth[info.palmIdx] / 2.0) /
                this->palmColWidth[info.palmIdx], 0.0, 1.0);
              ai = this->palmVerSize[info.palmIdx] -
                std::ceil(vPosInCol * this->palmVerSize[info.palmIdx]) - 1;
              aj = std::ceil(hPosInCol * this->palmHorSize[info.palmIdx]) - 1;
              ai = std::max(ai, 0);
              aj = std::max(aj, 0);
              aIndex = 1;
              if (ai == 1) {
                aIndex = (aj == 0) ? 5 : 6;
              } else if (ai == 2) {
                aIndex = (aj == 0) ? 10 : 11;
              }
              _tactileMsg.palm[aIndex] = tactileOutput;
            }
            break;
          case 2:
            // Pinky palm sensors: 1; 4 5; 10
            if (pos.Z() > 0) {
              vPosInCol = std::clamp(
                pos.X() / this->palmColLength[info.palmIdx], 0.0, 1.0);
              hPosInCol = std::clamp(
                (-pos.Y() + this->palmColWidth[info.palmIdx] / 2.0) /
                this->palmColWidth[info.palmIdx], 0.0, 1.0);

              ai = this->palmVerSize[info.palmIdx] -
                std::ceil(vPosInCol * this->palmVerSize[info.palmIdx]) - 1;
              aj = std::ceil(hPosInCol * this->palmHorSize[info.palmIdx]) - 1;
              ai = std::max(ai, 0);
              aj = std::max(aj, 0);
              aIndex = 0;
              if (ai == 1) {
                aIndex = (aj == 0) ? 3 : 4;
              } else if (ai == 2) {
                aIndex = 9;
              }
              _tactileMsg.palm[aIndex] = tactileOutput;
            }
            break;
          case 3:
            {
              // Sensors on bottom palm: 23 24; 25 26; 27 28; 29 30; 31 32
              const int baseIndex = 22;
              if (pos.Z() > 0) {
                vPosInCol = std::clamp(
                  (pos.Y() + this->palmColLength[info.palmIdx] / 2.0) /
                this->palmColLength[info.palmIdx], 0.0, 1.0);
                hPosInCol = std::clamp(
                  (pos.X() + this->palmColWidth[info.palmIdx] / 2.0) /
                this->palmColWidth[info.palmIdx], 0.0, 1.0);

                ai = this->palmVerSize[info.palmIdx] -
                  std::ceil(vPosInCol * this->palmVerSize[info.palmIdx]) - 1;
                aj = std::ceil(hPosInCol * this->palmHorSize[info.palmIdx]) - 1;
                ai = std::max(ai, 0);
                aj = std::max(aj, 0);
                aIndex = baseIndex + ai * this->palmHorSize[info.palmIdx] + aj;
                _tactileMsg.palm[aIndex] = tactileOutput;
              }
              break;
            }
          default:
            {
            // Sensors on mid palm (default): 14 15 16 17; 18 19 20 21 22
              const int baseIndex = 13;
              vPosInCol = std::clamp(
              pos.Y() / this->palmColLength[4], 0.0, 1.0);
              hPosInCol = std::clamp(
                (pos.Z() + this->palmColWidth[4] / 2.0) /
              this->palmColWidth[4], 0.0, 1.0);

              ai = this->palmVerSize[4] -
                std::ceil(vPosInCol * this->palmVerSize[4]) - 1;
              aj = std::ceil(hPosInCol * this->palmHorSize[4]) - 1;
              ai = std::max(ai, 0);
              aj = std::max(aj, 0);
              if (ai == 0) {
                // Four sensors on the first row, five on the second, so
                // adjust aj for sensors 16 and 17.
                aj = (aj > 2) ? aj - 1 : aj;
                aIndex = baseIndex + aj;
              } else {
                aIndex = baseIndex + ai * (this->palmHorSize[4] - 1) + aj;
              }
              _tactileMsg.palm[aIndex] = tactileOutput;
              break;
            }
        }
      } else if (info.fingerIdx != -1 && info.fingerColIdx != -1 && pos.Y() > 0) {
        // Finger, and the contact is on the inside of the hand (palm side).
        vPosInCol = std::clamp(
          (pos.Z() + this->fingerColLength[info.fingerColIdx] / 2) /
          this->fingerColLength[info.fingerColIdx], 0.0, 1.0);
        hPosInCol = std::clamp(
          (-pos.X() + this->fingerColWidth[info.fingerColIdx] / 2) /
          this->fingerColWidth[info.fingerColIdx], 0.0, 1.0);

        ai = this->fingerVerSize[info.fingerColIdx] -
          std::ceil(vPosInCol * this->fingerVerSize[info.fingerColIdx]) - 1;
        aj = std::ceil(hPosInCol * this->fingerHorSize[info.fingerColIdx]) - 1;
        ai = std::max(ai, 0);
        aj = std::max(aj, 0);

        aIndex = info.fingerColIdx * this->fingerHorSize[0] * this->fingerVerSize[0] +
          ai * this->fingerHorSize[info.fingerColIdx] + aj;

        switch (info.fingerIdx) {
          case 0:
            _tactileMsg.f0[aIndex] = tactileOutput;
            break;
          case 1:
            _tactileMsg.f1[aIndex] = tactileOutput;
            break;
          case 2:
            _tactileMsg.f2[aIndex] = tactileOutput;
            break;
          case 3:
            _tactileMsg.f3[aIndex] = tactileOutput;
            break;
          default:
            break;
        }
      }
    }
  }
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(SandiaHandPlugin,
              gz::sim::System,
              SandiaHandPlugin::ISystemConfigure,
              SandiaHandPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(SandiaHandPlugin,
    "drcsim_gazebo_ros_plugins::SandiaHandPlugin")
