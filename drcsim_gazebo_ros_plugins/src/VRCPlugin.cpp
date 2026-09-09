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
#include "drcsim_gazebo_ros_plugins/VRCPlugin.hpp"

#include <cmath>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Root.hh>

#include <atlas_msgs/msg/atlas_behavior_step_data.hpp>

#include "drcsim_gazebo_ros_plugins/RosNodeOptions.hpp"

using drcsim_gazebo_ros_plugins::VRCPlugin;

//////////////////////////////////////////////////
VRCPlugin::VRCPlugin()
{
}

//////////////////////////////////////////////////
VRCPlugin::~VRCPlugin()
{
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->rosSpinThread.joinable()) {
    this->rosSpinThread.join();
  }
}

//////////////////////////////////////////////////
double VRCPlugin::FirstOrZero(
  const std::optional<std::vector<double>> & _values)
{
  if (_values && !_values->empty()) {
    return (*_values)[0];
  }
  return 0.0;
}

//////////////////////////////////////////////////
void VRCPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & _eventMgr)
{
  this->world = gz::sim::World(_entity);
  if (!this->world.Valid(_ecm)) {
    gzerr << "VRCPlugin should be attached to a world entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  // _sdf is a const pointer; clone it into a mutable one since the plugin's
  // nested SDF blocks (<atlas>, <drc_vehicle>, <drc_fire_hose>) need
  // GetElement(), which is non-const in this sdformat version.
  this->sdfConfig = _sdf->Clone();
  this->eventMgr = &_eventMgr;
}

//////////////////////////////////////////////////
void VRCPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized && this->sdfConfig) {
    this->DeferredLoad(_ecm);
    this->initialized = true;
  }

  if (this->validConfig) {
    this->UpdateStates(_info, _ecm, *this->eventMgr);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::DeferredLoad(gz::sim::EntityComponentManager & _ecm)
{
  // By default, cheats are off. Allow override via environment variable.
  const char * cheatsEnabledString = std::getenv("VRC_CHEATS_ENABLED");
  this->cheatsEnabled =
    cheatsEnabledString && (std::string(cheatsEnabledString) == "1");

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->rosNode = std::make_shared<rclcpp::Node>(
    "vrc_plugin", drcsim_gazebo_ros_plugins::RosNodeOptionsFromEnv());

  this->LoadVRCROSAPI();

  this->robotCmdVel = geometry_msgs::msg::Twist();

  this->drcVehicle.Load(this->world, _ecm, this->sdfConfig);
  this->drcFireHose.Load(this->world, _ecm, this->sdfConfig);

  this->LoadRobotROSAPI();

  this->cmdVelTopicTimeout =
    this->rosNode->declare_parameter("cmd_vel_timeout", 0.1);

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
gz::sim::Entity VRCPlugin::AddJoint(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
  gz::sim::Entity _modelEntity, gz::sim::Entity _link1, gz::sim::Entity _link2)
{
  static_cast<void>(_modelEntity);
  static_cast<void>(_eventMgr);
  if (_link1 == gz::sim::kNullEntity) {
    // Pin _link2 to the world -- no physics joint at all; see the
    // class-level design note for why (three different joint-based
    // mechanisms were each tried and found to either not detach on
    // removal, or to actively destabilize the robot). The actual "hold in
    // place" behavior is a per-tick kinematic pose override in
    // UpdateStates(), driven by atlas.pinHoldPose, for as long as
    // pinJointEntity != kNullEntity. This placeholder entity carries no
    // components at all -- it exists purely so pinJointEntity's existing
    // "!= kNullEntity means pinned" contract, and RemoveJoint()'s existing
    // RequestRemoveEntity() call, keep working unchanged.
    return _ecm.CreateEntity();
  }

  // Cross-model (or same-model) rigid weld between two existing links.
  const gz::sim::Entity jointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(
    jointEntity,
    gz::sim::components::DetachableJoint({_link1, _link2, "fixed"}));
  return jointEntity;
}

//////////////////////////////////////////////////
void VRCPlugin::RemoveJoint(
  gz::sim::EntityComponentManager & _ecm, gz::sim::Entity & _joint)
{
  if (_joint != gz::sim::kNullEntity) {
    _ecm.RequestRemoveEntity(_joint);
    _joint = gz::sim::kNullEntity;
  }
}

//////////////////////////////////////////////////
gz::math::Pose3d VRCPlugin::ModelPoseForLinkWorldPose(
  const gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _modelEntity,
  gz::sim::Entity _linkEntity, const gz::math::Pose3d & _desiredLinkPose)
{
  const gz::math::Pose3d currentModelPose =
    gz::sim::worldPose(_modelEntity, _ecm);
  const gz::math::Pose3d currentLinkPose =
    gz::sim::worldPose(_linkEntity, _ecm);
  const gz::math::Pose3d linkRelPose =
    currentModelPose.Inverse() * currentLinkPose;
  return _desiredLinkPose * linkRelPose.Inverse();
}

//////////////////////////////////////////////////
void VRCPlugin::SetLinkWorldPose(
  gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _modelEntity,
  gz::sim::Entity _linkEntity, const gz::math::Pose3d & _pose)
{
  gz::sim::Model model(_modelEntity);
  model.SetWorldPoseCmd(
    _ecm, ModelPoseForLinkWorldPose(_ecm, _modelEntity, _linkEntity, _pose));
}

//////////////////////////////////////////////////
void VRCPlugin::Teleport(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
  gz::sim::Entity _pinLink, gz::sim::Entity & _pinJoint,
  const gz::math::Pose3d & _pose)
{
  this->RemoveJoint(_ecm, _pinJoint);
  const gz::sim::Entity modelEntity = _ecm.ParentEntity(_pinLink);
  this->SetLinkWorldPose(_ecm, modelEntity, _pinLink, _pose);
  _pinJoint = this->AddJoint(
    _ecm, _eventMgr, modelEntity, gz::sim::kNullEntity, _pinLink);
  // Teleport() is only ever called for atlas.pinLinkEntity/pinJointEntity
  // (the cmd_vel warp-while-pinned path) -- keep the per-tick kinematic
  // hold (see the class-level design note) targeting the new pose too,
  // not the original pin pose.
  this->atlas.pinHoldPose = _pose;
}

//////////////////////////////////////////////////
void VRCPlugin::SetLinkGravityMode(gz::sim::Entity _linkEntity, bool _enabled)
{
  if (_enabled) {
    this->gravityDisabledLinks.erase(_linkEntity);
  } else {
    this->gravityDisabledLinks.insert(_linkEntity);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::SetModelGravityMode(
  gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _modelEntity,
  bool _enabled)
{
  const std::vector<gz::sim::Entity> links = _ecm.ChildrenByComponents(
    _modelEntity, gz::sim::components::Link());
  for (const gz::sim::Entity link : links) {
    this->SetLinkGravityMode(link, _enabled);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::ApplyGravityCompensation(gz::sim::EntityComponentManager & _ecm)
{
  const auto gravity = this->world.Gravity(_ecm);
  if (!gravity) {
    return;
  }
  for (const gz::sim::Entity linkEntity : this->gravityDisabledLinks) {
    gz::sim::Link link(linkEntity);
    const auto inertial = link.WorldInertial(_ecm);
    if (!inertial) {
      continue;
    }
    const double mass = inertial->MassMatrix().Mass();
    link.AddWorldForce(_ecm, -mass * (*gravity));
  }
}

//////////////////////////////////////////////////
void VRCPlugin::SetFeetCollide(const std::string &)
{
  // No component-flag equivalent for runtime collide-mode toggling exists
  // in gz-sim (see the class-level design note) -- this is now a
  // documented no-op. Feet may generate brief spurious contacts during
  // pin/unpin/teleport transitions that the original avoided; this is a
  // cosmetic regression, not a functional one.
}

//////////////////////////////////////////////////
void VRCPlugin::PinAtlas(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
  bool _withGravity)
{
  this->RemoveJoint(_ecm, this->vehicleRobotJoint);
  if (this->atlas.pinJointEntity == gz::sim::kNullEntity) {
    this->atlas.pinJointEntity = this->AddJoint(
      _ecm, _eventMgr, this->atlas.modelEntity, gz::sim::kNullEntity,
      this->atlas.pinLinkEntity);
  }
  this->atlas.initialPose = gz::sim::worldPose(this->atlas.pinLinkEntity, _ecm);
  this->atlas.pinHoldPose = this->atlas.initialPose;

  this->SetModelGravityMode(_ecm, this->atlas.modelEntity, _withGravity);
  this->SetFeetCollide("none");
}

//////////////////////////////////////////////////
void VRCPlugin::UnpinAtlas(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr)
{
  this->warpRobotWithCmdVel = false;
  this->SetModelGravityMode(_ecm, this->atlas.modelEntity, true);
  this->RemoveJoint(_ecm, this->atlas.pinJointEntity);
  this->RemoveJoint(_ecm, this->vehicleRobotJoint);
  this->SetFeetCollide("all");
  static_cast<void>(_eventMgr);
}

//////////////////////////////////////////////////
void VRCPlugin::SetRobotModeTopic(const std_msgs::msg::String::SharedPtr _str)
{
  this->SetRobotMode(_str->data);
}

//////////////////////////////////////////////////
void VRCPlugin::SetRobotMode(const std::string & _str)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->pendingMode = _str;
  this->pendingSetRobotMode = true;
}

//////////////////////////////////////////////////
void VRCPlugin::DoSetRobotMode(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
  const std::string & _str)
{
  if (_str == "no_gravity") {
    this->warpRobotWithCmdVel = false;
    this->SetModelGravityMode(_ecm, this->atlas.modelEntity, false);
    this->RemoveJoint(_ecm, this->atlas.pinJointEntity);
    this->RemoveJoint(_ecm, this->vehicleRobotJoint);
  } else if (_str == "feet") {
    this->warpRobotWithCmdVel = false;
    this->SetModelGravityMode(_ecm, this->atlas.modelEntity, false);
    gz::sim::Model atlasModel(this->atlas.modelEntity);
    const gz::sim::Entity lFoot = atlasModel.LinkByName(_ecm, "l_foot");
    const gz::sim::Entity rFoot = atlasModel.LinkByName(_ecm, "r_foot");
    if (lFoot != gz::sim::kNullEntity) {
      this->SetLinkGravityMode(lFoot, true);
    }
    if (rFoot != gz::sim::kNullEntity) {
      this->SetLinkGravityMode(rFoot, true);
    }
    this->RemoveJoint(_ecm, this->atlas.pinJointEntity);
    this->RemoveJoint(_ecm, this->vehicleRobotJoint);
  } else if (_str == "pinned") {
    this->PinAtlas(_ecm, _eventMgr, false);
  } else if (_str == "pinned_with_gravity") {
    this->PinAtlas(_ecm, _eventMgr, true);
  } else if (_str == "nominal") {
    this->UnpinAtlas(_ecm, _eventMgr);
  } else if (_str == "harnessed") {
    // "harnessed" relied on Classic's downward-raycast ground-height query
    // (Entity::GetNearestEntityBelow()). gz-sim's raycast mechanism
    // (components::RaycastData) is real but its result carries no
    // entity-identity, and this mode was already excluded from the
    // original's own "available modes" help text below -- and the
    // original itself already falls back to a flat ground assumption
    // whenever the raycast finds nothing, so this port always takes that
    // same fallback (see the class-level design note).
    this->RemoveJoint(_ecm, this->atlas.pinJointEntity);
    this->RemoveJoint(_ecm, this->vehicleRobotJoint);
    gz::math::Pose3d atlasPose =
      gz::sim::worldPose(this->atlas.pinLinkEntity, _ecm);
    atlasPose.Pos().Z() = 1.15;
    atlasPose.Rot() = gz::math::Quaterniond::Identity;
    this->SetLinkWorldPose(
      _ecm, this->atlas.modelEntity, this->atlas.pinLinkEntity, atlasPose);
    this->atlas.pinJointEntity = this->AddJoint(
      _ecm, _eventMgr, this->atlas.modelEntity, gz::sim::kNullEntity,
      this->atlas.pinLinkEntity);
    this->atlas.initialPose = gz::sim::worldPose(this->atlas.pinLinkEntity, _ecm);
    this->atlas.pinHoldPose = atlasPose;
    this->SetModelGravityMode(_ecm, this->atlas.modelEntity, false);
  } else if (_str == "pid_stand") {
    // Robot is PID controlled in BDI stand Pose and PINNED.
    this->RemoveJoint(_ecm, this->atlas.pinJointEntity);
    this->RemoveJoint(_ecm, this->vehicleRobotJoint);
    // Capture before AddJoint(): the per-tick kinematic hold (see the
    // class-level design note) needs the pose to hold set before it can
    // start being applied, same as PinAtlas()/the "harnessed" branch above.
    this->atlas.pinHoldPose = gz::sim::worldPose(this->atlas.pinLinkEntity, _ecm);
    this->atlas.pinJointEntity = this->AddJoint(
      _ecm, _eventMgr, this->atlas.modelEntity, gz::sim::kNullEntity,
      this->atlas.pinLinkEntity);
    this->SetModelGravityMode(_ecm, this->atlas.modelEntity, false);

    this->atlasCommandController.SetPIDStand();
    RCLCPP_INFO(this->rosNode->get_logger(), "set robot configuration done");
  } else {
    RCLCPP_INFO(
      this->rosNode->get_logger(), "available modes:no_gravity, feet, pinned, "
      "nominal");
  }
}

//////////////////////////////////////////////////
void VRCPlugin::StepDataToTwist(
  gz::sim::EntityComponentManager & _ecm,
  const atlas_msgs::msg::AtlasBehaviorStepData & _step, double _dt,
  geometry_msgs::msg::Twist & _twist)
{
  const unsigned int footIdx = _step.foot_index;
  const gz::math::Pose3d currentPelvisPose =
    gz::sim::worldPose(this->atlas.pinLinkEntity, _ecm);
  gz::sim::Model atlasModel(this->atlas.modelEntity);
  const gz::sim::Entity footLink = atlasModel.LinkByName(
    _ecm, (footIdx == 0) ? "l_foot" : "r_foot");
  if (footLink == gz::sim::kNullEntity) {
    RCLCPP_ERROR(
      this->rosNode->get_logger(),
      "Couldn't find Atlas's foot link when faking walking.");
    return;
  }
  const gz::math::Pose3d currentFootPose =
    gz::sim::worldPose(footLink, _ecm);
  const gz::math::Pose3d tFootPelvis = currentPelvisPose - currentFootPose;

  const geometry_msgs::msg::Pose & tmpPose = _step.pose;
  gz::math::Pose3d goalFootPose;
  goalFootPose.Pos() =
    gz::math::Vector3d(tmpPose.position.x, tmpPose.position.y, tmpPose.position.z);
  goalFootPose.Rot() = gz::math::Quaterniond(
    tmpPose.orientation.w, tmpPose.orientation.x, tmpPose.orientation.y,
    tmpPose.orientation.z);

  const gz::math::Pose3d goalPelvisPose = goalFootPose + tFootPelvis;

  const double dx = goalPelvisPose.Pos().X() - currentPelvisPose.Pos().X();
  const double dy = goalPelvisPose.Pos().Y() - currentPelvisPose.Pos().Y();
  gz::math::Angle dyawAngle(
    goalPelvisPose.Rot().Euler().Z() - currentPelvisPose.Rot().Euler().Z());
  dyawAngle.Normalize();
  const double dyaw = dyawAngle.Radian();

  gz::math::Vector3d localD(dx, dy, 0);
  localD = currentPelvisPose.Rot().RotateVectorReverse(localD);

  _twist.linear.x = localD.X() / _dt;
  _twist.linear.y = localD.Y() / _dt;
  _twist.linear.z = 0.0;
  _twist.angular.x = 0.0;
  _twist.angular.y = 0.0;
  _twist.angular.z = dyaw / _dt;
}

//////////////////////////////////////////////////
void VRCPlugin::SetFakeASIC(
  const atlas_msgs::msg::AtlasSimInterfaceCommand::SharedPtr _asic)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->pendingASIC = *_asic;
  this->pendingFakeASIC = true;
}

//////////////////////////////////////////////////
void VRCPlugin::DoSetFakeASIC(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
  const atlas_msgs::msg::AtlasSimInterfaceCommand & _asic)
{
  // Disable the real BDI behavior library.
  atlas_msgs::msg::AtlasSimInterfaceCommand ac;
  ac.header.stamp = this->rosNode->get_clock()->now();
  ac.behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::USER;
  ac.k_effort.assign(this->atlasCommandController.jointNames.size(), 255);
  this->atlasCommandController.pubAtlasSimInterfaceCommand->publish(ac);

  const geometry_msgs::msg::Twist zeroVel;
  if (_asic.behavior == atlas_msgs::msg::AtlasSimInterfaceCommand::STAND) {
    // We fake STAND by pinning the robot.
    this->PinAtlas(_ecm, _eventMgr, true);
    this->SetRobotCmdVel(zeroVel, 0.0);
  } else if (_asic.behavior == atlas_msgs::msg::AtlasSimInterfaceCommand::USER) {
    this->UnpinAtlas(_ecm, _eventMgr);
    this->SetRobotCmdVel(zeroVel, 0.0);
  } else if (_asic.behavior == atlas_msgs::msg::AtlasSimInterfaceCommand::FREEZE) {
    // We fake FREEZE by doing PID around current joint positions.
    if (!this->atlasCommandController.jsValid) {
      RCLCPP_WARN(
        this->rosNode->get_logger(),
        "FREEZE commanded, but no valid joint state yet, so I can't set PID "
        "position goals.");
      return;
    }
    for (size_t i = 0;
      i < this->atlasCommandController.js->position.size() &&
      i < this->atlasCommandController.ac.position.size(); ++i)
    {
      this->atlasCommandController.ac.k_effort[i] = 255;
      this->atlasCommandController.ac.position[i] =
        this->atlasCommandController.js->position[i];
    }
    this->atlasCommandController.pubAtlasCommand->publish(
      this->atlasCommandController.ac);
    this->UnpinAtlas(_ecm, _eventMgr);
    this->SetRobotCmdVel(zeroVel, 0.0);
  } else if (_asic.behavior == atlas_msgs::msg::AtlasSimInterfaceCommand::STAND_PREP) {
    // no-op
    this->SetRobotCmdVel(zeroVel, 0.0);
  } else if (_asic.behavior == atlas_msgs::msg::AtlasSimInterfaceCommand::WALK) {
    if (_asic.walk_params.use_demo_walk) {
      RCLCPP_WARN(this->rosNode->get_logger(), "Demo walk requested, but it's unsupported.");
      return;
    }
    double dt = 0.0;
    for (const auto & step : _asic.walk_params.step_queue) {
      dt += step.duration;
    }
    const size_t stepIdx = _asic.walk_params.step_queue.size() - 1;
    geometry_msgs::msg::Twist cmdVel;
    this->StepDataToTwist(_ecm, _asic.walk_params.step_queue[stepIdx], dt, cmdVel);
    this->atlas.currentStepIndex = _asic.walk_params.step_queue[0].step_index;
    this->atlas.lastStepIndex = _asic.walk_params.step_queue[stepIdx].step_index;
    this->SetFeetCollide("none");
    this->SetRobotCmdVel(cmdVel, dt);
  } else if (_asic.behavior == atlas_msgs::msg::AtlasSimInterfaceCommand::STEP) {
    if (_asic.step_params.use_demo_walk) {
      RCLCPP_WARN(this->rosNode->get_logger(), "Demo walk requested, but it's unsupported.");
      return;
    }
    const double dt = _asic.step_params.desired_step.duration;
    geometry_msgs::msg::Twist cmdVel;
    this->StepDataToTwist(_ecm, _asic.step_params.desired_step, dt, cmdVel);
    this->atlas.currentStepIndex = _asic.step_params.desired_step.step_index;
    this->atlas.lastStepIndex = _asic.step_params.desired_step.step_index;
    this->SetFeetCollide("none");
    this->SetRobotCmdVel(cmdVel, dt);
  } else if (_asic.behavior == atlas_msgs::msg::AtlasSimInterfaceCommand::MANIPULATE) {
    // We fake STAND by pinning the robot.
    this->PinAtlas(_ecm, _eventMgr, true);
    this->SetRobotCmdVel(zeroVel, 0.0);
  } else {
    RCLCPP_WARN(
      this->rosNode->get_logger(), "SetFakeASIC: ignoring unknown behavior "
      "type %d", _asic.behavior);
    return;
  }
  this->atlas.currentBehavior = _asic.behavior;
}

//////////////////////////////////////////////////
void VRCPlugin::SetRobotCmdVelTopic(const geometry_msgs::msg::Twist::SharedPtr _cmd)
{
  this->SetRobotCmdVel(*_cmd, this->cmdVelTopicTimeout);
}

//////////////////////////////////////////////////
void VRCPlugin::SetRobotCmdVel(
  const geometry_msgs::msg::Twist & _cmd, double _duration)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  if (_duration > 0.0) {
    this->warpRobotStopTime = this->currentSimTime +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(_duration));
  } else {
    this->warpRobotStopTime = std::chrono::steady_clock::duration{0};
  }

  if (_cmd.linear.x == 0 && _cmd.linear.y == 0 && _cmd.angular.z == 0) {
    this->warpRobotWithCmdVel = false;
  } else {
    this->robotCmdVel = _cmd;
    this->warpRobotWithCmdVel = true;
    this->lastUpdateTime = this->currentSimTime;
  }
}

//////////////////////////////////////////////////
void VRCPlugin::SetRobotPose(const geometry_msgs::msg::Pose::SharedPtr _pose)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->pendingPose = *_pose;
  this->pendingSetRobotPose = true;
}

//////////////////////////////////////////////////
void VRCPlugin::DoSetRobotPose(
  gz::sim::EntityComponentManager & _ecm, const gz::math::Pose3d & _pose)
{
  gz::sim::Model(this->atlas.modelEntity).SetWorldPoseCmd(_ecm, _pose);
}

//////////////////////////////////////////////////
void VRCPlugin::RobotGrabFireHose(const geometry_msgs::msg::Pose::SharedPtr _cmd)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->pendingGrabPose = *_cmd;
  this->pendingGrabFireHose = true;
}

//////////////////////////////////////////////////
void VRCPlugin::DoRobotGrabFireHose(
  gz::sim::EntityComponentManager & _ecm, const gz::math::Pose3d & _cmd)
{
  /// \todo: get these from incoming message
  const std::string gripperName = "r_hand";
  const gz::math::Pose3d relPose(
    gz::math::Vector3d(0, -0.3, -0.1), gz::math::Quaterniond::Identity);

  if (this->drcFireHose.fireHoseModelEntity == gz::sim::kNullEntity ||
    this->drcFireHose.couplingLinkEntity == gz::sim::kNullEntity)
  {
    return;
  }

  gz::sim::Model atlasModel(this->atlas.modelEntity);
  const gz::sim::Entity gripper = atlasModel.LinkByName(_ecm, gripperName);
  if (gripper == gz::sim::kNullEntity) {
    return;
  }

  const gz::math::Pose3d gripperWorldPose = gz::sim::worldPose(gripper, _ecm);
  const gz::math::Pose3d pose = _cmd + relPose + gripperWorldPose;
  this->SetLinkWorldPose(
    _ecm, this->drcFireHose.fireHoseModelEntity,
    this->drcFireHose.couplingLinkEntity, pose);

  if (this->grabJoint == gz::sim::kNullEntity) {
    this->grabJoint = this->AddJoint(
      _ecm, *this->eventMgr, this->atlas.modelEntity, gripper,
      this->drcFireHose.couplingLinkEntity);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::RobotReleaseLink(const geometry_msgs::msg::Pose::SharedPtr)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->pendingReleaseLink = true;
}

//////////////////////////////////////////////////
void VRCPlugin::DoRobotReleaseLink(gz::sim::EntityComponentManager & _ecm)
{
  this->RemoveJoint(_ecm, this->grabJoint);
}

//////////////////////////////////////////////////
void VRCPlugin::SetRobotConfiguration(
  const sensor_msgs::msg::JointState::SharedPtr)
{
  // Planned but never implemented in the original either.
  RCLCPP_ERROR(
    this->rosNode->get_logger(), "The atlas/configuration handler is not "
    "implemented.");
}

//////////////////////////////////////////////////
gz::math::Pose3d VRCPlugin::ToGzPose(const geometry_msgs::msg::Pose & _pose)
{
  gz::math::Quaterniond q(
    _pose.orientation.w, _pose.orientation.x, _pose.orientation.y,
    _pose.orientation.z);
  q.Normalize();
  return gz::math::Pose3d(
    gz::math::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z), q);
}

//////////////////////////////////////////////////
void VRCPlugin::RobotEnterCar(const geometry_msgs::msg::Pose::SharedPtr _pose)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->pendingEnterCarPose = *_pose;
  this->pendingEnterCar = true;
}

//////////////////////////////////////////////////
void VRCPlugin::DoRobotEnterCar(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
  const gz::math::Pose3d & _pose)
{
  if (this->drcVehicle.modelEntity == gz::sim::kNullEntity) {
    RCLCPP_ERROR(
      this->rosNode->get_logger(), "drc_vehicle model not found, cannot "
      "enter car.");
    return;
  }

  this->RemoveJoint(_ecm, this->atlas.pinJointEntity);
  this->RemoveJoint(_ecm, this->vehicleRobotJoint);

  // Hardcoded offset of the robot when it's seated in the vehicle driver
  // seat.
  this->atlas.vehicleRelPose =
    gz::math::Pose3d(gz::math::Vector3d(-0.06, 0.3, 1.28), gz::math::Quaterniond::Identity);

  // Set robot configuration. Unlike the original, this only publishes the
  // AtlasCommand seating pose (see the class-level design note on the
  // dropped instant joint-position-teleport snap) -- it won't visibly
  // apply until AtlasPlugin exists downstream to consume it.
  this->atlasCommandController.SetSeatingConfiguration();
  RCLCPP_INFO(this->rosNode->get_logger(), "set robot configuration done");

  const gz::math::Pose3d vehicleWorldPose =
    gz::sim::worldPose(this->drcVehicle.modelEntity, _ecm);
  this->SetLinkWorldPose(
    _ecm, this->atlas.modelEntity, this->atlas.pinLinkEntity,
    _pose + this->atlas.vehicleRelPose + vehicleWorldPose);

  if (this->vehicleRobotJoint == gz::sim::kNullEntity) {
    this->vehicleRobotJoint = this->AddJoint(
      _ecm, _eventMgr, this->drcVehicle.modelEntity,
      this->drcVehicle.seatLinkEntity, this->atlas.pinLinkEntity);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::RobotExitCar(const geometry_msgs::msg::Pose::SharedPtr _pose)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->pendingExitCarPose = *_pose;
  this->pendingExitCar = true;
}

//////////////////////////////////////////////////
void VRCPlugin::DoRobotExitCar(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
  const gz::math::Pose3d & _pose)
{
  if (this->drcVehicle.modelEntity == gz::sim::kNullEntity) {
    RCLCPP_ERROR(
      this->rosNode->get_logger(), "drc_vehicle model not found, cannot "
      "exit car.");
    return;
  }

  this->RemoveJoint(_ecm, this->atlas.pinJointEntity);
  this->RemoveJoint(_ecm, this->vehicleRobotJoint);

  // Hardcoded offset of the robot when it's standing next to the vehicle.
  this->atlas.vehicleRelPose =
    gz::math::Pose3d(0.52, 1.7, 1.20, 0, 0, 0);

  this->atlasCommandController.SetPIDStand();
  RCLCPP_INFO(this->rosNode->get_logger(), "set configuration done");

  const gz::math::Pose3d vehicleWorldPose =
    gz::sim::worldPose(this->drcVehicle.modelEntity, _ecm);
  this->SetLinkWorldPose(
    _ecm, this->atlas.modelEntity, this->atlas.pinLinkEntity,
    _pose + this->atlas.vehicleRelPose + vehicleWorldPose);

  if (this->vehicleRobotJoint == gz::sim::kNullEntity) {
    this->vehicleRobotJoint = this->AddJoint(
      _ecm, _eventMgr, this->drcVehicle.modelEntity,
      this->drcVehicle.seatLinkEntity, this->atlas.pinLinkEntity);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::ProcessPendingActions(
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr)
{
  bool doPose = false;
  geometry_msgs::msg::Pose pose;
  bool doMode = false;
  std::string mode;
  bool doASIC = false;
  atlas_msgs::msg::AtlasSimInterfaceCommand asic;
  bool doEnterCar = false;
  geometry_msgs::msg::Pose enterCarPose;
  bool doExitCar = false;
  geometry_msgs::msg::Pose exitCarPose;
  bool doGrab = false;
  geometry_msgs::msg::Pose grabPose;
  bool doRelease = false;

  {
    std::lock_guard<std::mutex> lock(this->controlMutex);
    if (this->pendingSetRobotPose) {
      doPose = true;
      pose = this->pendingPose;
      this->pendingSetRobotPose = false;
    }
    if (this->pendingSetRobotMode) {
      doMode = true;
      mode = this->pendingMode;
      this->pendingSetRobotMode = false;
    }
    if (this->pendingFakeASIC) {
      doASIC = true;
      asic = this->pendingASIC;
      this->pendingFakeASIC = false;
    }
    if (this->pendingEnterCar) {
      doEnterCar = true;
      enterCarPose = this->pendingEnterCarPose;
      this->pendingEnterCar = false;
    }
    if (this->pendingExitCar) {
      doExitCar = true;
      exitCarPose = this->pendingExitCarPose;
      this->pendingExitCar = false;
    }
    if (this->pendingGrabFireHose) {
      doGrab = true;
      grabPose = this->pendingGrabPose;
      this->pendingGrabFireHose = false;
    }
    if (this->pendingReleaseLink) {
      doRelease = true;
      this->pendingReleaseLink = false;
    }
  }

  if (doPose) {
    this->DoSetRobotPose(_ecm, ToGzPose(pose));
  }
  if (doMode) {
    this->DoSetRobotMode(_ecm, _eventMgr, mode);
  }
  if (doASIC) {
    this->DoSetFakeASIC(_ecm, _eventMgr, asic);
  }
  if (doEnterCar) {
    this->DoRobotEnterCar(_ecm, _eventMgr, ToGzPose(enterCarPose));
  }
  if (doExitCar) {
    this->DoRobotExitCar(_ecm, _eventMgr, ToGzPose(exitCarPose));
  }
  if (doGrab) {
    this->DoRobotGrabFireHose(_ecm, ToGzPose(grabPose));
  }
  if (doRelease) {
    this->DoRobotReleaseLink(_ecm);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::UpdateStates(
  const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & _eventMgr)
{
  {
    std::lock_guard<std::mutex> lock(this->controlMutex);
    this->currentSimTime = _info.simTime;
  }

  const double curTime = std::chrono::duration<double>(_info.simTime).count();

  // Kinematically hold the pin link in place every tick while pinned --
  // see the class-level design note on why this replaced a physics joint
  // entirely. Runs unconditionally, every tick, ahead of the state
  // machine below, so it applies regardless of which state currently has
  // atlas pinned (startup pinning, "pinned"/"pinned_with_gravity"/
  // "pid_stand"/"harnessed" modes, or the cmd_vel warp-while-pinned path).
  if (this->atlas.pinJointEntity != gz::sim::kNullEntity &&
    this->atlas.pinLinkEntity != gz::sim::kNullEntity)
  {
    this->SetLinkWorldPose(
      _ecm, this->atlas.modelEntity, this->atlas.pinLinkEntity,
      this->atlas.pinHoldPose);
  }

  // If user chooses bdi_stand mode, robot will be initialized with PID
  // stand in BDI stand pose pinned.
  if (this->atlas.startupSequence == Robot::NONE) {
    this->atlas.InsertModel(this->world, _ecm, _eventMgr, this->sdfConfig, this->rosNode);
  } else if (this->atlas.startupSequence == Robot::SPAWN_QUEUED) {
    // Unreachable: SdfEntityCreator's model spawn is synchronous (see the
    // class-level design note), so InsertModel() never leaves this state
    // set. Kept for state-machine fidelity with the original.
    if (this->atlas.CheckGetModel(this->world, _ecm)) {
      this->atlas.startupSequence = Robot::SPAWN_SUCCESS;
    }
  } else if (this->atlas.startupSequence == Robot::SPAWN_SUCCESS) {
    RCLCPP_INFO(
      this->rosNode->get_logger(), "spawn success, set pinLink and call "
      "initialize controller");

    gz::sim::Model atlasModel(this->atlas.modelEntity);
    this->atlas.pinLinkEntity = atlasModel.LinkByName(_ecm, this->atlas.pinLinkName);

    if (this->atlas.pinLinkEntity == gz::sim::kNullEntity) {
      RCLCPP_ERROR(
        this->rosNode->get_logger(), "atlas robot pin link not found, "
        "VRCPlugin will not work.");
      this->atlas.startupSequence = Robot::NONE;
      return;
    }

    this->atlas.initialPose = gz::sim::worldPose(this->atlas.pinLinkEntity, _ecm);
    gz::sim::Link(this->atlas.pinLinkEntity).EnableVelocityChecks(_ecm);

    this->atlasCommandController.InitModel(_ecm, this->atlas.modelEntity, this->rosNode);

    this->atlas.startupSequence = Robot::INIT_MODEL_SUCCESS;
  } else if (this->atlas.startupSequence == Robot::INIT_MODEL_SUCCESS) {
    if (this->atlas.startInVehicle) {
      RCLCPP_INFO(this->rosNode->get_logger(), "Starting robot in vehicle.");
      this->DoRobotEnterCar(_ecm, _eventMgr, gz::math::Pose3d::Zero);
      this->atlas.startupSequence = Robot::INITIALIZED;
    } else if (this->atlas.startupMode == "bdi_stand") {
      switch (this->atlas.bdiStandSequence) {
        case Robot::BS_NONE:
          this->DoSetRobotMode(_ecm, _eventMgr, "pid_stand");
          this->atlas.bdiStandSequence = Robot::BS_PID_PINNED;
          this->atlas.startupBDIStandStartTime = _info.simTime;
          break;
        case Robot::BS_PID_PINNED:
          if ((_info.simTime - this->atlas.startupBDIStandStartTime) >
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(this->atlas.startupStandPrepDuration)))
          {
            RCLCPP_INFO(this->rosNode->get_logger(), "going into stand prep");
            this->atlasCommandController.SetBDIStandPrep();
            this->atlas.bdiStandSequence = Robot::BS_STAND_PREP_PINNED;
          }
          break;
        case Robot::BS_STAND_PREP_PINNED:
          if ((_info.simTime - this->atlas.startupBDIStandStartTime) >
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(this->atlas.startupNominal)))
          {
            RCLCPP_INFO(this->rosNode->get_logger(), "going into Nominal");
            this->DoSetRobotMode(_ecm, _eventMgr, "nominal");
            this->atlas.bdiStandSequence = Robot::BS_STAND_PREP;
          }
          break;
        case Robot::BS_STAND_PREP:
          if ((_info.simTime - this->atlas.startupBDIStandStartTime) >
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(this->atlas.startupStand)))
          {
            RCLCPP_INFO(
              this->rosNode->get_logger(), "going into Dynamic Stand Behavior");
            this->atlasCommandController.SetBDIStand();
            this->atlas.bdiStandSequence = Robot::BS_INITIALIZED;
            this->atlas.startupSequence = Robot::INITIALIZED;
          }
          break;
        default:
          break;
      }
    } else {
      switch (this->atlas.pinnedSequence) {
        case Robot::PS_NONE:
          this->DoSetRobotMode(_ecm, _eventMgr, "pinned");
          if (this->atlas.startupHarnessDuration == 0.0) {
            RCLCPP_INFO(this->rosNode->get_logger(), "Atlas will stay pinned.");
            this->atlas.pinnedSequence = Robot::PS_INITIALIZED;
          } else {
            RCLCPP_INFO(
              this->rosNode->get_logger(), "Resume to nominal mode after %f "
              "seconds.", this->atlas.startupHarnessDuration);
            this->atlas.pinnedSequence = Robot::PS_PINNED;
          }
          break;
        case Robot::PS_PINNED:
          if (this->atlas.startupHarnessDuration != 0.0 &&
            curTime > this->atlas.startupHarnessDuration)
          {
            this->DoSetRobotMode(_ecm, _eventMgr, "nominal");
            this->atlas.pinnedSequence = Robot::PS_INITIALIZED;
            this->atlas.startupSequence = Robot::INITIALIZED;
          }
          break;
        default:
          break;
      }
    }
  }

  this->ProcessPendingActions(_ecm, _eventMgr);
  this->ApplyGravityCompensation(_ecm);

  if (curTime > std::chrono::duration<double>(this->lastUpdateTime).count()) {
    this->CheckThreadStart(_ecm);

    const double dt = curTime - std::chrono::duration<double>(this->lastUpdateTime).count();

    if (this->warpRobotWithCmdVel && (_info.simTime <= this->warpRobotStopTime)) {
      this->lastUpdateTime = _info.simTime;
      const gz::math::Pose3d curPose =
        gz::sim::worldPose(this->atlas.pinLinkEntity, _ecm);
      gz::math::Pose3d newPose = curPose;

      gz::math::Vector3d cmd(
        this->robotCmdVel.linear.x, this->robotCmdVel.linear.y, 0);
      cmd = curPose.Rot().RotateVector(cmd);

      newPose.Pos() = curPose.Pos() + cmd * dt;
      // Prevent robot from drifting vertically.
      newPose.Pos().Z() = this->atlas.initialPose.Pos().Z();

      gz::math::Vector3d rpy = curPose.Rot().Euler();
      rpy.X() = 0;
      rpy.Y() = 0;
      rpy.Z() = rpy.Z() + this->robotCmdVel.angular.z * dt;
      newPose.Rot() = gz::math::Quaterniond(rpy);

      this->Teleport(
        _ecm, _eventMgr, this->atlas.pinLinkEntity, this->atlas.pinJointEntity,
        newPose);
    }
  }

  if (this->atlas.startupSequence == Robot::INITIALIZED && this->cheatsEnabled) {
    atlas_msgs::msg::AtlasSimInterfaceState asis;
    asis.error_code = atlas_msgs::msg::AtlasSimInterfaceState::NO_ERRORS;
    asis.current_behavior = this->atlas.currentBehavior;
    asis.desired_behavior = this->atlas.currentBehavior;
    for (auto & f : asis.f_out) {
      f = 0.0;
    }
    const gz::math::Pose3d curPose =
      gz::sim::worldPose(this->atlas.pinLinkEntity, _ecm);
    asis.pos_est.position.x = curPose.Pos().X();
    asis.pos_est.position.y = curPose.Pos().Y();
    asis.pos_est.position.z = curPose.Pos().Z();
    gz::sim::Link pinLink(this->atlas.pinLinkEntity);
    const auto curVel = pinLink.WorldLinearVelocity(_ecm);
    if (curVel) {
      asis.pos_est.velocity.x = curVel->X();
      asis.pos_est.velocity.y = curVel->Y();
      asis.pos_est.velocity.z = curVel->Z();
    }

    gz::sim::Model atlasModel(this->atlas.modelEntity);
    const gz::sim::Entity lFootLink = atlasModel.LinkByName(_ecm, "l_foot");
    if (lFootLink == gz::sim::kNullEntity) {
      RCLCPP_WARN(
        this->rosNode->get_logger(), "Couldn't find l_foot link when "
        "publishing fake behavior data.");
    } else {
      const gz::math::Pose3d lFootPose = gz::sim::worldPose(lFootLink, _ecm);
      asis.foot_pos_est[0].position.x = lFootPose.Pos().X();
      asis.foot_pos_est[0].position.y = lFootPose.Pos().Y();
      asis.foot_pos_est[0].position.z = lFootPose.Pos().Z();
      asis.foot_pos_est[0].orientation.w = lFootPose.Rot().W();
      asis.foot_pos_est[0].orientation.x = lFootPose.Rot().X();
      asis.foot_pos_est[0].orientation.y = lFootPose.Rot().Y();
      asis.foot_pos_est[0].orientation.z = lFootPose.Rot().Z();
    }
    const gz::sim::Entity rFootLink = atlasModel.LinkByName(_ecm, "r_foot");
    if (rFootLink == gz::sim::kNullEntity) {
      RCLCPP_WARN(
        this->rosNode->get_logger(), "Couldn't find r_foot link when "
        "publishing fake behavior data.");
    } else {
      const gz::math::Pose3d rFootPose = gz::sim::worldPose(rFootLink, _ecm);
      asis.foot_pos_est[1].position.x = rFootPose.Pos().X();
      asis.foot_pos_est[1].position.y = rFootPose.Pos().Y();
      asis.foot_pos_est[1].position.z = rFootPose.Pos().Z();
      asis.foot_pos_est[1].orientation.w = rFootPose.Rot().W();
      asis.foot_pos_est[1].orientation.x = rFootPose.Rot().X();
      asis.foot_pos_est[1].orientation.y = rFootPose.Rot().Y();
      asis.foot_pos_est[1].orientation.z = rFootPose.Rot().Z();
    }
    for (auto & k : asis.k_effort) {
      k = 0;
    }

    if (asis.current_behavior == atlas_msgs::msg::AtlasSimInterfaceCommand::WALK) {
      const double timeRemaining = std::chrono::duration<double>(
        this->warpRobotStopTime - _info.simTime).count();
      if (timeRemaining > 0.0) {
        asis.walk_feedback.t_step_rem = timeRemaining * 1e3;
        asis.walk_feedback.current_step_index = this->atlas.currentStepIndex;
      } else {
        asis.walk_feedback.t_step_rem = 0.0;
        asis.walk_feedback.current_step_index = this->atlas.lastStepIndex;
      }
      asis.walk_feedback.next_step_index_needed = this->atlas.lastStepIndex + 1;
    }

    this->atlas.pubFakeASIS->publish(asis);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::LoadVRCROSAPI()
{
  if (!this->cheatsEnabled) {
    return;
  }

  this->subRobotEnterCar = this->rosNode->create_subscription<geometry_msgs::msg::Pose>(
    "drc_world/robot_enter_car", 100,
    std::bind(&VRCPlugin::RobotEnterCar, this, std::placeholders::_1));

  this->subRobotExitCar = this->rosNode->create_subscription<geometry_msgs::msg::Pose>(
    "drc_world/robot_exit_car", 100,
    std::bind(&VRCPlugin::RobotExitCar, this, std::placeholders::_1));

  this->subRobotGrab = this->rosNode->create_subscription<geometry_msgs::msg::Pose>(
    "drc_world/robot_grab_link", 100,
    std::bind(&VRCPlugin::RobotGrabFireHose, this, std::placeholders::_1));

  this->subRobotRelease = this->rosNode->create_subscription<geometry_msgs::msg::Pose>(
    "drc_world/robot_release_link", 100,
    std::bind(&VRCPlugin::RobotReleaseLink, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void VRCPlugin::LoadRobotROSAPI()
{
  this->atlas.startupHarnessDuration =
    this->rosNode->declare_parameter("atlas.time_to_unpin", 5.0);
  this->atlas.startupMode =
    this->rosNode->declare_parameter("atlas.startup_mode", std::string(""));
  this->atlas.startInVehicle =
    this->rosNode->declare_parameter("robot_start_in_vehicle", false);
  if (this->atlas.startupMode == "bdi_stand") {
    RCLCPP_INFO(this->rosNode->get_logger(), "Starting robot with BDI standing");
  } else if (this->atlas.startupMode == "pinned") {
    RCLCPP_INFO(this->rosNode->get_logger(), "Starting robot pinned");
  } else {
    RCLCPP_ERROR(
      this->rosNode->get_logger(), "Unsupported atlas.startup_mode [%s]",
      this->atlas.startupMode.c_str());
  }

  if (!this->cheatsEnabled) {
    return;
  }

  this->atlas.subTrajectory =
    this->rosNode->create_subscription<geometry_msgs::msg::Twist>(
    "atlas/cmd_vel", 100,
    std::bind(&VRCPlugin::SetRobotCmdVelTopic, this, std::placeholders::_1));

  this->atlas.subPose = this->rosNode->create_subscription<geometry_msgs::msg::Pose>(
    "atlas/set_pose", 100,
    std::bind(&VRCPlugin::SetRobotPose, this, std::placeholders::_1));

  this->atlas.subConfiguration =
    this->rosNode->create_subscription<sensor_msgs::msg::JointState>(
    "atlas/configuration", 100,
    std::bind(&VRCPlugin::SetRobotConfiguration, this, std::placeholders::_1));

  this->atlas.subMode = this->rosNode->create_subscription<std_msgs::msg::String>(
    "atlas/mode", 100,
    std::bind(&VRCPlugin::SetRobotModeTopic, this, std::placeholders::_1));

  this->atlas.subFakeASIC =
    this->rosNode->create_subscription<atlas_msgs::msg::AtlasSimInterfaceCommand>(
    "atlas/fake/atlas_sim_interface_command", 100,
    std::bind(&VRCPlugin::SetFakeASIC, this, std::placeholders::_1));

  this->atlas.pubFakeASIS =
    this->rosNode->create_publisher<atlas_msgs::msg::AtlasSimInterfaceState>(
    "atlas/fake/atlas_sim_interface_state", rclcpp::QoS(1).transient_local());
}

//////////////////////////////////////////////////
void VRCPlugin::CheckThreadStart(gz::sim::EntityComponentManager & _ecm)
{
  if (!this->drcFireHose.isInitialized ||
    this->drcFireHose.screwJointEntity != gz::sim::kNullEntity)
  {
    return;
  }

  const gz::math::Pose3d connectPose = this->drcFireHose.couplingRelativePose;

  std::vector<gz::sim::Entity> collisions = _ecm.ChildrenByComponents(
    this->drcFireHose.couplingLinkEntity, gz::sim::components::Collision());
  gz::sim::Entity attachmentCol = gz::sim::kNullEntity;
  for (const gz::sim::Entity col : collisions) {
    const auto * name = _ecm.Component<gz::sim::components::Name>(col);
    if (name && name->Data() == "attachment_col") {
      attachmentCol = col;
      break;
    }
  }
  if (attachmentCol == gz::sim::kNullEntity) {
    return;
  }

  const auto * collisionElement =
    _ecm.Component<gz::sim::components::CollisionElement>(attachmentCol);
  if (!collisionElement || !collisionElement->Data().Geom() ||
    !collisionElement->Data().Geom()->CylinderShape())
  {
    return;
  }
  const double cylinderLength =
    collisionElement->Data().Geom()->CylinderShape()->Length();
  const gz::math::Pose3d colRelativePose = collisionElement->Data().RawPose();
  const double collisionSurfaceZOffset =
    colRelativePose.Pos().X() - cylinderLength / 2.0;

  const gz::math::Pose3d relativePose =
    (gz::math::Pose3d(collisionSurfaceZOffset, 0, 0, 0, 0, 0) +
    gz::sim::worldPose(this->drcFireHose.couplingLinkEntity, _ecm)) -
    gz::sim::worldPose(this->drcFireHose.spoutLinkEntity, _ecm);

  const double posErrInsert =
    relativePose.Pos().Z() - connectPose.Pos().Z() + collisionSurfaceZOffset;
  const double posErrCenter =
    std::fabs(relativePose.Pos().X() - connectPose.Pos().X()) +
    std::fabs(relativePose.Pos().Y() - connectPose.Pos().Y());
  const double rotErr =
    (relativePose.Rot().XAxis() - connectPose.Rot().XAxis()).Length();
  double valveAng = 0.0;
  if (this->drcFireHose.valveJointEntity != gz::sim::kNullEntity) {
    valveAng = FirstOrZero(
      gz::sim::Joint(this->drcFireHose.valveJointEntity).Position(_ecm));
  }

  // Check that the hose coupler is positioned within tolerance and that the
  // valve is not opened, because the water rushing out would prevent you
  // from attaching a hose. This check also prevents out-of-order execution
  // that would confuse scoring in VRCScoringPlugin.
  if (posErrInsert > 0.0 && posErrCenter < 0.003 && rotErr < 0.05 &&
    valveAng > -0.1)
  {
    // A real screw joint has no cross-model path here (see the class-level
    // design note) -- docking is a fixed weld instead. This also means
    // there's no live joint angle to auto-detect "unscrewing" with, so
    // (unlike the original) this connection persists once made.
    this->drcFireHose.screwJointEntity = this->AddJoint(
      _ecm, *this->eventMgr, this->drcFireHose.standpipeModelEntity,
      this->drcFireHose.spoutLinkEntity, this->drcFireHose.couplingLinkEntity);
  }
}

//////////////////////////////////////////////////
void VRCPlugin::Vehicle::Load(
  gz::sim::World & _world, gz::sim::EntityComponentManager & _ecm,
  const sdf::ElementPtr & _pluginSdf)
{
  this->isInitialized = false;

  std::string modelName = "drc_vehicle";
  std::string seatLinkName = "chassis";
  if (_pluginSdf->HasElement("drc_vehicle")) {
    const auto vehicleSdf = _pluginSdf->GetElement("drc_vehicle");
    if (vehicleSdf->HasElement("model_name")) {
      modelName = vehicleSdf->Get<std::string>("model_name");
    }
    if (vehicleSdf->HasElement("seat_link")) {
      seatLinkName = vehicleSdf->Get<std::string>("seat_link");
    }
  }

  this->modelEntity = _world.ModelByName(_ecm, modelName);
  if (this->modelEntity == gz::sim::kNullEntity) {
    gzmsg << "drc vehicle not found." << std::endl;
    return;
  }

  gz::sim::Model vehicleModel(this->modelEntity);
  this->seatLinkEntity = vehicleModel.LinkByName(_ecm, seatLinkName);
  if (this->seatLinkEntity == gz::sim::kNullEntity) {
    gzerr << "drc vehicle seat link not found." << std::endl;
    return;
  }

  this->initialPose = gz::sim::worldPose(this->seatLinkEntity, _ecm);
  this->isInitialized = true;
}

//////////////////////////////////////////////////
void VRCPlugin::FireHose::Load(
  gz::sim::World & _world, gz::sim::EntityComponentManager & _ecm,
  const sdf::ElementPtr & _pluginSdf)
{
  this->isInitialized = false;

  if (!_pluginSdf->HasElement("drc_fire_hose")) {
    gzmsg << "VRCPlugin: no <drc_fire_hose> block, threading disabled."
          << std::endl;
    return;
  }
  const auto fireHoseSdf = _pluginSdf->GetElement("drc_fire_hose");

  const std::string fireHoseModelName =
    fireHoseSdf->Get<std::string>("fire_hose_model");
  this->fireHoseModelEntity = _world.ModelByName(_ecm, fireHoseModelName);
  if (this->fireHoseModelEntity == gz::sim::kNullEntity) {
    gzmsg << "VRCPlugin: fire_hose_model [" << fireHoseModelName
          << "] not found, threading disabled." << std::endl;
    return;
  }
  this->initialFireHosePose =
    gz::sim::worldPose(this->fireHoseModelEntity, _ecm);

  gz::sim::Model fireHoseModel(this->fireHoseModelEntity);
  const std::string couplingLinkName =
    fireHoseSdf->Get<std::string>("coupling_link");
  this->couplingLinkEntity = fireHoseModel.LinkByName(_ecm, couplingLinkName);
  if (this->couplingLinkEntity == gz::sim::kNullEntity) {
    gzmsg << "VRCPlugin: coupling link [" << couplingLinkName
          << "] not found, threading disabled." << std::endl;
    return;
  }

  const std::string standpipeModelName =
    fireHoseSdf->Get<std::string>("standpipe_model");
  this->standpipeModelEntity = _world.ModelByName(_ecm, standpipeModelName);
  if (this->standpipeModelEntity == gz::sim::kNullEntity) {
    gzerr << "VRCPlugin: standpipe model [" << standpipeModelName
          << "] not found" << std::endl;
    return;
  }

  gz::sim::Model standpipeModel(this->standpipeModelEntity);
  const std::string spoutLinkName = fireHoseSdf->Get<std::string>("spout_link");
  this->spoutLinkEntity = standpipeModel.LinkByName(_ecm, spoutLinkName);
  if (this->spoutLinkEntity == gz::sim::kNullEntity) {
    gzerr << "VRCPlugin: spout link [" << spoutLinkName << "] not found"
          << std::endl;
    return;
  }

  const std::string valveModelName = fireHoseSdf->HasElement("valve_model") ?
    fireHoseSdf->Get<std::string>("valve_model") : "valve";
  this->valveModelEntity = _world.ModelByName(_ecm, valveModelName);
  if (this->valveModelEntity == gz::sim::kNullEntity) {
    gzwarn << "VRCPlugin: valve model [" << valveModelName
           << "] not found, scoring will be wrong" << std::endl;
  } else {
    const std::string valveJointName = fireHoseSdf->HasElement("valve_joint") ?
      fireHoseSdf->Get<std::string>("valve_joint") : "valve";
    gz::sim::Model valveModel(this->valveModelEntity);
    this->valveJointEntity = valveModel.JointByName(_ecm, valveJointName);
    if (this->valveJointEntity == gz::sim::kNullEntity) {
      gzwarn << "VRCPlugin: valve joint [" << valveJointName
             << "] not found, scoring will be wrong" << std::endl;
    } else {
      gz::sim::Joint(this->valveJointEntity).EnablePositionCheck(_ecm);
    }
  }

  this->threadPitch = fireHoseSdf->Get<double>("thread_pitch");
  this->couplingRelativePose =
    fireHoseSdf->Get<gz::math::Pose3d>("coupling_relative_pose");

  // The original's SetInitialConfiguration() zeroed each fire-hose joint's
  // position via an instant teleport-style setter that has no gz-sim
  // equivalent (see the class-level design note); dropped, joints keep
  // whatever pose the world SDF gave them.

  this->isInitialized = true;
}

//////////////////////////////////////////////////
VRCPlugin::Robot::Robot()
: bdiStandSequence(Robot::BS_NONE),
  pinnedSequence(Robot::PS_NONE),
  startupSequence(Robot::NONE),
  startupHarnessDuration(5.0),
  currentBehavior(-1),
  currentStepIndex(0),
  lastStepIndex(0)
{
  // Bunch of hardcoded presets.
  this->startupStandPrepDuration = 2.0;
  this->startupNominal = this->startupStandPrepDuration + 2.0;
  this->startupStand = this->startupNominal + 0.1;
}

//////////////////////////////////////////////////
void VRCPlugin::Robot::InsertModel(
  gz::sim::World & _world, gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & _eventMgr,
  const sdf::ElementPtr & _pluginSdf,
  const rclcpp::Node::SharedPtr & _rosNode)
{
  this->spawnPose = gz::math::Pose3d::Zero;
  this->modelName = "atlas";
  this->pinLinkName = "utorso";

  if (_pluginSdf->HasElement("atlas")) {
    const auto atlasSdf = _pluginSdf->GetElement("atlas");
    if (atlasSdf->HasElement("model_name")) {
      this->modelName = atlasSdf->Get<std::string>("model_name");
    }
    if (atlasSdf->HasElement("pin_link")) {
      this->pinLinkName = atlasSdf->Get<std::string>("pin_link");
    }
  }

  this->modelEntity = _world.ModelByName(_ecm, this->modelName);

  if (this->modelEntity != gz::sim::kNullEntity) {
    RCLCPP_INFO(
      _rosNode->get_logger(), "atlas model found (included in world file).");
    this->startupSequence = Robot::SPAWN_SUCCESS;
    return;
  }

  RCLCPP_INFO(
    _rosNode->get_logger(), "atlas model not in world file, spawning from "
    "the robot_description parameter.");

  const double x = _rosNode->declare_parameter("robot_initial_pose.x", 0.0);
  const double y = _rosNode->declare_parameter("robot_initial_pose.y", 0.0);
  const double z = _rosNode->declare_parameter("robot_initial_pose.z", 0.0);
  const double roll = _rosNode->declare_parameter("robot_initial_pose.roll", 0.0);
  const double pitch = _rosNode->declare_parameter("robot_initial_pose.pitch", 0.0);
  const double yaw = _rosNode->declare_parameter("robot_initial_pose.yaw", 0.0);
  this->spawnPose = gz::math::Pose3d(x, y, z, roll, pitch, yaw);

  const std::string robotStr =
    _rosNode->declare_parameter("robot_description", std::string(""));
  if (robotStr.empty()) {
    RCLCPP_ERROR(
      _rosNode->get_logger(), "failed to spawn model: robot_description "
      "parameter not set.");
    this->startupSequence = Robot::NONE;
    return;
  }

  sdf::Root root;
  const sdf::Errors errors = root.LoadSdfString(robotStr);
  if (!errors.empty() || !root.Model()) {
    RCLCPP_ERROR(
      _rosNode->get_logger(), "failed to parse robot_description as an "
      "SDF or URDF model.");
    this->startupSequence = Robot::NONE;
    return;
  }
  gz::sim::SdfEntityCreator creator(_ecm, _eventMgr);
  this->modelEntity = creator.CreateEntities(root.Model());
  creator.SetParent(this->modelEntity, _world.Entity());
  // sdf::Root::Model() only returns a const pointer, so the spawn pose
  // can't be set on the DOM before creation -- command it on the live
  // entity instead, the same mechanism used for every other teleport in
  // this plugin.
  gz::sim::Model(this->modelEntity).SetWorldPoseCmd(_ecm, this->spawnPose);

  // SdfEntityCreator::CreateEntities() is synchronous (unlike Classic's
  // async InsertModelString()+poll), so the model is already live -- no
  // SPAWN_QUEUED wait is needed (see the class-level design note).
  this->startupSequence = Robot::SPAWN_SUCCESS;
}

//////////////////////////////////////////////////
bool VRCPlugin::Robot::CheckGetModel(
  gz::sim::World & _world, gz::sim::EntityComponentManager & _ecm)
{
  this->modelEntity = _world.ModelByName(_ecm, this->modelName);
  return this->modelEntity != gz::sim::kNullEntity;
}

//////////////////////////////////////////////////
VRCPlugin::AtlasCommandController::AtlasCommandController()
: atlasVersion(5),
  atlasSubVersion(0)
{
}

//////////////////////////////////////////////////
std::string VRCPlugin::AtlasCommandController::FindJoint(
  gz::sim::EntityComponentManager & _ecm, const std::string & _st1,
  const std::string & _st2)
{
  gz::sim::Model model(this->modelEntity);
  if (model.JointByName(_ecm, _st1) != gz::sim::kNullEntity) {
    return _st1;
  }
  if (model.JointByName(_ecm, _st2) != gz::sim::kNullEntity) {
    return _st2;
  }
  if (const auto node = this->rosNode.lock()) {
    RCLCPP_INFO(
      node->get_logger(), "VRCPlugin: joint by names [%s] or [%s] not found.",
      _st1.c_str(), _st2.c_str());
  }
  return std::string();
}

//////////////////////////////////////////////////
std::string VRCPlugin::AtlasCommandController::FindJoint(
  gz::sim::EntityComponentManager & _ecm, const std::string & _st1,
  const std::string & _st2, const std::string & _st3)
{
  return this->FindJoint(_ecm, this->FindJoint(_ecm, _st1, _st2), _st3);
}

//////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::GetJointStates(
  const sensor_msgs::msg::JointState::SharedPtr _js)
{
  this->js = _js;
  this->jsValid = true;
}

//////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::InitModel(
  gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _modelEntity,
  const rclcpp::Node::SharedPtr & _rosNode)
{
  this->modelEntity = _modelEntity;
  this->rosNode = _rosNode;

  this->atlasVersion = _rosNode->declare_parameter("atlas_version", 5);
  this->atlasSubVersion = _rosNode->declare_parameter("atlas_sub_version", 0);

  // Must match those inside AtlasPlugin.
  this->jointNames.push_back(this->FindJoint(_ecm, "back_bkz", "back_lbz"));
  this->jointNames.push_back(this->FindJoint(_ecm, "back_bky", "back_mby"));
  this->jointNames.push_back(this->FindJoint(_ecm, "back_bkx", "back_ubx"));
  this->jointNames.push_back(this->FindJoint(_ecm, "neck_ry", "neck_ay"));
  this->jointNames.push_back(this->FindJoint(_ecm, "l_leg_hpz", "l_leg_uhz"));
  this->jointNames.push_back(this->FindJoint(_ecm, "l_leg_hpx", "l_leg_mhx"));
  this->jointNames.push_back(this->FindJoint(_ecm, "l_leg_hpy", "l_leg_lhy"));
  this->jointNames.push_back("l_leg_kny");
  this->jointNames.push_back(this->FindJoint(_ecm, "l_leg_aky", "l_leg_uay"));
  this->jointNames.push_back(this->FindJoint(_ecm, "l_leg_akx", "l_leg_lax"));
  this->jointNames.push_back(this->FindJoint(_ecm, "r_leg_hpz", "r_leg_uhz"));
  this->jointNames.push_back(this->FindJoint(_ecm, "r_leg_hpx", "r_leg_mhx"));
  this->jointNames.push_back(this->FindJoint(_ecm, "r_leg_hpy", "r_leg_lhy"));
  this->jointNames.push_back("r_leg_kny");
  this->jointNames.push_back(this->FindJoint(_ecm, "r_leg_aky", "r_leg_uay"));
  this->jointNames.push_back(this->FindJoint(_ecm, "r_leg_akx", "r_leg_lax"));
  this->jointNames.push_back(
    this->FindJoint(_ecm, "l_arm_shz", "l_arm_shy", "l_arm_usy"));
  this->jointNames.push_back("l_arm_shx");
  this->jointNames.push_back("l_arm_ely");
  this->jointNames.push_back("l_arm_elx");
  this->jointNames.push_back(this->FindJoint(_ecm, "l_arm_wry", "l_arm_uwy"));
  this->jointNames.push_back(this->FindJoint(_ecm, "l_arm_wrx", "l_arm_mwx"));

  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
    this->atlasVersion > 4)
  {
    this->jointNames.push_back(this->FindJoint(_ecm, "l_arm_wry2", "l_arm_lwy"));
  }

  this->jointNames.push_back(
    this->FindJoint(_ecm, "r_arm_shz", "r_arm_shy", "r_arm_usy"));
  this->jointNames.push_back("r_arm_shx");
  this->jointNames.push_back("r_arm_ely");
  this->jointNames.push_back("r_arm_elx");
  this->jointNames.push_back(this->FindJoint(_ecm, "r_arm_wry", "r_arm_uwy"));
  this->jointNames.push_back(this->FindJoint(_ecm, "r_arm_wrx", "r_arm_mwx"));

  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
    this->atlasVersion > 4)
  {
    this->jointNames.push_back(this->FindJoint(_ecm, "r_arm_wry2", "r_arm_lwy"));
  }

  const unsigned int n = this->jointNames.size();
  this->ac.position.resize(n);
  this->ac.velocity.resize(n);
  this->ac.effort.resize(n);
  this->ac.kp_position.resize(n);
  this->ac.ki_position.resize(n);
  this->ac.kd_position.resize(n);
  this->ac.kp_velocity.resize(n);
  this->ac.i_effort_min.resize(n);
  this->ac.i_effort_max.resize(n);
  this->ac.k_effort.resize(n);

  for (unsigned int i = 0; i < n; ++i) {
    // FindJoint() returns an empty name for any DOF this robot version
    // doesn't have (e.g. no wry2 joints on Atlas 4.1) -- ROS 2's
    // declare_parameter(), unlike ROS 1's getParam(), throws if the same
    // parameter name is declared twice, and every missing joint would
    // otherwise collide on the same "atlas_controller.gains..*" name.
    // Nothing to configure for a joint that doesn't exist anyway, so just
    // leave its gains at their already-zeroed default.
    if (this->jointNames[i].empty()) {
      continue;
    }
    const std::string prefix = "atlas_controller.gains." + this->jointNames[i] + ".";
    this->ac.kp_position[i] = _rosNode->declare_parameter(prefix + "p", 0.0);
    this->ac.ki_position[i] = _rosNode->declare_parameter(prefix + "i", 0.0);
    this->ac.kd_position[i] = _rosNode->declare_parameter(prefix + "d", 0.0);
    const double iClamp = _rosNode->declare_parameter(prefix + "i_clamp", 0.0);
    this->ac.i_effort_min[i] = -iClamp;
    this->ac.i_effort_max[i] = iClamp;
    this->ac.k_effort[i] = 255;

    this->ac.velocity[i] = 0;
    this->ac.effort[i] = 0;
    this->ac.kp_velocity[i] = 0;
  }

  this->pubAtlasCommand =
    _rosNode->create_publisher<atlas_msgs::msg::AtlasCommand>(
    "atlas/atlas_command", rclcpp::QoS(1).transient_local());

  this->pubAtlasSimInterfaceCommand =
    _rosNode->create_publisher<atlas_msgs::msg::AtlasSimInterfaceCommand>(
    "atlas/atlas_sim_interface_command", rclcpp::QoS(1).transient_local());

  this->subJointStates =
    _rosNode->create_subscription<sensor_msgs::msg::JointState>(
    "atlas/joint_states", 1,
    std::bind(&AtlasCommandController::GetJointStates, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetPIDStand()
{
  const auto node = this->rosNode.lock();
  if (!node) {
    return;
  }
  this->ac.header.stamp = node->get_clock()->now();

  int index = 0;

  if (this->atlasVersion < 4) {
    this->ac.position[0] = -1.8823047867044806e-05;
    this->ac.position[1] = 0.0016903011128306389;
    this->ac.position[2] = 9.384587610838935e-05;
    this->ac.position[3] = -0.6108658313751221;
    this->ac.position[4] = 0.30274710059165955;
    this->ac.position[5] = 0.05022283270955086;
    this->ac.position[6] = -0.25109854340553284;
    this->ac.position[7] = 0.5067367553710938;
    this->ac.position[8] = -0.2464604675769806;
    this->ac.position[9] = -0.05848940089344978;
    this->ac.position[10] = -0.30258211493492126;
    this->ac.position[11] = -0.07534884661436081;
    this->ac.position[12] = -0.2539609372615814;
    this->ac.position[13] = 0.5230700969696045;
    this->ac.position[14] = -0.2662496864795685;
    this->ac.position[15] = 0.0634056106209755;
    this->ac.position[16] = 0.29979637265205383;
    this->ac.position[17] = -1.303655982017517;
    this->ac.position[18] = 2.000823736190796;
    this->ac.position[19] = 0.4982665777206421;
    this->ac.position[20] = 0.00030532144592143595;
    this->ac.position[21] = -0.004383780527859926;
    this->ac.position[22] = 0.2997862696647644;
    this->ac.position[23] = 1.303290843963623;
    this->ac.position[24] = 2.0007426738739014;
    this->ac.position[25] = -0.4982258975505829;
    this->ac.position[26] = 0.0002723461075220257;
    this->ac.position[27] = 0.004452839493751526;
  } else {
    this->ac.position[index++] = 0.0;
    this->ac.position[index++] = 0.00225254;
    this->ac.position[index++] = 0.0;
    this->ac.position[index++] = -0.1106;

    this->ac.position[index++] = -0.00692196;
    this->ac.position[index++] = 0.0690;
    this->ac.position[index++] = -0.472917;
    this->ac.position[index++] = 0.93299556;
    this->ac.position[index++] = -0.4400587703;
    this->ac.position[index++] = -0.0689798;

    this->ac.position[index++] = -this->ac.position[4];
    this->ac.position[index++] = -this->ac.position[5];
    this->ac.position[index++] = this->ac.position[6];
    this->ac.position[index++] = this->ac.position[7];
    this->ac.position[index++] = this->ac.position[8];
    this->ac.position[index++] = -this->ac.position[9];

    this->ac.position[index++] = -0.299681926;
    this->ac.position[index++] = -1.300665;
    this->ac.position[index++] = 1.852762;
    this->ac.position[index++] = 0.492914;
    this->ac.position[index++] = 0.00165999;
    this->ac.position[index++] = -0.00095767089;

    if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
      this->atlasVersion > 4)
    {
      this->ac.position[index++] = 0.01305307;
    }

    this->ac.position[index++] = (this->atlasVersion >= 4) ?
      -this->ac.position[16] : this->ac.position[16];
    this->ac.position[index++] = -this->ac.position[17];
    this->ac.position[index++] = this->ac.position[18];
    this->ac.position[index++] = -this->ac.position[19];
    this->ac.position[index++] = this->ac.position[20];
    this->ac.position[index++] = -this->ac.position[21];

    if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
      this->atlasVersion > 4)
    {
      this->ac.position[index++] = this->ac.position[22];
    }

    this->ac.effort[1] = -27.6;
    this->ac.effort[6] = -23.5;
    this->ac.effort[7] = -105.7;
    this->ac.effort[8] = 24.1;
    this->ac.effort[6 + 6] = -23.5;
    this->ac.effort[7 + 6] = -105.7;
    this->ac.effort[8 + 6] = 24.1;
  }

  for (unsigned int i = 0; i < this->jointNames.size(); ++i) {
    this->ac.k_effort[i] = 255;
  }

  // The original also called Model::SetJointPositions() here for an
  // instant visual snap to this pose; gz-sim has no equivalent (see the
  // class-level design note) -- only the AtlasCommand publish survives, so
  // this pose only takes effect once AtlasPlugin exists downstream to PID
  // toward it.
  this->pubAtlasCommand->publish(this->ac);
}

//////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetBDIFREEZE()
{
  const auto node = this->rosNode.lock();
  if (!node) {
    return;
  }
  atlas_msgs::msg::AtlasSimInterfaceCommand cmd;
  cmd.header.stamp = node->get_clock()->now();
  cmd.behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::FREEZE;
  this->pubAtlasSimInterfaceCommand->publish(cmd);
}

//////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetBDIStandPrep()
{
  const auto node = this->rosNode.lock();
  if (!node) {
    return;
  }
  atlas_msgs::msg::AtlasSimInterfaceCommand cmd;
  cmd.header.stamp = node->get_clock()->now();
  cmd.behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::STAND_PREP;
  cmd.k_effort.assign(this->jointNames.size(), 0);
  this->pubAtlasSimInterfaceCommand->publish(cmd);
}

//////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetBDIStand()
{
  const auto node = this->rosNode.lock();
  if (!node) {
    return;
  }
  atlas_msgs::msg::AtlasSimInterfaceCommand cmd;
  cmd.k_effort.assign(this->jointNames.size(), 0);
  cmd.header.stamp = node->get_clock()->now();
  cmd.behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::STAND;
  this->pubAtlasSimInterfaceCommand->publish(cmd);
}

//////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetSeatingConfiguration()
{
  const auto node = this->rosNode.lock();
  if (!node) {
    return;
  }
  int index = 0;
  this->ac.header.stamp = node->get_clock()->now();
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.45;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = -1.60;
  this->ac.position[index++] = 1.60;
  this->ac.position[index++] = -0.10;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = -0.45;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = -1.60;
  this->ac.position[index++] = 1.60;
  this->ac.position[index++] = -0.10;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 1.50;
  this->ac.position[index++] = 1.50;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;

  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
    this->atlasVersion > 4)
  {
    this->ac.position[index++] = 0.0;
  }

  this->ac.position[index++] = -this->ac.position[16];
  this->ac.position[index++] = -this->ac.position[17];
  this->ac.position[index++] = this->ac.position[18];
  this->ac.position[index++] = -this->ac.position[19];
  this->ac.position[index++] = this->ac.position[20];
  this->ac.position[index++] = -this->ac.position[21];

  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
    this->atlasVersion > 4)
  {
    this->ac.position[index++] = this->ac.position[22];
  }

  this->pubAtlasCommand->publish(this->ac);
}

//////////////////////////////////////////////////
void VRCPlugin::AtlasCommandController::SetStandingConfiguration()
{
  const auto node = this->rosNode.lock();
  if (!node) {
    return;
  }
  int index = 0;
  this->ac.header.stamp = node->get_clock()->now();
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = -1.60;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;
  this->ac.position[index++] = 0.00;

  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
    this->atlasVersion > 4)
  {
    this->ac.position[index++] = 0.00;
  }

  this->ac.position[index++] = -this->ac.position[16];
  this->ac.position[index++] = -this->ac.position[17];
  this->ac.position[index++] = this->ac.position[18];
  this->ac.position[index++] = -this->ac.position[19];
  this->ac.position[index++] = this->ac.position[20];
  this->ac.position[index++] = -this->ac.position[21];

  if ((this->atlasVersion == 4 && this->atlasSubVersion == 0) ||
    this->atlasVersion > 4)
  {
    this->ac.position[index++] = this->ac.position[22];
  }

  this->pubAtlasCommand->publish(this->ac);
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(VRCPlugin,
              gz::sim::System,
              VRCPlugin::ISystemConfigure,
              VRCPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(VRCPlugin, "drcsim_gazebo_ros_plugins::VRCPlugin")
