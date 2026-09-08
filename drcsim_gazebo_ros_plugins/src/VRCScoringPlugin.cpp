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

#include "drcsim_gazebo_ros_plugins/VRCScoringPlugin.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/DetachableJoint.hh>

#include <sdf/Box.hh>

#include <builtin_interfaces/msg/time.hpp>

#include "drcsim_gazebo_ros_plugins/RosNodeOptions.hpp"

using DoubleSeconds = std::chrono::duration<double>;

namespace
{
builtin_interfaces::msg::Time ToRosTime(double _seconds)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(_seconds));
  t.nanosec = static_cast<uint32_t>((_seconds - t.sec) * 1e9);
  return t;
}
}  // namespace

namespace drcsim_gazebo_ros_plugins
{

/////////////////////////////////////////////////
VRCScoringPlugin::VRCScoringPlugin() = default;

/////////////////////////////////////////////////
VRCScoringPlugin::~VRCScoringPlugin()
{
  if (this->scoreFileStream.is_open()) {
    this->WriteScore(
      this->lastSimTime, std::chrono::system_clock::now(), "Shutting down", true);
  }
}

/////////////////////////////////////////////////
void VRCScoringPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->world = gz::sim::World(_entity);
  const std::string worldName = this->world.Name(_ecm).value_or("");
  gzmsg << "VRCScoringPlugin: world name is \"" << worldName << "\"" << std::endl;

  if (worldName == "qual_task_1") {
    this->worldType = WorldType::QUAL_1;
  } else if (worldName == "qual_task_2") {
    this->worldType = WorldType::QUAL_2;
  } else if (worldName == "qual_task_3") {
    this->worldType = WorldType::QUAL_3;
  } else if (worldName == "qual_task_4") {
    this->worldType = WorldType::QUAL_4;
  } else if (worldName == "vrc_task_1") {
    this->worldType = WorldType::VRC_1;
  } else if (worldName == "vrc_task_2") {
    this->worldType = WorldType::VRC_2;
  } else if (worldName == "vrc_task_3") {
    this->worldType = WorldType::VRC_3;
  } else {
    gzerr << "VRCScoringPlugin: unknown world name \"" << worldName
          << "\"; not scoring." << std::endl;
    return;
  }

  if (_sdf->HasElement("fall_accel_threshold")) {
    this->fallAccelThreshold = _sdf->Get<double>("fall_accel_threshold");
  }

  if (_sdf->HasElement("score_file")) {
    this->scoreFilePath = std::filesystem::path(_sdf->Get<std::string>("score_file"));
  } else {
    const char * homePath = std::getenv("HOME");
    this->scoreFilePath = homePath ?
      std::filesystem::path(homePath) : std::filesystem::path("/tmp/gazebo");
    this->scoreFilePath /= ".gazebo";
    this->scoreFilePath /= "scores";
    this->scoreFilePath /= worldName + ".score";
  }

  if (!std::filesystem::exists(this->scoreFilePath.parent_path())) {
    std::filesystem::create_directories(this->scoreFilePath.parent_path());
  }
  this->scoreFileStream.open(this->scoreFilePath.string(), std::fstream::out);
  if (!this->scoreFileStream.is_open()) {
    gzerr << "Failed to open score file: " << this->scoreFilePath << std::endl;
    return;
  }
  gzmsg << "Writing score data to " << this->scoreFilePath << std::endl;

  this->scoreFileStream << "# Score data for world " << worldName << std::endl;
  this->runStartTimeWall = std::chrono::system_clock::now();
  const std::time_t timeSec = std::chrono::system_clock::to_time_t(this->runStartTimeWall);
  this->scoreFileStream << "# Started at: " << std::fixed << std::setprecision(3)
                        << DoubleSeconds(this->runStartTimeWall.time_since_epoch()).count()
                        << "; " << std::ctime(&timeSec);
  this->scoreFileStream << "# Format: " << std::endl;
  this->scoreFileStream << "# wallTime(sec),simTime(sec),"
    "wallTimeElapsed(sec),simTimeElapsed(sec),completionScore(count),"
    "falls(count)" << std::endl;

  this->validConfig = true;

  // Sibling <model> entities declared in the same SDF world file (gates,
  // atlas, ...) are not guaranteed to exist yet at this point -- see
  // FindArenaStuff's doc comment. PreUpdate retries both that and the
  // atlas lookup, every tick, until each succeeds.
}

/////////////////////////////////////////////////
void VRCScoringPlugin::PreUpdate(
  const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & _ecm)
{
  if (!this->validConfig || this->atlasReady) {
    return;
  }

  if (!this->arenaReady) {
    if (!this->FindArenaStuff(_ecm)) {
      return;
    }
    this->arenaReady = true;
  }

  if (this->FindAtlas(_ecm)) {
    this->CompleteDeferredLoad();
    this->atlasReady = true;
  }
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindArenaStuff(gz::sim::EntityComponentManager & _ecm)
{
  switch (this->worldType) {
    case WorldType::QUAL_1:
      return this->FindGates(_ecm);
    case WorldType::QUAL_2:
      return this->FindQual2Stuff(_ecm);
    case WorldType::QUAL_3:
      return this->FindGates(_ecm);
    case WorldType::QUAL_4:
      return this->FindGates(_ecm);
    case WorldType::VRC_1:
      return this->FindVRC1Stuff(_ecm);
    case WorldType::VRC_2:
      return this->FindGates(_ecm);
    case WorldType::VRC_3:
      return this->FindVRC3Stuff(_ecm);
    default:
      return false;
  }
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindAtlas(gz::sim::EntityComponentManager & _ecm)
{
  if (this->atlasEntity == gz::sim::kNullEntity) {
    this->atlasEntity = this->world.ModelByName(_ecm, "atlas");
    if (this->atlasEntity == gz::sim::kNullEntity) {
      return false;
    }
  }
  if (this->atlasHeadEntity == gz::sim::kNullEntity) {
    gz::sim::Model atlasModel(this->atlasEntity);
    this->atlasHeadEntity = atlasModel.LinkByName(_ecm, "head");
    if (this->atlasHeadEntity == gz::sim::kNullEntity) {
      gzerr << "VRCScoringPlugin: unable to find head for scoring falls" << std::endl;
      return false;
    }
    gz::sim::Link(this->atlasHeadEntity).EnableVelocityChecks(_ecm, true);
  }
  return true;
}

/////////////////////////////////////////////////
void VRCScoringPlugin::CompleteDeferredLoad()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->rosNode = std::make_shared<rclcpp::Node>(
    "vrc_scoring_plugin", drcsim_gazebo_ros_plugins::RosNodeOptionsFromEnv());
  this->pubScore = this->rosNode->create_publisher<atlas_msgs::msg::VRCScore>(
    "vrc_score", rclcpp::QoS(1).transient_local());
}

/////////////////////////////////////////////////
void VRCScoringPlugin::PostUpdate(
  const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
{
  if (!this->validConfig || !this->atlasReady || _info.paused) {
    return;
  }

  this->lastSimTime = _info.simTime;
  const auto simTime = _info.simTime;
  const auto wallTime = std::chrono::system_clock::now();

  const int prevScore = this->completionScore;
  const int prevFalls = this->falls;
  std::string scoreMsg;
  bool forceLogScore = false;

  if (this->worldType == WorldType::QUAL_1 || this->worldType == WorldType::QUAL_3 ||
    this->worldType == WorldType::QUAL_4)
  {
    if (this->CheckNextGate(_ecm, scoreMsg)) {
      this->completionScore += 1;
    }
  } else if (this->worldType == WorldType::QUAL_2) {
    if (this->completionScore == 0) {
      if (this->CheckDrillInBin(_ecm, scoreMsg)) {
        this->completionScore += 1;
      }
    }
  } else if (this->worldType == WorldType::VRC_1) {
    const bool firstGate = (this->nextGate == this->gates.begin());
    if (firstGate) {
      if (this->CheckNextGate(_ecm, scoreMsg)) {
        forceLogScore = true;
        this->StartClock(simTime, wallTime, scoreMsg);
      }
    } else {
      if (this->completionScore == 0) {
        if (this->CheckAtlasInVehicle(_ecm, scoreMsg)) {
          this->completionScore += 1;
        }
      } else {
        if (this->CheckNextGate(_ecm, scoreMsg)) {
          this->completionScore += 1;
          if (this->nextGate == this->gates.end()) {
            this->StopClock(simTime, wallTime, scoreMsg);
          }
        }
      }
    }
  } else if (this->worldType == WorldType::VRC_2) {
    const bool firstGate = (this->nextGate == this->gates.begin());
    if (firstGate) {
      if (this->CheckNextGate(_ecm, scoreMsg)) {
        forceLogScore = true;
        this->StartClock(simTime, wallTime, scoreMsg);
      }
    } else {
      if (this->CheckNextGate(_ecm, scoreMsg)) {
        this->completionScore += 1;
        if (this->nextGate == this->gates.end()) {
          this->StopClock(simTime, wallTime, scoreMsg);
        }
      }
    }
  } else if (this->worldType == WorldType::VRC_3) {
    if (this->CheckNextGate(_ecm, scoreMsg)) {
      forceLogScore = true;
      this->StartClock(simTime, wallTime, scoreMsg);
    }

    const bool hoseAligned = this->CheckHoseAligned(_ecm, scoreMsg);
    const bool hoseConnected = this->CheckHoseConnected(_ecm, scoreMsg);

    if (this->completionScore == 0) {
      if (this->CheckHoseOffTable(_ecm, scoreMsg)) {
        this->completionScore += 1;
      }
    } else if (this->completionScore == 1) {
      if (hoseAligned) {
        this->completionScore += 1;
      }
    } else if (this->completionScore == 2) {
      if (hoseConnected) {
        this->completionScore += 1;
      }
    } else if (this->completionScore == 3) {
      if (this->CheckValveOpen(_ecm, scoreMsg)) {
        this->completionScore += 1;
        this->StopClock(simTime, wallTime, scoreMsg);
      }
    }
  }

  if (this->CheckFall(_ecm, simTime, scoreMsg)) {
    this->falls += 1;
  }

  if (prevScore != this->completionScore || prevFalls != this->falls) {
    forceLogScore = true;
  }
  this->WriteScore(simTime, wallTime, scoreMsg, forceLogScore);
}

/////////////////////////////////////////////////
std::optional<gz::math::AxisAlignedBox> VRCScoringPlugin::CollisionWorldBox(
  const gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _collisionEntity)
{
  const auto * elem = _ecm.Component<gz::sim::components::CollisionElement>(_collisionEntity);
  if (!elem || !elem->Data().Geom() || !elem->Data().Geom()->BoxShape()) {
    return std::nullopt;
  }

  const gz::math::Vector3d halfSize = elem->Data().Geom()->BoxShape()->Size() * 0.5;
  const gz::sim::Entity linkEntity = _ecm.ParentEntity(_collisionEntity);
  const gz::math::Pose3d colWorldPose =
    gz::sim::worldPose(linkEntity, _ecm) * elem->Data().RawPose();

  gz::math::Vector3d boxMin;
  gz::math::Vector3d boxMax;
  bool first = true;
  for (const double sx : {-1.0, 1.0}) {
    for (const double sy : {-1.0, 1.0}) {
      for (const double sz : {-1.0, 1.0}) {
        const gz::math::Vector3d localCorner(
          sx * halfSize.X(), sy * halfSize.Y(), sz * halfSize.Z());
        const gz::math::Vector3d corner =
          colWorldPose.Pos() + colWorldPose.Rot().RotateVector(localCorner);
        if (first) {
          boxMin = corner;
          boxMax = corner;
          first = false;
        } else {
          boxMin = gz::math::Vector3d(
            std::min(boxMin.X(), corner.X()), std::min(boxMin.Y(), corner.Y()),
            std::min(boxMin.Z(), corner.Z()));
          boxMax = gz::math::Vector3d(
            std::max(boxMax.X(), corner.X()), std::max(boxMax.Y(), corner.Y()),
            std::max(boxMax.Z(), corner.Z()));
        }
      }
    }
  }
  return gz::math::AxisAlignedBox(boxMin, boxMax);
}

/////////////////////////////////////////////////
int VRCScoringPlugin::IsPoseInGate(
  const gz::math::Pose3d & _robotWorldPose, const gz::math::Pose3d & _gateWorldPose,
  double _gateWidth)
{
  const gz::math::Vector3d robotLocalPosition =
    _gateWorldPose.Rot().Inverse().RotateVector(_robotWorldPose.Pos() - _gateWorldPose.Pos());

  if (std::fabs(robotLocalPosition.Y()) <= _gateWidth / 2.0) {
    return (robotLocalPosition.X() >= 0.0) ? 1 : -1;
  }
  return 0;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckNextGate(
  const gz::sim::EntityComponentManager & _ecm, std::string & _msg)
{
  if (this->nextGate == this->gates.end()) {
    return false;
  }

  gz::math::Pose3d pose;
  std::string tmpString;
  switch (this->nextGate->type) {
    case Gate::GateType::PEDESTRIAN:
      // We require that Atlas is NOT in the vehicle when it crosses this gate.
      if (this->CheckAtlasInVehicle(_ecm, tmpString)) {
        return false;
      }
      pose = gz::sim::worldPose(this->atlasEntity, _ecm);
      break;
    case Gate::GateType::VEHICLE:
      // We require that Atlas is in the vehicle when it crosses this gate.
      if (!this->CheckAtlasInVehicle(_ecm, tmpString)) {
        return false;
      }
      pose = gz::sim::worldPose(this->vehicleEntity, _ecm);
      break;
    default:
      return false;
  }

  const int gateSide = IsPoseInGate(pose, this->nextGate->pose, this->nextGate->width);
  if (this->nextGateSide < 0 && gateSide > 0) {
    std::stringstream ss;
    ss << "Successfully passed through gate " << (this->nextGate->number + 1) << ". ";
    gzmsg << ss.str() << std::endl;
    _msg += ss.str();

    ++this->nextGate;
    this->nextGateSide = 0;
    return true;
  } else {
    if (this->nextGateSide > 0 && gateSide < 0) {
      gzmsg << "Went backward through gate " << (this->nextGate->number + 1) << std::endl;
    }
    this->nextGateSide = gateSide;
  }
  return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckAtlasInVehicle(
  const gz::sim::EntityComponentManager & _ecm, std::string & _msg)
{
  if (this->vehicleSeatCollision == gz::sim::kNullEntity) {
    return false;
  }

  const gz::math::Vector3d robotPosition = gz::sim::worldPose(this->atlasEntity, _ecm).Pos();

  const auto seatBox = CollisionWorldBox(_ecm, this->vehicleSeatCollision);
  const auto seatBackBox = CollisionWorldBox(_ecm, this->vehicleSeatBackCollision);
  if (!seatBox || !seatBackBox) {
    return false;
  }

  gz::math::Vector3d boxMin = seatBox->Min();
  gz::math::Vector3d boxMax = seatBox->Max();
  boxMin.Z(seatBackBox->Min().Z());
  boxMax.Z(seatBackBox->Max().Z());

  if (robotPosition.X() >= boxMin.X() && robotPosition.X() <= boxMax.X() &&
    robotPosition.Y() >= boxMin.Y() && robotPosition.Y() <= boxMax.Y() &&
    robotPosition.Z() >= boxMin.Z() && robotPosition.Z() <= boxMax.Z())
  {
    std::stringstream ss;
    ss << "Successfully moved Atlas into vehicle. ";
    _msg += ss.str();
    gzmsg << ss.str() << std::endl;
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckDrillInBin(
  const gz::sim::EntityComponentManager & _ecm, std::string & _msg)
{
  if (this->completionScore != 0) {
    return false;
  }
  const gz::math::Vector3d drillPosition = gz::sim::worldPose(this->drillEntity, _ecm).Pos();
  if (this->bin.Contains(drillPosition)) {
    std::stringstream ss;
    ss << "Successfully placed drill in bin. ";
    _msg += ss.str();
    gzmsg << ss.str() << std::endl;
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckFall(
  const gz::sim::EntityComponentManager & _ecm,
  const std::chrono::steady_clock::duration & _simTime, std::string & _msg)
{
  // Don't count falls after task completion + quiet time.
  if (this->stopTimeSim != std::chrono::steady_clock::duration::zero() &&
    (_simTime - this->stopTimeSim) >= this->postCompletionQuietTime)
  {
    return false;
  }

  const auto vel = gz::sim::Link(this->atlasHeadEntity).WorldLinearVelocity(_ecm);
  if (!vel) {
    return false;
  }
  const gz::math::Vector3d currVel = *vel;

  // Don't declare a fall if we had one recently. This check also handles
  // initial conditions, which currently include dropping the robot onto
  // the ground at t=0.
  if ((_simTime - this->prevFallTime) < std::chrono::seconds(15)) {
    this->prevVelTime = _simTime;
    this->prevLinearVel = currVel;
    return false;
  }

  // Differentiate to get acceleration.
  const double dt = DoubleSeconds(_simTime - this->prevVelTime).count();
  const double accel = (currVel.Z() - this->prevLinearVel.Z()) / dt;
  this->prevVelTime = _simTime;
  this->prevLinearVel = currVel;
  if (std::fabs(accel) > this->fallAccelThreshold) {
    std::stringstream ss;
    ss << "Damaging fall detected, acceleration of: " << accel << " m/s^2. ";
    gzmsg << ss.str() << std::endl;
    _msg += ss.str();
    this->prevFallTime = _simTime;
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckHoseOffTable(
  const gz::sim::EntityComponentManager & _ecm, std::string & _msg)
{
  // Check that the height of the hose coupler is within a few cm of the
  // standpipe.
  const double standpipeZ = gz::sim::worldPose(this->standpipeEntity, _ecm).Pos().Z();
  const double couplerZ = gz::sim::worldPose(this->hoseCouplerEntity, _ecm).Pos().Z();
  if (couplerZ >= (standpipeZ - 0.05)) {
    std::stringstream ss;
    ss << "Successfully picked hose up off table. ";
    gzmsg << ss.str() << std::endl;
    _msg += ss.str();
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckHoseAligned(
  const gz::sim::EntityComponentManager & _ecm, std::string & _msg)
{
  // Check whether a detachable joint (VRCPlugin's stand-in for the
  // original's Classic screw joint -- see VRCPlugin's design notes) has
  // been created from the standpipe. That's true only once VRCPlugin has
  // decided the coupler and spout are aligned.
  bool attached = false;
  _ecm.Each<gz::sim::components::DetachableJoint>(
    [&](const gz::sim::Entity &, const gz::sim::components::DetachableJoint * _dj) -> bool
    {
      if (_dj->Data().parentLink == this->standpipeEntity) {
        attached = true;
        return false;
      }
      return true;
    });

  if (attached) {
    if (!this->isHoseAligned) {
      std::stringstream ss;
      ss << "Successfully aligned the hose with the standpipe. ";
      gzmsg << ss.str() << std::endl;
      _msg += ss.str();
      this->hoseCouplerAlignedPose = gz::sim::worldPose(this->hoseCouplerEntity, _ecm);
      this->isHoseAligned = true;
      this->isHoseConnected = false;
    }
    return true;
  } else {
    if (this->isHoseAligned) {
      gzmsg << "Unaligned the hose from the standpipe" << std::endl;
      this->isHoseAligned = false;
      this->isHoseConnected = false;
    }
    return false;
  }
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckHoseConnected(
  const gz::sim::EntityComponentManager & _ecm, std::string & _msg)
{
  // Must be aligned (i.e., the weld joint must exist).
  if (!this->isHoseAligned) {
    return false;
  }

  // Check for a sufficient change in coupler position along its X axis,
  // which indicates that the hose coupler has threaded on to the
  // standpipe screw joint.
  const gz::math::Vector3d couplerWorldPosition =
    gz::sim::worldPose(this->hoseCouplerEntity, _ecm).Pos();
  const gz::math::Vector3d couplerLocalPosition =
    this->hoseCouplerAlignedPose.Rot().Inverse().RotateVector(
      couplerWorldPosition - this->hoseCouplerAlignedPose.Pos());
  const double dist = couplerLocalPosition.X();
  // Maximum depth is 2cm; let's get most of the way there.
  if (dist <= -0.015) {
    if (!this->isHoseConnected) {
      std::stringstream ss;
      ss << "Successfully connected the hose to the standpipe. ";
      gzmsg << ss.str() << std::endl;
      _msg += ss.str();
      this->isHoseConnected = true;
    }
    return true;
  } else {
    // Allow for a little bit (2mm) of hysteresis after connection.
    if (this->isHoseConnected && dist > -0.013) {
      gzmsg << "Disconnected the hose to the standpipe" << std::endl;
      this->isHoseConnected = false;
    }
    return false;
  }
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::CheckValveOpen(
  const gz::sim::EntityComponentManager & _ecm, std::string & _msg)
{
  // Doesn't count unless the hose is connected.
  if (!this->isHoseConnected) {
    return false;
  }

  const auto positions = gz::sim::Joint(this->valveJointEntity).Position(_ecm);
  if (!positions || positions->empty()) {
    return false;
  }

  // The valve starts at 0 and can be turned CCW several rotations. We
  // check that it's been turned at least one rotation.
  if ((*positions)[0] < -2.0 * M_PI) {
    std::stringstream ss;
    ss << "Successfully opened valve. ";
    gzmsg << ss.str() << std::endl;
    _msg += ss.str();
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void VRCScoringPlugin::StartClock(
  const std::chrono::steady_clock::duration & _simTime,
  const std::chrono::system_clock::time_point & _wallTime, std::string & _msg)
{
  this->startTimeSim = _simTime;
  this->startTimeWall = _wallTime;
  std::stringstream ss;
  ss << "Starting clock. ";
  gzmsg << ss.str() << std::endl;
  _msg += ss.str();
}

/////////////////////////////////////////////////
void VRCScoringPlugin::StopClock(
  const std::chrono::steady_clock::duration & _simTime,
  const std::chrono::system_clock::time_point & _wallTime, std::string & _msg)
{
  this->stopTimeSim = _simTime;
  this->stopTimeWall = _wallTime;
  std::stringstream ss;
  ss << "Stopping clock. ";
  gzmsg << ss.str() << std::endl;
  _msg += ss.str();
}

/////////////////////////////////////////////////
void VRCScoringPlugin::WriteScore(
  const std::chrono::steady_clock::duration & _simTime,
  const std::chrono::system_clock::time_point & _wallTime,
  const std::string & _msg, bool _force)
{
  // Write at 1Hz.
  if (!_force && DoubleSeconds(_simTime - this->prevScoreTime).count() < 1.0) {
    return;
  }

  if (!this->scoreFileStream.is_open()) {
    gzerr << "Score file stream is no longer open: " << this->scoreFilePath << std::endl;
    return;
  }

  // If we've passed the first gate, compute elapsed time.
  std::chrono::steady_clock::duration elapsedTimeSim{std::chrono::steady_clock::duration::zero()};
  if (this->stopTimeSim != std::chrono::steady_clock::duration::zero()) {
    elapsedTimeSim = this->stopTimeSim - this->startTimeSim;
  } else if (this->startTimeSim != std::chrono::steady_clock::duration::zero()) {
    elapsedTimeSim = _simTime - this->startTimeSim;
  }

  std::chrono::system_clock::duration elapsedTimeWall{std::chrono::system_clock::duration::zero()};
  if (this->stopTimeWall != std::chrono::system_clock::time_point{}) {
    elapsedTimeWall = this->stopTimeWall - this->startTimeWall;
  } else if (this->startTimeWall != std::chrono::system_clock::time_point{}) {
    elapsedTimeWall = _wallTime - this->startTimeWall;
  }

  const auto runElapsedTimeWall = _wallTime - this->runStartTimeWall;

  this->scoreFileStream << std::fixed << std::setprecision(3)
                        << DoubleSeconds(runElapsedTimeWall).count() << ","
                        << DoubleSeconds(_simTime).count() << ","
                        << DoubleSeconds(elapsedTimeWall).count() << ","
                        << DoubleSeconds(elapsedTimeSim).count() << ","
                        << this->completionScore << ","
                        << this->falls << ",\"" << _msg << "\"" << std::endl;

  // Also publish via ROS.
  if (this->pubScore) {
    atlas_msgs::msg::VRCScore rosScoreMsg;
    rosScoreMsg.wall_time = ToRosTime(DoubleSeconds(runElapsedTimeWall).count());
    rosScoreMsg.sim_time = ToRosTime(DoubleSeconds(_simTime).count());
    rosScoreMsg.wall_time_elapsed = ToRosTime(DoubleSeconds(elapsedTimeWall).count());
    rosScoreMsg.sim_time_elapsed = ToRosTime(DoubleSeconds(elapsedTimeSim).count());
    rosScoreMsg.completion_score = this->completionScore;
    rosScoreMsg.falls = this->falls;
    rosScoreMsg.message = _msg;
    if (this->worldType == WorldType::VRC_1) {
      rosScoreMsg.task_type = atlas_msgs::msg::VRCScore::TASK_DRIVING;
    } else if (this->worldType == WorldType::VRC_2) {
      rosScoreMsg.task_type = atlas_msgs::msg::VRCScore::TASK_WALKING;
    } else if (this->worldType == WorldType::VRC_3) {
      rosScoreMsg.task_type = atlas_msgs::msg::VRCScore::TASK_MANIPULATION;
    } else {
      rosScoreMsg.task_type = atlas_msgs::msg::VRCScore::TASK_OTHER;
    }
    this->pubScore->publish(rosScoreMsg);
  }

  this->prevScoreTime = _simTime;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindQual2Stuff(gz::sim::EntityComponentManager & _ecm)
{
  this->drillEntity = this->world.ModelByName(_ecm, "drill");
  if (this->drillEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find drill" << std::endl;
    return false;
  }

  const gz::sim::Entity binModelEntity = this->world.ModelByName(_ecm, "bin");
  if (binModelEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find bin" << std::endl;
    return false;
  }

  gz::sim::Model binModel(binModelEntity);
  const gz::sim::Entity binLinkEntity = binModel.LinkByName(_ecm, "link");
  if (binLinkEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find bin link" << std::endl;
    return false;
  }

  gz::sim::Link binLink(binLinkEntity);
  const gz::sim::Entity bottomCollisionEntity = binLink.CollisionByName(_ecm, "bottom_collision");
  if (bottomCollisionEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find bin bottom collision" << std::endl;
    return false;
  }
  const auto bottomBox = CollisionWorldBox(_ecm, bottomCollisionEntity);
  if (!bottomBox) {
    gzerr << "Failed to compute bin bottom collision bounding box" << std::endl;
    return false;
  }

  const gz::sim::Entity side1CollisionEntity = binLink.CollisionByName(_ecm, "side1_collision");
  if (side1CollisionEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find bin side1 collision" << std::endl;
    return false;
  }
  const auto side1Box = CollisionWorldBox(_ecm, side1CollisionEntity);
  if (!side1Box) {
    gzerr << "Failed to compute bin side1 collision bounding box" << std::endl;
    return false;
  }

  // Give a bit of tolerance for possible offset in origin of drill.
  this->bin = gz::math::AxisAlignedBox(
    gz::math::Vector3d(bottomBox->Min().X(), bottomBox->Min().Y(), bottomBox->Min().Z() - 0.15),
    gz::math::Vector3d(bottomBox->Max().X(), bottomBox->Max().Y(), side1Box->Max().Z()));

  return true;
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindVRC1Stuff(gz::sim::EntityComponentManager & _ecm)
{
  this->vehicleEntity = this->world.ModelByName(_ecm, "drc_vehicle");
  if (this->vehicleEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find vehicle" << std::endl;
    return false;
  }

  gz::sim::Model vehicleModel(this->vehicleEntity);
  const gz::sim::Entity chassisLinkEntity = vehicleModel.LinkByName(_ecm, "chassis");
  if (chassisLinkEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find chassis link" << std::endl;
    return false;
  }

  gz::sim::Link chassisLink(chassisLinkEntity);
  this->vehicleSeatCollision = chassisLink.CollisionByName(_ecm, "seat");
  if (this->vehicleSeatCollision == gz::sim::kNullEntity) {
    gzerr << "Failed to find vehicle seat collision" << std::endl;
    return false;
  }
  this->vehicleSeatBackCollision = chassisLink.CollisionByName(_ecm, "seat_back");
  if (this->vehicleSeatBackCollision == gz::sim::kNullEntity) {
    gzerr << "Failed to find vehicle seat back collision" << std::endl;
    return false;
  }

  return this->FindGates(_ecm);
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindVRC3Stuff(gz::sim::EntityComponentManager & _ecm)
{
  const gz::sim::Entity hoseModelEntity = this->world.ModelByName(_ecm, "vrc_firehose_long");
  if (hoseModelEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find hose" << std::endl;
    return false;
  }
  gz::sim::Model hoseModel(hoseModelEntity);
  this->hoseCouplerEntity = hoseModel.LinkByName(_ecm, "coupling");
  if (this->hoseCouplerEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find hose coupler" << std::endl;
    return false;
  }

  const gz::sim::Entity standpipeModelEntity = this->world.ModelByName(_ecm, "standpipe");
  if (standpipeModelEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find standpipe model" << std::endl;
    return false;
  }
  gz::sim::Model standpipeModel(standpipeModelEntity);
  this->standpipeEntity = standpipeModel.LinkByName(_ecm, "standpipe");
  if (this->standpipeEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find standpipe link" << std::endl;
    return false;
  }

  const gz::sim::Entity valveModelEntity = this->world.ModelByName(_ecm, "valve");
  if (valveModelEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find valve model" << std::endl;
    return false;
  }
  gz::sim::Model valveModel(valveModelEntity);
  this->valveJointEntity = valveModel.JointByName(_ecm, "valve");
  if (this->valveJointEntity == gz::sim::kNullEntity) {
    gzerr << "Failed to find valve joint" << std::endl;
    return false;
  }
  gz::sim::Joint(this->valveJointEntity).EnablePositionCheck(_ecm, true);

  this->isHoseAligned = false;
  this->isHoseConnected = false;

  return this->FindGates(_ecm);
}

/////////////////////////////////////////////////
bool VRCScoringPlugin::FindGates(gz::sim::EntityComponentManager & _ecm)
{
  // Walk through the world and accumulate the things that appear to be
  // gates. We assume that gates are named '[vehicle]gate_<int>'.
  const std::vector<gz::sim::Entity> models = this->world.Models(_ecm);
  for (const gz::sim::Entity modelEntity : models) {
    gz::sim::Model model(modelEntity);
    const std::string name = model.Name(_ecm);

    const std::string::size_type underscore = name.rfind('_');
    if (underscore == std::string::npos) {
      continue;
    }
    const std::string prefix = name.substr(0, underscore);
    const std::string numberStr = name.substr(underscore + 1);
    if (prefix != "gate" && prefix != "vehiclegate") {
      continue;
    }

    unsigned int gateNum = 0;
    try {
      std::size_t consumed = 0;
      gateNum = static_cast<unsigned int>(std::stoul(numberStr, &consumed));
      if (consumed != numberStr.size()) {
        gzwarn << "Ignoring gate name that failed to parse: " << name << std::endl;
        continue;
      }
    } catch (const std::exception &) {
      gzwarn << "Ignoring gate name that failed to parse: " << name << std::endl;
      continue;
    }

    // Determine width of gate: the larger of the X and Y dimensions of the
    // merged bounding box of all box-shaped collisions in the model.
    gz::math::Vector3d boxMin;
    gz::math::Vector3d boxMax;
    bool haveBox = false;
    for (const gz::sim::Entity linkEntity : model.Links(_ecm)) {
      gz::sim::Link link(linkEntity);
      for (const gz::sim::Entity colEntity : link.Collisions(_ecm)) {
        const auto colBox = CollisionWorldBox(_ecm, colEntity);
        if (!colBox) {
          continue;
        }
        if (!haveBox) {
          boxMin = colBox->Min();
          boxMax = colBox->Max();
          haveBox = true;
        } else {
          boxMin = gz::math::Vector3d(
            std::min(boxMin.X(), colBox->Min().X()), std::min(boxMin.Y(), colBox->Min().Y()),
            std::min(boxMin.Z(), colBox->Min().Z()));
          boxMax = gz::math::Vector3d(
            std::max(boxMax.X(), colBox->Max().X()), std::max(boxMax.Y(), colBox->Max().Y()),
            std::max(boxMax.Z(), colBox->Max().Z()));
        }
      }
    }
    if (!haveBox) {
      continue;
    }
    const gz::math::Vector3d bboxSize = boxMax - boxMin;
    const double gateWidth = std::max(bboxSize.X(), bboxSize.Y());

    const Gate::GateType gateType =
      (prefix == "vehiclegate") ? Gate::GateType::VEHICLE : Gate::GateType::PEDESTRIAN;

    const Gate g(name, gateType, gateNum, gz::sim::worldPose(modelEntity, _ecm), gateWidth);
    this->gates.push_back(g);
    gzmsg << "Stored gate named " << g.name << " with index " << g.number
          << " and width " << g.width << std::endl;
  }

  if (this->gates.empty()) {
    gzerr << "Found no gates." << std::endl;
    this->nextGate = this->gates.end();
    return false;
  }

  // Sort in order of increasing gate number (in case we encountered them
  // out-of-order in the list of models).
  this->gates.sort();
  // Set the first gate we're looking for.
  this->nextGate = this->gates.begin();
  this->nextGateSide = -1;

  return true;
}

}  // namespace drcsim_gazebo_ros_plugins

GZ_ADD_PLUGIN(
  drcsim_gazebo_ros_plugins::VRCScoringPlugin,
  gz::sim::System,
  drcsim_gazebo_ros_plugins::VRCScoringPlugin::ISystemConfigure,
  drcsim_gazebo_ros_plugins::VRCScoringPlugin::ISystemPreUpdate,
  drcsim_gazebo_ros_plugins::VRCScoringPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  drcsim_gazebo_ros_plugins::VRCScoringPlugin, "drcsim_gazebo_ros_plugins::VRCScoringPlugin")
