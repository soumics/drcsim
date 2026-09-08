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
#include "drcsim_gazebo_ros_plugins/AtlasPlugin.hpp"

#include <gz/msgs/contacts.pb.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/JointAxis.hh>

#include "drcsim_gazebo_ros_plugins/RosNodeOptions.hpp"

using drcsim_gazebo_ros_plugins::AtlasPlugin;

//////////////////////////////////////////////////
AtlasPlugin::AtlasPlugin()
{
  this->delayWindowStart = std::chrono::steady_clock::now();
}

//////////////////////////////////////////////////
AtlasPlugin::~AtlasPlugin()
{
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->rosSpinThread.joinable()) {
    this->rosSpinThread.join();
  }
}

//////////////////////////////////////////////////
double AtlasPlugin::FirstOrZero(
  const std::optional<std::vector<double>> & _values)
{
  if (_values && !_values->empty()) {
    return (*_values)[0];
  }
  return 0.0;
}

//////////////////////////////////////////////////
void AtlasPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "AtlasPlugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->sdfConfig = _sdf;
}

//////////////////////////////////////////////////
void AtlasPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized && this->sdfConfig) {
    this->Load(_ecm);
    this->initialized = true;
  }

  if (this->validConfig) {
    this->UpdateStates(_info, _ecm);
  }
}

//////////////////////////////////////////////////
bool AtlasPlugin::GetAtlasVersion()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->rosNode = std::make_shared<rclcpp::Node>(
    "atlas_plugin", drcsim_gazebo_ros_plugins::RosNodeOptionsFromEnv());

  this->atlasVersion = this->rosNode->declare_parameter("atlas_version", 5);
  this->atlasSubVersion =
    this->rosNode->declare_parameter("atlas_sub_version", 0);
  return true;
}

//////////////////////////////////////////////////
std::string AtlasPlugin::FindJoint(
  gz::sim::EntityComponentManager & _ecm, const std::string & _st1,
  const std::string & _st2)
{
  if (this->model.JointByName(_ecm, _st1) != gz::sim::kNullEntity) {
    return _st1;
  }
  if (this->model.JointByName(_ecm, _st2) != gz::sim::kNullEntity) {
    return _st2;
  }
  RCLCPP_INFO(
    this->rosNode->get_logger(),
    "Atlas[XX]Plugin: joint by names [%s] or [%s] not found, returning "
    "empty string.", _st1.c_str(), _st2.c_str());
  return std::string();
}

//////////////////////////////////////////////////
std::string AtlasPlugin::FindJoint(
  gz::sim::EntityComponentManager & _ecm, const std::string & _st1,
  const std::string & _st2, const std::string & _st3)
{
  return this->FindJoint(_ecm, this->FindJoint(_ecm, _st1, _st2), _st3);
}

//////////////////////////////////////////////////
void AtlasPlugin::Load(gz::sim::EntityComponentManager & _ecm)
{
  if (!this->GetAtlasVersion()) {
    return;
  }

  const char * cheatsEnabledString = std::getenv("VRC_CHEATS_ENABLED");
  this->cheatsEnabled =
    cheatsEnabledString && (std::string(cheatsEnabledString) == "1");

  // Hardcoded joint list for Atlas.
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

  this->joints.resize(this->jointNames.size());
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    this->joints[i] = this->model.JointByName(_ecm, this->jointNames[i]);
    if (this->joints[i] == gz::sim::kNullEntity) {
      RCLCPP_ERROR(
        this->rosNode->get_logger(),
        "atlas robot expected joint[%s] not present, plugin not loaded",
        this->jointNames[i].c_str());
      return;
    }
    gz::sim::Joint jointWrapper(this->joints[i]);
    jointWrapper.EnablePositionCheck(_ecm);
    jointWrapper.EnableVelocityCheck(_ecm);
    jointWrapper.EnableTransmittedWrenchCheck(_ecm);
  }

  this->effortLimit.resize(this->jointNames.size());
  for (unsigned int i = 0; i < this->effortLimit.size(); ++i) {
    double limit = 100.0;
    const auto axes = gz::sim::Joint(this->joints[i]).Axis(_ecm);
    if (axes && !axes->empty() && std::isfinite((*axes)[0].Effort())) {
      limit = (*axes)[0].Effort();
    }
    this->effortLimit[i] = limit;
  }

  this->errorTerms.assign(this->joints.size(), ErrorTerms());
  this->asiErrorTerms.assign(this->joints.size(), ErrorTerms());

  {
    static const double jointDampingLowerBound = 0.0;
    for (unsigned int i = 0; i < this->jointNames.size(); ++i) {
      const double maxEffort = this->effortLimit[i];
      double maxVelocity = 100.0;
      const auto axes = gz::sim::Joint(this->joints[i]).Axis(_ecm);
      if (axes && !axes->empty() && std::isfinite((*axes)[0].MaxVelocity()) &&
        (*axes)[0].MaxVelocity() > 0.0)
      {
        maxVelocity = (*axes)[0].MaxVelocity();
      }

      this->jointDampingMax.push_back(maxEffort / maxVelocity * 50.0);
      this->jointDampingMin.push_back(jointDampingLowerBound);

      double damping = 0.0;
      if (axes && !axes->empty()) {
        damping = (*axes)[0].Damping();
      }
      this->jointDampingModel.push_back(damping);
      this->lastJointCFMDamping.push_back(damping);
    }
  }

  {
    this->atlasState.position.resize(this->joints.size());
    this->atlasState.velocity.resize(this->joints.size());
    this->atlasState.effort.resize(this->joints.size());
    this->atlasState.kp_position.resize(this->joints.size());
    this->atlasState.ki_position.resize(this->joints.size());
    this->atlasState.kd_position.resize(this->joints.size());
    this->atlasState.kp_velocity.resize(this->joints.size());
    this->atlasState.i_effort_min.resize(this->joints.size());
    this->atlasState.i_effort_max.resize(this->joints.size());
    this->atlasState.k_effort.resize(this->joints.size());

    this->jointStates.name = this->jointNames;
    this->jointStates.position.resize(this->joints.size());
    this->jointStates.velocity.resize(this->joints.size());
    this->jointStates.effort.resize(this->joints.size());
  }

  {
    this->atlasCommand.position.resize(this->joints.size());
    this->atlasCommand.velocity.resize(this->joints.size());
    this->atlasCommand.effort.resize(this->joints.size());
    this->atlasCommand.kp_position.resize(this->joints.size());
    this->atlasCommand.ki_position.resize(this->joints.size());
    this->atlasCommand.kd_position.resize(this->joints.size());
    this->atlasCommand.kp_velocity.resize(this->joints.size());
    this->atlasCommand.i_effort_min.resize(this->joints.size());
    this->atlasCommand.i_effort_max.resize(this->joints.size());
    this->atlasCommand.k_effort.resize(this->joints.size());
    this->ZeroAtlasCommand();
  }

  {
    this->asiState.header.stamp = rclcpp::Time();
    this->asiState.error_code = atlas_msgs::msg::AtlasSimInterfaceState::NO_ERRORS;
    this->asiState.current_behavior = -1;
    this->asiState.desired_behavior = -1;
    this->asiState.k_effort.assign(this->jointNames.size(), 255);

    this->asiCommand.position.resize(this->joints.size());
    this->asiCommand.velocity.resize(this->joints.size());
    this->asiCommand.effort.resize(this->joints.size());
    this->asiCommand.kp_position.resize(this->joints.size());
    this->asiCommand.ki_position.resize(this->joints.size());
    this->asiCommand.kp_velocity.resize(this->joints.size());
  }

  // Force torque joints.
  this->lWristJointEntity =
    this->model.JointByName(_ecm, this->FindJoint(_ecm, "l_arm_wrx", "l_arm_mwx"));
  this->rWristJointEntity =
    this->model.JointByName(_ecm, this->FindJoint(_ecm, "r_arm_wrx", "r_arm_mwx"));
  this->lAnkleJointEntity =
    this->model.JointByName(_ecm, this->FindJoint(_ecm, "l_leg_akx", "l_leg_lax"));
  this->rAnkleJointEntity =
    this->model.JointByName(_ecm, this->FindJoint(_ecm, "r_leg_akx", "r_leg_lax"));
  for (const gz::sim::Entity e :
    {this->lWristJointEntity, this->rWristJointEntity,
      this->lAnkleJointEntity, this->rAnkleJointEntity})
  {
    if (e != gz::sim::kNullEntity) {
      gz::sim::Joint(e).EnableTransmittedWrenchCheck(_ecm);
    }
  }

  // IMU link (parent link of imu_sensor after fixed-joint reduction).
  this->imuLinkEntity = this->model.LinkByName(_ecm, "pelvis");
  if (this->imuLinkEntity == gz::sim::kNullEntity) {
    gzerr << "pelvis link (for imu) not found" << std::endl;
  } else {
    gz::sim::Link imuLink(this->imuLinkEntity);
    imuLink.EnableVelocityChecks(_ecm);
    imuLink.EnableAccelerationChecks(_ecm);
  }

  // Force-create ContactSensorData on foot collisions for the debug contact
  // topics (see the class-level design note).
  for (const auto & pair : std::vector<std::pair<std::string, std::vector<gz::sim::Entity> *>>{
    {"l_foot", &this->lFootCollisions}, {"r_foot", &this->rFootCollisions}})
  {
    const gz::sim::Entity footLink = this->model.LinkByName(_ecm, pair.first);
    if (footLink == gz::sim::kNullEntity) {
      gzerr << pair.first << " link not found" << std::endl;
      continue;
    }
    const std::vector<gz::sim::Entity> collisions = _ecm.ChildrenByComponents(
      footLink, gz::sim::components::Collision());
    for (const gz::sim::Entity col : collisions) {
      if (!_ecm.EntityHasComponentType(
          col, gz::sim::components::ContactSensorData::typeId))
      {
        _ecm.CreateComponent(col, gz::sim::components::ContactSensorData());
      }
      pair.second->push_back(col);
    }
  }

  this->LoadROS();
  this->validConfig = true;
}

//////////////////////////////////////////////////
void AtlasPlugin::LoadROS()
{
  this->LoadPIDGainsFromParameter();

  this->atlasCommandAgeBufferDuration = this->rosNode->declare_parameter(
    "atlas_controller.statistics_time_window_size", 1.0);
  const double stepSize = 0.001;
  unsigned int bufferSize = static_cast<unsigned int>(
    this->atlasCommandAgeBufferDuration / stepSize);
  if (bufferSize == 0) {
    bufferSize = 1;
  }
  this->atlasCommandAgeBuffer.assign(bufferSize, 0.0);
  this->atlasCommandAgeDelta2Buffer.assign(bufferSize, 0.0);

  if (this->cheatsEnabled) {
    this->delayWindowSize =
      this->rosNode->declare_parameter("atlas.delay_window_size", 5.0);
    this->delayMaxPerWindow =
      this->rosNode->declare_parameter("atlas.delay_max_per_window", 0.25);
    this->delayMaxPerStep =
      this->rosNode->declare_parameter("atlas.delay_max_per_step", 0.025);
  }

  this->statsUpdateRate = std::clamp(
    this->rosNode->declare_parameter(
      "atlas.controller_statistics.update_rate", 1000.0),
    1.0, 10000.0);

  ////////////////////////////////////////////////////////////////
  // ROS Publishers
  ////////////////////////////////////////////////////////////////
  if (this->cheatsEnabled) {
    this->pubLFootContact =
      this->rosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "atlas/debug/l_foot_contact", 10);
    this->pubRFootContact =
      this->rosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "atlas/debug/r_foot_contact", 10);
  }

  this->pubDelayStatistics =
    this->rosNode->create_publisher<atlas_msgs::msg::SynchronizationStatistics>(
    "atlas/synchronization_statistics", rclcpp::QoS(100).transient_local());

  this->pubControllerStatistics =
    this->rosNode->create_publisher<atlas_msgs::msg::ControllerStatistics>(
    "atlas/controller_statistics", 10);

  this->pubImu = this->rosNode->create_publisher<sensor_msgs::msg::Imu>(
    "atlas/imu", 10);

  this->pubForceTorqueSensors =
    this->rosNode->create_publisher<atlas_msgs::msg::ForceTorqueSensors>(
    "atlas/force_torque_sensors", 10);

  this->pubJointStates = this->rosNode->create_publisher<sensor_msgs::msg::JointState>(
    "atlas/joint_states", 1);

  this->pubAtlasState =
    this->rosNode->create_publisher<atlas_msgs::msg::AtlasState>(
    "atlas/atlas_state", rclcpp::QoS(100).transient_local());

  this->pubASIState =
    this->rosNode->create_publisher<atlas_msgs::msg::AtlasSimInterfaceState>(
    "atlas/atlas_sim_interface_state", 1);

  ////////////////////////////////////////////////////////////////
  // ROS Subscribers
  ////////////////////////////////////////////////////////////////
  if (this->cheatsEnabled) {
    this->subTic = this->rosNode->create_subscription<std_msgs::msg::String>(
      "atlas/debug/sync_delay", 1,
      std::bind(&AtlasPlugin::Tic, this, std::placeholders::_1));

    this->subTest = this->rosNode->create_subscription<atlas_msgs::msg::Test>(
      "atlas/debug/test", 1,
      std::bind(&AtlasPlugin::SetExperimentalDampingPID, this, std::placeholders::_1));
  }

  this->subAtlasCommand =
    this->rosNode->create_subscription<atlas_msgs::msg::AtlasCommand>(
    "atlas/atlas_command", rclcpp::QoS(100),
    std::bind(&AtlasPlugin::SetAtlasCommand, this, std::placeholders::_1));

  this->subJointCommands =
    this->rosNode->create_subscription<osrf_msgs::msg::JointCommands>(
    "atlas/joint_commands", rclcpp::QoS(1),
    std::bind(&AtlasPlugin::SetJointCommands, this, std::placeholders::_1));

  this->subAtlasControlMode =
    this->rosNode->create_subscription<std_msgs::msg::String>(
    "atlas/control_mode", 100,
    std::bind(&AtlasPlugin::OnRobotMode, this, std::placeholders::_1));

  this->subASICommand =
    this->rosNode->create_subscription<atlas_msgs::msg::AtlasSimInterfaceCommand>(
    "atlas/atlas_sim_interface_command", rclcpp::QoS(1),
    std::bind(&AtlasPlugin::SetASICommand, this, std::placeholders::_1));

  ////////////////////////////////////////////////////////////////
  // ROS Services
  ////////////////////////////////////////////////////////////////
  this->InitFilter();
  this->atlasFiltersService =
    this->rosNode->create_service<atlas_msgs::srv::AtlasFilters>(
    "atlas/atlas_filters",
    std::bind(
      &AtlasPlugin::AtlasFilters, this, std::placeholders::_1,
      std::placeholders::_2));

  this->resetControlsService =
    this->rosNode->create_service<atlas_msgs::srv::ResetControls>(
    "atlas/reset_controls",
    std::bind(
      &AtlasPlugin::ResetControls, this, std::placeholders::_1,
      std::placeholders::_2));

  this->setJointDampingService =
    this->rosNode->create_service<atlas_msgs::srv::SetJointDamping>(
    "atlas/set_joint_damping",
    std::bind(
      &AtlasPlugin::SetJointDamping, this, std::placeholders::_1,
      std::placeholders::_2));

  this->getJointDampingService =
    this->rosNode->create_service<atlas_msgs::srv::GetJointDamping>(
    "atlas/get_joint_damping",
    std::bind(
      &AtlasPlugin::GetJointDamping, this, std::placeholders::_1,
      std::placeholders::_2));

  this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor->add_node(this->rosNode);
  this->rosSpinThread = std::thread(
    [this]()
    {
      this->executor->spin();
    });
}

//////////////////////////////////////////////////
void AtlasPlugin::UpdateStates(
  const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  if (_info.simTime <= this->lastControllerUpdateTime) {
    return;
  }

  const rclcpp::Time stamp(
    std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count());

  this->GetAndPublishRobotStates(_ecm, stamp);

  if (this->atlasCommand.desired_controller_period_ms != 0) {
    this->EnforceSynchronizationDelay(stamp);
  }

  if (this->startupStep == AtlasPlugin::NOMINAL) {
    this->UpdateAtlasSimInterface(_ecm, stamp);
  } else if (this->startupStep == AtlasPlugin::USER) {
    std::lock_guard<std::mutex> lock(this->asiMutex);
    this->asiState.desired_behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::USER;
    this->startupStep = AtlasPlugin::NOMINAL;
  } else if (this->startupStep == AtlasPlugin::FREEZE) {
    std::lock_guard<std::mutex> lock(this->asiMutex);
    this->asiState.desired_behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::FREEZE;
    this->startupStep = AtlasPlugin::USER;
  }

  {
    std::lock_guard<std::mutex> lock(this->controlMutex);
    this->CalculateControllerStatistics(stamp);
    this->UpdatePIDControl(
      _ecm, std::chrono::duration<double>(
        _info.simTime - this->lastControllerUpdateTime).count());
  }

  this->lastControllerUpdateTime = _info.simTime;

  this->PublishControllerStatistics(stamp);
}

//////////////////////////////////////////////////
void AtlasPlugin::GetAndPublishRobotStates(
  gz::sim::EntityComponentManager & _ecm, const rclcpp::Time & _stamp)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  this->GetIMUState(_ecm, _stamp);
  this->GetForceTorqueSensorState(_ecm);
  this->OnLContactUpdate(_ecm);
  this->OnRContactUpdate(_ecm);

  this->atlasState.header.stamp = _stamp;
  this->jointStates.header.stamp = _stamp;

  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    gz::sim::Joint joint(this->joints[i]);
    const double q = FirstOrZero(joint.Position(_ecm));
    const double qd = FirstOrZero(joint.Velocity(_ecm));

    this->atlasState.position[i] = q;
    this->atlasState.velocity[i] = qd;
    // effort[i] is cached from the previous UpdatePIDControl() cycle.
    this->atlasState.effort[i] = this->jointStates.effort[i];

    this->jointStates.position[i] = q;
    this->jointStates.velocity[i] = qd;
  }

  {
    std::lock_guard<std::mutex> filterLock(this->filterMutex);
    if (this->filterVelocity) {
      this->Filter(this->atlasState.velocity, this->jointStates.velocity);
    }
    if (this->filterPosition) {
      this->Filter(this->atlasState.position, this->jointStates.position);
    }
  }

  this->pubJointStates->publish(this->jointStates);
  this->pubAtlasState->publish(this->atlasState);
}

//////////////////////////////////////////////////
void AtlasPlugin::GetIMUState(
  gz::sim::EntityComponentManager & _ecm, const rclcpp::Time & _stamp)
{
  if (this->imuLinkEntity == gz::sim::kNullEntity) {
    return;
  }

  gz::sim::Link imuLink(this->imuLinkEntity);
  const auto worldPose = imuLink.WorldPose(_ecm);
  const auto angularVel = imuLink.WorldAngularVelocity(_ecm);
  const auto linearAcc = imuLink.WorldLinearAcceleration(_ecm);
  if (!worldPose || !angularVel || !linearAcc) {
    return;
  }

  const gz::math::Vector3d bodyAngularVel =
    worldPose->Rot().RotateVectorReverse(*angularVel);
  const gz::math::Vector3d bodyLinearAcc =
    worldPose->Rot().RotateVectorReverse(*linearAcc);

  this->atlasState.angular_velocity.x = bodyAngularVel.X();
  this->atlasState.angular_velocity.y = bodyAngularVel.Y();
  this->atlasState.angular_velocity.z = bodyAngularVel.Z();
  this->atlasState.linear_acceleration.x = bodyLinearAcc.X();
  this->atlasState.linear_acceleration.y = bodyLinearAcc.Y();
  this->atlasState.linear_acceleration.z = bodyLinearAcc.Z();
  this->atlasState.orientation.x = worldPose->Rot().X();
  this->atlasState.orientation.y = worldPose->Rot().Y();
  this->atlasState.orientation.z = worldPose->Rot().Z();
  this->atlasState.orientation.w = worldPose->Rot().W();

  sensor_msgs::msg::Imu imuMsg;
  imuMsg.header.frame_id = this->imuLinkName;
  imuMsg.header.stamp = _stamp;
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

//////////////////////////////////////////////////
void AtlasPlugin::GetForceTorqueSensorState(gz::sim::EntityComponentManager & _ecm)
{
  atlas_msgs::msg::ForceTorqueSensors ftMsg;

  const auto readWrench = [&_ecm](gz::sim::Entity _joint)
    -> std::optional<gz::msgs::Wrench>
    {
      if (_joint == gz::sim::kNullEntity) {
        return std::nullopt;
      }
      const auto wrenches = gz::sim::Joint(_joint).TransmittedWrench(_ecm);
      if (!wrenches || wrenches->empty()) {
        return std::nullopt;
      }
      return (*wrenches)[0];
    };

  if (const auto wrench = readWrench(this->lAnkleJointEntity)) {
    this->atlasState.l_foot.force.z = wrench->force().z();
    this->atlasState.l_foot.torque.x = wrench->torque().x();
    this->atlasState.l_foot.torque.y = wrench->torque().y();
    ftMsg.l_foot.force.z = wrench->force().z();
    ftMsg.l_foot.torque.x = wrench->torque().x();
    ftMsg.l_foot.torque.y = wrench->torque().y();
  }

  if (const auto wrench = readWrench(this->rAnkleJointEntity)) {
    this->atlasState.r_foot.force.z = wrench->force().z();
    this->atlasState.r_foot.torque.x = wrench->torque().x();
    this->atlasState.r_foot.torque.y = wrench->torque().y();
    ftMsg.r_foot.force.z = wrench->force().z();
    ftMsg.r_foot.torque.x = wrench->torque().x();
    ftMsg.r_foot.torque.y = wrench->torque().y();
  }

  if (const auto wrench = readWrench(this->lWristJointEntity)) {
    this->atlasState.l_hand.force.x = wrench->force().x();
    this->atlasState.l_hand.force.y = wrench->force().y();
    this->atlasState.l_hand.force.z = wrench->force().z();
    this->atlasState.l_hand.torque.x = wrench->torque().x();
    this->atlasState.l_hand.torque.y = wrench->torque().y();
    this->atlasState.l_hand.torque.z = wrench->torque().z();
    ftMsg.l_hand = this->atlasState.l_hand;
  }

  if (const auto wrench = readWrench(this->rWristJointEntity)) {
    this->atlasState.r_hand.force.x = wrench->force().x();
    this->atlasState.r_hand.force.y = wrench->force().y();
    this->atlasState.r_hand.force.z = wrench->force().z();
    this->atlasState.r_hand.torque.x = wrench->torque().x();
    this->atlasState.r_hand.torque.y = wrench->torque().y();
    this->atlasState.r_hand.torque.z = wrench->torque().z();
    ftMsg.r_hand = this->atlasState.r_hand;
  }

  this->pubForceTorqueSensors->publish(ftMsg);
}

//////////////////////////////////////////////////
void AtlasPlugin::OnLContactUpdate(gz::sim::EntityComponentManager & _ecm)
{
  if (!this->cheatsEnabled || this->lFootCollisions.empty()) {
    return;
  }
  gz::math::Vector3d fTotal;
  for (const gz::sim::Entity col : this->lFootCollisions) {
    const auto * data = _ecm.Component<gz::sim::components::ContactSensorData>(col);
    if (!data) {
      continue;
    }
    for (const auto & contact : data->Data().contact()) {
      for (int j = 0; j < contact.wrench_size(); ++j) {
        fTotal += gz::msgs::Convert(contact.wrench(j).body_1_wrench().force());
      }
    }
  }
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.frame_id = "l_foot";
  msg.wrench.force.x = fTotal.X();
  msg.wrench.force.y = fTotal.Y();
  msg.wrench.force.z = fTotal.Z();
  this->pubLFootContact->publish(msg);
}

//////////////////////////////////////////////////
void AtlasPlugin::OnRContactUpdate(gz::sim::EntityComponentManager & _ecm)
{
  if (!this->cheatsEnabled || this->rFootCollisions.empty()) {
    return;
  }
  gz::math::Vector3d fTotal;
  for (const gz::sim::Entity col : this->rFootCollisions) {
    const auto * data = _ecm.Component<gz::sim::components::ContactSensorData>(col);
    if (!data) {
      continue;
    }
    for (const auto & contact : data->Data().contact()) {
      for (int j = 0; j < contact.wrench_size(); ++j) {
        fTotal += gz::msgs::Convert(contact.wrench(j).body_1_wrench().force());
      }
    }
  }
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.frame_id = "r_foot";
  msg.wrench.force.x = fTotal.X();
  msg.wrench.force.y = fTotal.Y();
  msg.wrench.force.z = fTotal.Z();
  this->pubRFootContact->publish(msg);
}

//////////////////////////////////////////////////
void AtlasPlugin::UpdatePIDControl(gz::sim::EntityComponentManager & _ecm, double _dt)
{
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    gz::sim::Joint joint(this->joints[i]);

    double lower = -1e16;
    double upper = 1e16;
    const auto axes = joint.Axis(_ecm);
    if (axes && !axes->empty()) {
      lower = (*axes)[0].Lower();
      upper = (*axes)[0].Upper();
    }
    const double positionTarget =
      std::clamp(this->atlasCommand.position[i], lower, upper);

    const double qP = positionTarget - this->atlasState.position[i];
    if (_dt != 0.0) {
      this->errorTerms[i].dQpDt = (qP - this->errorTerms[i].qP) / _dt;
    }
    this->errorTerms[i].qP = qP;

    // Take advantage of cfm damping by passing kp_velocity through to the
    // intrinsic joint damping coefficient -- simulating infinite-bandwidth
    // kp_velocity (see the class-level design note: this is now a cached
    // value only, gz-sim has no runtime joint-damping mutation API).
    const double jointDampingCoef = std::clamp(
      static_cast<double>(this->atlasState.kp_velocity[i]),
      this->jointDampingModel[i], this->jointDampingMax[i]);
    if (this->lastJointCFMDamping[i] != jointDampingCoef) {
      this->lastJointCFMDamping[i] = jointDampingCoef;
    }

    double kpVelocityDampingEffort = 0.0;
    const double kpVelocityDampingCoef = jointDampingCoef - this->jointDampingModel[i];
    if (kpVelocityDampingCoef > 0.0) {
      kpVelocityDampingEffort = kpVelocityDampingCoef * this->atlasState.velocity[i];
    }

    this->errorTerms[i].kIqI = std::clamp(
      this->errorTerms[i].kIqI +
      _dt * this->atlasState.ki_position[i] * this->errorTerms[i].qP,
      static_cast<double>(this->atlasState.i_effort_min[i]),
      static_cast<double>(this->atlasState.i_effort_max[i]));

    const double kEffort =
      static_cast<double>(this->atlasState.k_effort[i]) / 255.0;

    const double forceUnclamped =
      kEffort * (
      this->atlasState.kp_position[i] * this->errorTerms[i].qP +
      this->errorTerms[i].kIqI +
      this->atlasState.kd_position[i] * this->errorTerms[i].dQpDt +
      jointDampingCoef * this->atlasCommand.velocity[i] +
      this->atlasCommand.effort[i]) +
      (1.0 - kEffort) * this->asiState.f_out[i];

    double forceClamped = std::clamp(
      forceUnclamped, -this->effortLimit[i] + kpVelocityDampingEffort,
      this->effortLimit[i] + kpVelocityDampingEffort);

    joint.SetForce(_ecm, {forceClamped});

    this->atlasState.effort[i] = forceClamped;
    this->jointStates.effort[i] = forceClamped;
  }
}

//////////////////////////////////////////////////
void AtlasPlugin::UpdateAtlasSimInterface(
  gz::sim::EntityComponentManager & _ecm, const rclcpp::Time & _stamp)
{
  // Snapshot k_effort under controlMutex *before* taking asiMutex below --
  // never hold both at once (some ROS callbacks lock them in the opposite
  // order), so a short-lived, non-nested lock here avoids a lock-ordering
  // deadlock entirely rather than relying on a project-wide ordering rule.
  std::vector<uint8_t> kEffortSnapshot;
  {
    std::lock_guard<std::mutex> controlLock(this->controlMutex);
    kEffortSnapshot = this->atlasState.k_effort;
  }

  std::lock_guard<std::mutex> lock(this->asiMutex);

  this->asiState.header.stamp = _stamp;

  // Inlined equivalent of the (Apache-2.0, open-source) AtlasSimInterface
  // shim's process_control_input() -- see the class-level design note.
  // The shim hardcodes dt = 0.001 rather than computing it from
  // timestamps (its own TODO says so); preserved exactly.
  static const double kShimDt = 0.001;
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    gz::sim::Joint joint(this->joints[i]);
    const double q = FirstOrZero(joint.Position(_ecm));

    const double qP = this->asiCommand.position[i] - q;
    if (kShimDt > 0.0) {
      this->asiErrorTerms[i].dQpDt = (qP - this->asiErrorTerms[i].qP) / kShimDt;
    }
    this->asiErrorTerms[i].qP = qP;

    this->asiErrorTerms[i].kIqI +=
      kShimDt * this->asiCommand.ki_position[i] * this->asiErrorTerms[i].qP;

    this->asiState.f_out[i] =
      this->asiCommand.kp_position[i] * this->asiErrorTerms[i].qP +
      this->asiErrorTerms[i].kIqI +
      this->asiCommand.kp_velocity[i] * this->asiCommand.velocity[i] +
      this->asiCommand.effort[i];
  }

  // The shim's get_desired_behavior()/get_current_behavior() never
  // actually write back their output string (see the class-level design
  // note) -- rather than faithfully reproducing "current_behavior always
  // reports NONE", just mirror desired_behavior directly.
  this->asiState.current_behavior = this->asiState.desired_behavior;

  this->asiState.k_effort = kEffortSnapshot;

  this->pubASIState->publish(this->asiState);
}

//////////////////////////////////////////////////
void AtlasPlugin::CalculateControllerStatistics(const rclcpp::Time & _stamp)
{
  this->atlasCommandAge = _stamp.seconds() - this->atlasCommandStamp.seconds();

  const double weightedAge =
    this->atlasCommandAge / this->atlasCommandAgeBuffer.size();

  const double delta = this->atlasCommandAge - this->atlasCommandAgeMean;

  this->atlasCommandAgeMean += weightedAge;
  this->atlasCommandAgeMean -=
    this->atlasCommandAgeBuffer[this->atlasCommandAgeBufferIndex];

  const double delta2 = delta * (this->atlasCommandAge - this->atlasCommandAgeMean);
  this->atlasCommandAgeVariance += delta2;
  this->atlasCommandAgeVariance -=
    this->atlasCommandAgeDelta2Buffer[this->atlasCommandAgeBufferIndex];

  this->atlasCommandAgeBuffer[this->atlasCommandAgeBufferIndex] = weightedAge;
  this->atlasCommandAgeDelta2Buffer[this->atlasCommandAgeBufferIndex] = delta2;

  this->atlasCommandAgeBufferIndex =
    (this->atlasCommandAgeBufferIndex + 1) % this->atlasCommandAgeBuffer.size();
}

//////////////////////////////////////////////////
void AtlasPlugin::PublishControllerStatistics(const rclcpp::Time & _stamp)
{
  // The original tracked publisher connect/disconnect counts explicitly;
  // rclcpp's own get_subscription_count() is a direct, simpler substitute
  // already used elsewhere in this migration (e.g. SandiaHandPlugin's
  // tactile publisher).
  if (this->pubControllerStatistics->get_subscription_count() == 0) {
    return;
  }
  if (_stamp.seconds() - this->lastControllerStatisticsTime <
    1.0 / this->statsUpdateRate)
  {
    return;
  }

  atlas_msgs::msg::ControllerStatistics msg;
  msg.header.stamp = _stamp;
  {
    std::lock_guard<std::mutex> lock(this->controlMutex);
    msg.command_age = this->atlasCommandAge;
    msg.command_age_mean = this->atlasCommandAgeMean;
    msg.command_age_variance =
      this->atlasCommandAgeVariance / (this->atlasCommandAgeBuffer.size() - 1);
    msg.command_age_window_size = this->atlasCommandAgeBufferDuration;
  }
  this->pubControllerStatistics->publish(msg);
  this->lastControllerStatisticsTime = _stamp.seconds();
}

//////////////////////////////////////////////////
void AtlasPlugin::EnforceSynchronizationDelay(const rclcpp::Time & _stamp)
{
  const auto windowSize = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(this->delayWindowSize));

  const auto curWallTime = std::chrono::steady_clock::now();
  if (curWallTime >= this->delayWindowStart + windowSize) {
    this->delayWindowStart = curWallTime;
    this->delayInWindow = 0.0;
  }

  std::chrono::duration<double> delayInStepSum(0.0);
  if (this->delayInWindow < this->delayMaxPerWindow) {
    while (delayInStepSum.count() < this->delayMaxPerStep &&
      this->delayInWindow < this->delayMaxPerWindow)
    {
      std::unique_lock<std::mutex> lock(this->controlMutex);
      const double age = _stamp.seconds() - this->atlasCommandStamp.seconds();

      if (age <= 0.001 * this->atlasCommand.desired_controller_period_ms) {
        break;
      }

      const auto waitDuration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(
          std::min(
            this->delayMaxPerStep - delayInStepSum.count(),
            this->delayMaxPerWindow - this->delayInWindow)));

      const auto before = std::chrono::steady_clock::now();
      this->delayCondition.wait_for(lock, waitDuration);
      const auto delayTime = std::chrono::steady_clock::now() - before;

      delayInStepSum += delayTime;
      this->delayInWindow += std::chrono::duration<double>(delayTime).count();
    }
  }

  this->delayStatistics.delay_in_step = delayInStepSum.count();
  this->delayStatistics.delay_in_window = this->delayInWindow;
  this->delayStatistics.delay_window_remain = std::chrono::duration<double>(
    (this->delayWindowStart + windowSize) - curWallTime).count();
  this->pubDelayStatistics->publish(this->delayStatistics);
}

//////////////////////////////////////////////////
void AtlasPlugin::InitFilter()
{
  // Filter design from Matlab: [b,a] = butter(1,0.025) // 12.5Hz
  this->filCoefA[0] = 1.0;
  this->filCoefA[1] = -0.924390491658207;
  this->filCoefB[0] = 0.037804754170897;
  this->filCoefB[1] = 0.037804754170897;

  this->unfilteredIn.assign(this->joints.size(), std::vector<double>(FIL_N_STEPS, 0.0));
  this->unfilteredOut.assign(this->joints.size(), std::vector<double>(FIL_N_STEPS, 0.0));
}

//////////////////////////////////////////////////
void AtlasPlugin::Filter(std::vector<float> & _aState, std::vector<double> & _jState)
{
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    for (int j = FIL_N_STEPS - 2; j >= 0; --j) {
      this->unfilteredIn[i][j + 1] = this->unfilteredIn[i][j];
      this->unfilteredOut[i][j + 1] = this->unfilteredOut[i][j];
    }
    this->unfilteredIn[i][0] = _aState[i];

    double tmp = 0.0;
    for (unsigned int j = 0; j < FIL_N_STEPS; ++j) {
      tmp += this->filCoefB[j] * this->unfilteredIn[i][j];
    }
    for (unsigned int j = 1; j < FIL_N_STEPS; ++j) {
      tmp -= this->filCoefA[j] * this->unfilteredOut[i][j];
    }
    this->unfilteredOut[i][0] = tmp;
    _aState[i] = tmp;
    _jState[i] = tmp;
  }
}

//////////////////////////////////////////////////
void AtlasPlugin::LoadPIDGainsFromParameter()
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    const std::string prefix = "atlas_controller.gains." + this->jointNames[i] + ".";
    const double p = this->rosNode->declare_parameter(prefix + "p", 0.0);
    const double iGain = this->rosNode->declare_parameter(prefix + "i", 0.0);
    const double d = this->rosNode->declare_parameter(prefix + "d", 0.0);
    const double iClamp = this->rosNode->declare_parameter(prefix + "i_clamp", 0.0);

    this->atlasState.kp_position[i] = p;
    this->atlasState.ki_position[i] = iGain;
    this->atlasState.kd_position[i] = d;
    this->atlasState.i_effort_min[i] = -iClamp;
    this->atlasState.i_effort_max[i] = iClamp;
    // Default k_effort is full PID control.
    this->atlasState.k_effort[i] = 255;
  }
}

//////////////////////////////////////////////////
void AtlasPlugin::ZeroAtlasCommand()
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  for (unsigned int i = 0; i < this->jointNames.size(); ++i) {
    this->atlasCommand.position[i] = 0.0;
    this->atlasCommand.velocity[i] = 0.0;
    this->atlasCommand.effort[i] = 0.0;
    this->atlasState.kp_position[i] = 0.0;
    this->atlasState.ki_position[i] = 0.0;
    this->atlasState.kd_position[i] = 0.0;
    this->atlasState.kp_velocity[i] = 0.0;
    this->atlasState.i_effort_min[i] = 0.0;
    this->atlasState.i_effort_max[i] = 0.0;
    this->atlasState.k_effort[i] = 0;
  }
  this->atlasCommand.desired_controller_period_ms = 0;
}

//////////////////////////////////////////////////
void AtlasPlugin::Tic(const std_msgs::msg::String::SharedPtr)
{
  this->delayCondition.notify_one();
}

//////////////////////////////////////////////////
void AtlasPlugin::SetAtlasCommand(const atlas_msgs::msg::AtlasCommand::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  this->atlasCommandStamp = rclcpp::Time(_msg->header.stamp);

  if (_msg->position.size() == this->atlasCommand.position.size()) {
    std::copy(
      _msg->position.begin(), _msg->position.end(), this->atlasCommand.position.begin());
  }
  if (_msg->velocity.size() == this->atlasCommand.velocity.size()) {
    std::copy(
      _msg->velocity.begin(), _msg->velocity.end(), this->atlasCommand.velocity.begin());
  }
  if (_msg->effort.size() == this->atlasCommand.effort.size()) {
    std::copy(_msg->effort.begin(), _msg->effort.end(), this->atlasCommand.effort.begin());
  }
  if (_msg->kp_position.size() == this->atlasState.kp_position.size()) {
    std::copy(
      _msg->kp_position.begin(), _msg->kp_position.end(),
      this->atlasState.kp_position.begin());
  }
  if (_msg->ki_position.size() == this->atlasState.ki_position.size()) {
    std::copy(
      _msg->ki_position.begin(), _msg->ki_position.end(),
      this->atlasState.ki_position.begin());
  }
  if (_msg->kd_position.size() == this->atlasState.kd_position.size()) {
    std::copy(
      _msg->kd_position.begin(), _msg->kd_position.end(),
      this->atlasState.kd_position.begin());
  }
  if (_msg->kp_velocity.size() == this->atlasState.kp_velocity.size()) {
    std::copy(
      _msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->atlasState.kp_velocity.begin());
    // Joint damping mutation has no gz-sim equivalent (see the class-level
    // design note) -- kp_velocity still reaches the control law via
    // UpdatePIDControl()'s jointDampingCoef term regardless.
  }
  if (_msg->i_effort_min.size() == this->atlasState.i_effort_min.size()) {
    std::copy(
      _msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->atlasState.i_effort_min.begin());
  }
  if (_msg->i_effort_max.size() == this->atlasState.i_effort_max.size()) {
    std::copy(
      _msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->atlasState.i_effort_max.begin());
  }
  if (_msg->k_effort.size() == this->atlasState.k_effort.size()) {
    std::copy(
      _msg->k_effort.begin(), _msg->k_effort.end(), this->atlasState.k_effort.begin());
  }

  this->atlasCommand.desired_controller_period_ms = _msg->desired_controller_period_ms;

  this->delayCondition.notify_one();
}

//////////////////////////////////////////////////
void AtlasPlugin::SetJointCommands(const osrf_msgs::msg::JointCommands::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  this->atlasCommandStamp = rclcpp::Time(_msg->header.stamp);

  if (_msg->position.size() == this->atlasCommand.position.size()) {
    std::copy(
      _msg->position.begin(), _msg->position.end(), this->atlasCommand.position.begin());
  }
  if (_msg->velocity.size() == this->atlasCommand.velocity.size()) {
    std::copy(
      _msg->velocity.begin(), _msg->velocity.end(), this->atlasCommand.velocity.begin());
  }
  if (_msg->effort.size() == this->atlasCommand.effort.size()) {
    std::copy(_msg->effort.begin(), _msg->effort.end(), this->atlasCommand.effort.begin());
  }
  if (_msg->kp_position.size() == this->atlasState.kp_position.size()) {
    std::copy(
      _msg->kp_position.begin(), _msg->kp_position.end(),
      this->atlasState.kp_position.begin());
  }
  if (_msg->ki_position.size() == this->atlasState.ki_position.size()) {
    std::copy(
      _msg->ki_position.begin(), _msg->ki_position.end(),
      this->atlasState.ki_position.begin());
  }
  if (_msg->kd_position.size() == this->atlasState.kd_position.size()) {
    std::copy(
      _msg->kd_position.begin(), _msg->kd_position.end(),
      this->atlasState.kd_position.begin());
  }
  if (_msg->kp_velocity.size() == this->atlasState.kp_velocity.size()) {
    std::copy(
      _msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->atlasState.kp_velocity.begin());
  }
  if (_msg->i_effort_min.size() == this->atlasState.i_effort_min.size()) {
    std::copy(
      _msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->atlasState.i_effort_min.begin());
  }
  if (_msg->i_effort_max.size() == this->atlasState.i_effort_max.size()) {
    std::copy(
      _msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->atlasState.i_effort_max.begin());
  }
}

//////////////////////////////////////////////////
void AtlasPlugin::SetExperimentalDampingPID(const atlas_msgs::msg::Test::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  // Joint damping mutation has no gz-sim equivalent (see the class-level
  // design note) -- the original called Joint::SetDamping() per-joint here.

  if (_msg->kp_position.size() == this->atlasState.kp_position.size()) {
    std::copy(
      _msg->kp_position.begin(), _msg->kp_position.end(),
      this->atlasState.kp_position.begin());
  }
  if (_msg->ki_position.size() == this->atlasState.ki_position.size()) {
    std::copy(
      _msg->ki_position.begin(), _msg->ki_position.end(),
      this->atlasState.ki_position.begin());
  }
  if (_msg->kd_position.size() == this->atlasState.kd_position.size()) {
    std::copy(
      _msg->kd_position.begin(), _msg->kd_position.end(),
      this->atlasState.kd_position.begin());
  }
  if (_msg->kp_velocity.size() == this->atlasState.kp_velocity.size()) {
    std::copy(
      _msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->atlasState.kp_velocity.begin());
  }
  if (_msg->i_effort_min.size() == this->atlasState.i_effort_min.size()) {
    std::copy(
      _msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->atlasState.i_effort_min.begin());
  }
  if (_msg->i_effort_max.size() == this->atlasState.i_effort_max.size()) {
    std::copy(
      _msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->atlasState.i_effort_max.begin());
  }
  if (_msg->k_effort.size() == this->atlasState.k_effort.size()) {
    std::copy(
      _msg->k_effort.begin(), _msg->k_effort.end(), this->atlasState.k_effort.begin());
  }
}

//////////////////////////////////////////////////
void AtlasPlugin::SetASICommand(
  const atlas_msgs::msg::AtlasSimInterfaceCommand::SharedPtr _msg)
{
  {
    std::lock_guard<std::mutex> lock(this->controlMutex);
    if (_msg->k_effort.size() == this->atlasState.k_effort.size()) {
      std::copy(
        _msg->k_effort.begin(), _msg->k_effort.end(), this->atlasState.k_effort.begin());
    }
  }

  std::lock_guard<std::mutex> lock(this->asiMutex);

  this->asiState.desired_behavior = _msg->behavior;

  if (_msg->position.size() == this->asiCommand.position.size()) {
    std::copy(
      _msg->position.begin(), _msg->position.end(), this->asiCommand.position.begin());
  }
  if (_msg->velocity.size() == this->asiCommand.velocity.size()) {
    std::copy(
      _msg->velocity.begin(), _msg->velocity.end(), this->asiCommand.velocity.begin());
  }
  if (_msg->effort.size() == this->asiCommand.effort.size()) {
    std::copy(_msg->effort.begin(), _msg->effort.end(), this->asiCommand.effort.begin());
  }
  if (_msg->kp_position.size() == this->asiCommand.kp_position.size()) {
    std::copy(
      _msg->kp_position.begin(), _msg->kp_position.end(),
      this->asiCommand.kp_position.begin());
  }
  if (_msg->ki_position.size() == this->asiCommand.ki_position.size()) {
    std::copy(
      _msg->ki_position.begin(), _msg->ki_position.end(),
      this->asiCommand.ki_position.begin());
  }
  if (_msg->kp_velocity.size() == this->asiCommand.kp_velocity.size()) {
    std::copy(
      _msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->asiCommand.kp_velocity.begin());
  }
}

//////////////////////////////////////////////////
void AtlasPlugin::OnRobotMode(const std_msgs::msg::String::SharedPtr _mode)
{
  std::lock_guard<std::mutex> lock(this->asiMutex);

  if (_mode->data == "Freeze" || _mode->data == "StandPrep" ||
    _mode->data == "Stand" || _mode->data == "Walk" || _mode->data == "Manipulate")
  {
    RCLCPP_WARN(
      this->rosNode->get_logger(),
      "controlling AtlasSimInterface library over atlas/control_mode is "
      "deprecated, please switch to using ROS topic "
      "atlas/atlas_sim_interface_command and look for feedback on "
      "atlas/atlas_sim_interface_state.");

    if (_mode->data == "Freeze") {
      this->asiState.desired_behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::FREEZE;
    } else if (_mode->data == "StandPrep") {
      this->asiState.desired_behavior =
        atlas_msgs::msg::AtlasSimInterfaceCommand::STAND_PREP;
    } else if (_mode->data == "Stand") {
      this->asiState.desired_behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::STAND;
    } else if (_mode->data == "Walk") {
      this->asiState.desired_behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::WALK;
    } else if (_mode->data == "Manipulate") {
      this->asiState.desired_behavior =
        atlas_msgs::msg::AtlasSimInterfaceCommand::MANIPULATE;
    }
  } else if (_mode->data == "User") {
    this->LoadPIDGainsFromParameter();
    this->asiState.desired_behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::USER;
    for (unsigned int i = 0; i < this->jointNames.size(); ++i) {
      this->asiState.f_out[i] = 0.0;
    }
  } else if (_mode->data == "ragdoll") {
    this->ZeroAtlasCommand();
    this->asiState.desired_behavior = atlas_msgs::msg::AtlasSimInterfaceCommand::USER;
    for (unsigned int i = 0; i < this->jointNames.size(); ++i) {
      this->asiState.f_out[i] = 0.0;
    }
  } else {
    RCLCPP_WARN(
      this->rosNode->get_logger(), "Unknown robot mode [%s]", _mode->data.c_str());
  }
}

//////////////////////////////////////////////////
void AtlasPlugin::AtlasFilters(
  const std::shared_ptr<atlas_msgs::srv::AtlasFilters::Request> _req,
  std::shared_ptr<atlas_msgs::srv::AtlasFilters::Response> _res)
{
  std::lock_guard<std::mutex> lock(this->filterMutex);

  _res->success = true;
  this->filterVelocity = _req->filter_velocity;
  this->filterPosition = _req->filter_position;

  std::string statusMessage;
  if (_req->coef_a.size() == 2) {
    this->filCoefA[0] = _req->coef_a[0];
    this->filCoefA[1] = _req->coef_a[1];
  } else if (!_req->coef_a.empty()) {
    _res->success = false;
    statusMessage += "AtlasFilters: coef_a has size [" +
      std::to_string(_req->coef_a.size()) + "], only 0 or 2 is allowed.\n";
  }

  if (_req->coef_b.size() == 2) {
    this->filCoefB[0] = _req->coef_b[0];
    this->filCoefB[1] = _req->coef_b[1];
  } else if (!_req->coef_b.empty()) {
    _res->success = false;
    statusMessage += "AtlasFilters: coef_b has size [" +
      std::to_string(_req->coef_b.size()) + "], only 0 or 2 is allowed.\n";
  }

  if (!statusMessage.empty()) {
    RCLCPP_WARN(this->rosNode->get_logger(), "%s", statusMessage.c_str());
  }
  _res->status_message = statusMessage;
}

//////////////////////////////////////////////////
void AtlasPlugin::ResetControls(
  const std::shared_ptr<atlas_msgs::srv::ResetControls::Request> _req,
  std::shared_ptr<atlas_msgs::srv::ResetControls::Response> _res)
{
  _res->success = true;
  _res->status_message = "success";

  if (_req->reset_bdi_controller) {
    std::lock_guard<std::mutex> lock(this->asiMutex);
    this->asiErrorTerms.assign(this->joints.size(), ErrorTerms());
  }

  if (_req->reset_pid_controller) {
    std::lock_guard<std::mutex> lock(this->controlMutex);
    this->errorTerms.assign(this->joints.size(), ErrorTerms());
  }

  if (_req->reload_pid_from_ros) {
    this->LoadPIDGainsFromParameter();
  } else {
    auto msg = std::make_shared<atlas_msgs::msg::AtlasCommand>(_req->atlas_command);
    this->SetAtlasCommand(msg);
  }
}

//////////////////////////////////////////////////
void AtlasPlugin::SetJointDamping(
  const std::shared_ptr<atlas_msgs::srv::SetJointDamping::Request> _req,
  std::shared_ptr<atlas_msgs::srv::SetJointDamping::Response> _res)
{
  _res->success = true;
  std::string statusMessage;
  {
    std::lock_guard<std::mutex> lock(this->controlMutex);
    for (unsigned int i = 0; i < this->joints.size(); ++i) {
      const double d = std::clamp(
        _req->damping_coefficients[i], this->jointDampingMin[i],
        this->jointDampingMax[i]);

      // Joint damping mutation has no gz-sim equivalent (see the
      // class-level design note) -- only the cached value is updated.
      this->jointDampingModel[i] = d;
      this->lastJointCFMDamping[i] = d;

      if (d != _req->damping_coefficients[i]) {
        statusMessage += "requested joint damping for joint [" +
          this->jointNames[i] + "] of [" +
          std::to_string(_req->damping_coefficients[i]) + "] is truncated to [" +
          std::to_string(d) + "].\n";
        _res->success = false;
      }
    }
  }
  if (!_res->success) {
    RCLCPP_WARN(this->rosNode->get_logger(), "%s", statusMessage.c_str());
  } else {
    statusMessage += "You have successfully changed model damping parameters.";
    RCLCPP_INFO(this->rosNode->get_logger(), "%s", statusMessage.c_str());
  }
  _res->status_message = statusMessage;
}

//////////////////////////////////////////////////
void AtlasPlugin::GetJointDamping(
  const std::shared_ptr<atlas_msgs::srv::GetJointDamping::Request>,
  std::shared_ptr<atlas_msgs::srv::GetJointDamping::Response> _res)
{
  _res->success = true;
  _res->status_message = "success";

  std::lock_guard<std::mutex> lock(this->controlMutex);
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    _res->damping_coefficients[i] = this->jointDampingModel[i];
    _res->damping_coefficients_max[i] = this->jointDampingMax[i];
    _res->damping_coefficients_min[i] = this->jointDampingMin[i];
  }
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(AtlasPlugin,
              gz::sim::System,
              AtlasPlugin::ISystemConfigure,
              AtlasPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AtlasPlugin, "drcsim_gazebo_ros_plugins::AtlasPlugin")
