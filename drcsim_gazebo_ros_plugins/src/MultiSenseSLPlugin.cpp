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
#include "drcsim_gazebo_ros_plugins/MultiSenseSLPlugin.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Name.hh>

#include "drcsim_gazebo_ros_plugins/RosNodeOptions.hpp"

using drcsim_gazebo_ros_plugins::MultiSenseSLPlugin;

//////////////////////////////////////////////////
MultiSenseSLPlugin::MultiSenseSLPlugin()
{
}

//////////////////////////////////////////////////
MultiSenseSLPlugin::~MultiSenseSLPlugin()
{
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->rosSpinThread.joinable()) {
    this->rosSpinThread.join();
  }
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    gzerr << "MultiSenseSLPlugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->sdfConfig = _sdf;
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized && this->sdfConfig) {
    // Deferred from Configure(): a model's child link/joint entities are
    // not guaranteed to exist yet when Configure() runs.
    this->Load(_ecm);
    this->initialized = true;
  }

  if (this->validConfig) {
    this->UpdateStates(_info, _ecm);
  }
}

//////////////////////////////////////////////////
double MultiSenseSLPlugin::FirstOrZero(
  const std::optional<std::vector<double>> & _values)
{
  if (_values && !_values->empty()) {
    return (*_values)[0];
  }
  return 0.0;
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::Load(gz::sim::EntityComponentManager & _ecm)
{
  // Get imu link.
  this->imuLinkEntity = this->model.LinkByName(_ecm, this->imuLinkName);
  if (this->imuLinkEntity == gz::sim::kNullEntity) {
    gzerr << this->imuLinkName << " not found" << std::endl;
  } else {
    gz::sim::Link imuLink(this->imuLinkEntity);
    imuLink.EnableVelocityChecks(_ecm);
    imuLink.EnableAccelerationChecks(_ecm);
  }

  const gz::sim::Entity spindleLinkEntity =
    this->model.LinkByName(_ecm, "hokuyo_link");
  if (spindleLinkEntity == gz::sim::kNullEntity) {
    gzerr << "spindle link not found, plugin will stop loading" << std::endl;
    return;
  }

  this->spindleJointEntity = this->model.JointByName(_ecm, "hokuyo_joint");
  if (this->spindleJointEntity == gz::sim::kNullEntity) {
    gzerr << "spindle joint not found, plugin will stop loading" << std::endl;
    return;
  }
  gz::sim::Joint spindleJoint(this->spindleJointEntity);
  spindleJoint.EnablePositionCheck(_ecm);
  spindleJoint.EnableVelocityCheck(_ecm);

  // Publish joint states for spindle joint.
  this->jointStates.name = {"hokuyo_joint"};
  this->jointStates.position.assign(1, 0.0);
  this->jointStates.velocity.assign(1, 0.0);
  this->jointStates.effort.assign(1, 0.0);

  // The stereo pair is two camera sensors in gz-sim (Classic's single
  // "multicamera" sensor type does not exist here; see multisense_sl_v4.urdf).
  for (const char * camera : {"left_camera_sensor", "right_camera_sensor"}) {
    if (_ecm.EntityByComponents(gz::sim::components::Name(camera)) == gz::sim::kNullEntity) {
      gzerr << "stereo camera sensor [" << camera << "] not found" << std::endl;
    }
  }
  if (_ecm.EntityByComponents(
      gz::sim::components::Name("head_hokuyo_sensor")) == gz::sim::kNullEntity)
  {
    gzerr << "laser sensor not found" << std::endl;
  }

  /// \todo: hardcoded for now, make them into plugin parameters
  this->spindlePID.Init(0.03, 0.30, 0.00001, 1.0, -1.0, 10.0, -10.0);

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->rosNode = std::make_shared<rclcpp::Node>(
    "multisense_sl_plugin", drcsim_gazebo_ros_plugins::RosNodeOptionsFromEnv());

  const int atlasVersion = this->rosNode->declare_parameter("atlas_version", 5);
  if (atlasVersion == 1) {
    this->rosNamespace = "/multisense_sl";
  } else if (atlasVersion >= 3) {
    this->rosNamespace = "/multisense";
  } else {
    RCLCPP_WARN(
      this->rosNode->get_logger(),
      "atlas_version [%d] not one of 1, 3, 4, or 5; assuming atlas v1.",
      atlasVersion);
    this->rosNamespace = "/multisense_sl";
  }

  this->pubJointStates =
    this->rosNode->create_publisher<sensor_msgs::msg::JointState>(
    this->rosNamespace + "/joint_states", 10);
  this->pubImu = this->rosNode->create_publisher<sensor_msgs::msg::Imu>(
    this->rosNamespace + "/imu", 10);

  // Spindle speed to start with (rad/s), clamped like set_spindle_speed.
  // 0 (the original's behaviour) leaves the lidar still -- a fixed 2D scan
  // plane; atlas.launch.py spins it so RViz builds a 3D point cloud.
  {
    auto initial = std::make_shared<std_msgs::msg::Float64>();
    initial->data = this->rosNode->declare_parameter("spindle_speed", 0.0);
    this->SetSpindleSpeed(initial);
  }

  this->subSetSpindleSpeed =
    this->rosNode->create_subscription<std_msgs::msg::Float64>(
    this->rosNamespace + "/set_spindle_speed", 100,
    std::bind(&MultiSenseSLPlugin::SetSpindleSpeed, this, std::placeholders::_1));

  // Deprecated alias, kept for backward compatibility (originally added for
  // upstream issue 272).
  this->subSetMultiCameraFrameRateOld =
    this->rosNode->create_subscription<std_msgs::msg::Float64>(
    this->rosNamespace + "/fps", 100,
    std::bind(
      &MultiSenseSLPlugin::SetMultiCameraFrameRateOld, this,
      std::placeholders::_1));

  this->subSetMultiCameraFrameRate =
    this->rosNode->create_subscription<std_msgs::msg::Float64>(
    this->rosNamespace + "/set_fps", 100,
    std::bind(
      &MultiSenseSLPlugin::SetMultiCameraFrameRate, this, std::placeholders::_1));

  // The original left this subscription commented out ("currently this
  // causes simulation to crash"); that crash was specific to old
  // Gazebo/ROS 1, and this callback only ever updates cached values now
  // (see the class-level design note on dropped camera-sensor mutation),
  // so it's wired up here.
  this->subSetMultiCameraResolution =
    this->rosNode->create_subscription<std_msgs::msg::Int32>(
    this->rosNamespace + "/set_camera_resolution_mode", 100,
    std::bind(
      &MultiSenseSLPlugin::SetMultiCameraResolution, this, std::placeholders::_1));

  // SetSpindleState/SetMultiCameraExposureTime/SetMultiCameraGain are kept
  // as methods (below) but, matching the original exactly, are never bound
  // to a subscription -- the original marked this whole group "not
  // implemented, not supported" and never advertised them either.

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
void MultiSenseSLPlugin::UpdateStates(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  if (_info.simTime <= this->lastControllerUpdateTime) {
    return;
  }

  const rclcpp::Time stamp(
    std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count());

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
      imuMsg.header.stamp = stamp;

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

  const double dt = std::chrono::duration<double>(
    _info.simTime - this->lastControllerUpdateTime).count();

  gz::sim::Joint spindleJoint(this->spindleJointEntity);
  this->jointStates.header.stamp = stamp;
  this->jointStates.position[0] = FirstOrZero(spindleJoint.Position(_ecm));
  this->jointStates.velocity[0] = FirstOrZero(spindleJoint.Velocity(_ecm));
  this->jointStates.effort[0] = 0.0;

  if (this->spindleOn) {
    // PID control (velocity) spindle.
    const double spindleError =
      FirstOrZero(spindleJoint.Velocity(_ecm)) - this->spindleSpeed;
    const double spindleCmd =
      this->spindlePID.Update(spindleError, std::chrono::duration<double>(dt));
    spindleJoint.SetForce(_ecm, {spindleCmd});
    this->jointStates.effort[0] = spindleCmd;
  } else {
    this->spindlePID.Reset();
  }
  this->pubJointStates->publish(this->jointStates);

  this->lastControllerUpdateTime = _info.simTime;
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::SetSpindleSpeed(
  const std_msgs::msg::Float64::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->spindleSpeed = _msg->data;
  const double maxRad = this->spindleMaxRPM * 2.0 * M_PI / 60.0;
  const double minRad = this->spindleMinRPM * 2.0 * M_PI / 60.0;
  if (this->spindleSpeed > maxRad) {
    this->spindleSpeed = maxRad;
  } else if (this->spindleSpeed < minRad) {
    this->spindleSpeed = minRad;
  }
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::SetSpindleState(
  const std_msgs::msg::Bool::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->spindleOn = _msg->data;
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::SetMultiCameraFrameRateOld(
  const std_msgs::msg::Float64::SharedPtr _msg)
{
  RCLCPP_WARN(
    this->rosNode->get_logger(),
    "Frame rate was modified but the topic ~/fps has been replaced by "
    "~/set_fps.");
  this->SetMultiCameraFrameRate(_msg);
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::SetMultiCameraFrameRate(
  const std_msgs::msg::Float64::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  // Limit frame rate to what is capable.
  this->multiCameraFrameRate = _msg->data;

  // FIXME: Hardcoded lower limit on all resolutions.
  if (this->multiCameraFrameRate < 1.0) {
    RCLCPP_INFO(
      this->rosNode->get_logger(),
      "Camera rate cannot be below 1Hz at any resolution");
    this->multiCameraFrameRate = 1.0;
  }

  // FIXME: Hardcoded upper limit. Need to switch rates between modes.
  double maxRate = 30.0;
  switch (this->imagerMode) {
    case 0:
      maxRate = 15.0;
      break;
    case 1:
      maxRate = 30.0;
      break;
    case 2:
      maxRate = 60.0;
      break;
    case 3:
      maxRate = 70.0;
      break;
    default:
      RCLCPP_ERROR(
        this->rosNode->get_logger(),
        "MultiSense SL internal state error (%d)", this->imagerMode);
      break;
  }
  if (this->multiCameraFrameRate > maxRate) {
    RCLCPP_INFO(
      this->rosNode->get_logger(),
      "Camera rate cannot be above %.0fHz at this resolution", maxRate);
    this->multiCameraFrameRate = maxRate;
  }

  // No live camera-sensor reconfiguration API in gz-sim reachable from a
  // System plugin (see the class-level design note) -- multiCameraFrameRate
  // is now a cached value only.
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::SetMultiCameraResolution(
  const std_msgs::msg::Int32::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);

  if (_msg->data < 0 || _msg->data > 3) {
    RCLCPP_WARN(
      this->rosNode->get_logger(),
      "set_camera_resolution_mode must be between 0 - 3: "
      "0=2MP(2048x1088)@15fps, 1=1MP(2048x544)@30fps, "
      "2=0.5MP(1024x544)@60fps (default), 3=VGA(640x480)@70fps");
    return;
  }

  this->imagerMode = _msg->data;

  double maxRate = 30.0;
  switch (this->imagerMode) {
    case 0:
      maxRate = 15.0;
      break;
    case 1:
      maxRate = 30.0;
      break;
    case 2:
      maxRate = 60.0;
      break;
    case 3:
      maxRate = 70.0;
      break;
    default:
      break;
  }
  if (this->multiCameraFrameRate > maxRate) {
    RCLCPP_INFO(
      this->rosNode->get_logger(), "Reducing frame rate to %.0fHz.", maxRate);
    this->multiCameraFrameRate = maxRate;
  }

  // No live camera-sensor reconfiguration API in gz-sim reachable from a
  // System plugin -- imagerMode/multiCameraFrameRate are cached values
  // only; the original's per-mode image width/height reconfiguration
  // (Camera::SetImageWidth/Height()) has no equivalent here.
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::SetMultiCameraExposureTime(
  const std_msgs::msg::Float64::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->multiCameraExposureTime = _msg->data;
  gzwarn << "setting camera exposure time in sim not implemented" << std::endl;
}

//////////////////////////////////////////////////
void MultiSenseSLPlugin::SetMultiCameraGain(
  const std_msgs::msg::Float64::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->controlMutex);
  this->multiCameraGain = _msg->data;
  gzwarn << "setting camera gain in sim not implemented" << std::endl;
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(MultiSenseSLPlugin,
              gz::sim::System,
              MultiSenseSLPlugin::ISystemConfigure,
              MultiSenseSLPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(MultiSenseSLPlugin,
    "drcsim_gazebo_ros_plugins::MultiSenseSLPlugin")
