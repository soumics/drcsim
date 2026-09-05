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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__MULTISENSESLPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__MULTISENSESLPLUGIN_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/math/PID.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Drives the MultiSense SL head sensor head: a velocity-PID-driven
/// spindle joint (rotates the head lidar), a head IMU feed, joint state
/// publishing for the spindle, and ROS topics for adjusting the stereo
/// camera's frame rate/resolution/exposure/gain. Ported from the original
/// Gazebo-Classic `MultiSenseSL` plugin (drcsim, ROS 1 era) to gz-sim's
/// System interface for Gazebo Harmonic, and from roscpp to rclcpp.
///
/// Design notes vs. the original:
/// - Spindle velocity PID: `gazebo::common::PID` -> `gz::math::PID`, same
///   pattern already used by `DRCVehiclePlugin` and `RobotiqHandPlugin`.
/// - IMU: the original read a dedicated `sensors::ImuSensor` object
///   directly. gz-sim's IMU sensor output is transport-only (needs the
///   `gz-sim-imu-system` world plugin loaded, and isn't readable off the
///   ECM), so -- same technique as `SandiaHandPlugin`'s IMU handling --
///   this reads the head link's raw physics state instead:
///   `Link::WorldPose`/`WorldAngularVelocity`/`WorldLinearAcceleration`
///   (the latter two need `EnableVelocityChecks`/`EnableAccelerationChecks`
///   called once first), with world-frame vectors rotated into the link's
///   own frame via `pose.Rot().RotateVectorReverse(worldVec)`.
/// - Camera control topics (frame rate, resolution mode, exposure, gain):
///   the original mutated a live `sensors::MultiCameraSensor` object found
///   via the global `SensorManager` singleton (`SetUpdateRate()`,
///   `SetImageWidth/Height()` on each camera). gz-sim has no equivalent
///   runtime camera-reconfiguration API reachable from a `System` plugin
///   (same category of gap as the dropped joint-damping/limit mutations in
///   `DRCVehiclePlugin`/`SandiaHandPlugin`/`IRobotHandPlugin`) -- these
///   topics/services are kept so downstream code depending on this
///   interface doesn't break, but they now only update a cached value,
///   they no longer reconfigure a live camera. `SetMultiCameraExposureTime`/
///   `SetMultiCameraGain` were already no-ops in the original ("not
///   implemented" `gzwarn`), so no behavior is lost there specifically.
/// - The original picked its ROS namespace (`/multisense` vs
///   `/multisense_sl`) by reading a shared ROS 1 param-server value,
///   `/atlas_version`, that some other node was expected to have already
///   set. ROS 2 has no shared/global parameter server -- same gap already
///   handled for `SandiaHandPlugin`'s PID gains -- so `atlas_version` is
///   declared as this node's own parameter instead (default `5`, the
///   modern Atlas), settable the same way once a launch file provides it.
/// - ROS integration: same shared-process/shared-rclcpp-context pattern as
///   the other Tier 2 ROS-coupled plugins.
class MultiSenseSLPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  MultiSenseSLPlugin();
  ~MultiSenseSLPlugin() override;

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
  void Load(gz::sim::EntityComponentManager & _ecm);
  void UpdateStates(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm);

  void SetSpindleSpeed(const std_msgs::msg::Float64::SharedPtr _msg);
  void SetSpindleState(const std_msgs::msg::Bool::SharedPtr _msg);
  void SetMultiCameraFrameRateOld(const std_msgs::msg::Float64::SharedPtr _msg);
  void SetMultiCameraFrameRate(const std_msgs::msg::Float64::SharedPtr _msg);
  void SetMultiCameraResolution(const std_msgs::msg::Int32::SharedPtr _msg);
  void SetMultiCameraExposureTime(const std_msgs::msg::Float64::SharedPtr _msg);
  void SetMultiCameraGain(const std_msgs::msg::Float64::SharedPtr _msg);

  static double FirstOrZero(const std::optional<std::vector<double>> & _values);

  gz::sim::Model model{gz::sim::kNullEntity};
  std::shared_ptr<const sdf::Element> sdfConfig;
  bool initialized{false};
  bool validConfig{false};

  std::string imuLinkName{"head"};
  gz::sim::Entity imuLinkEntity{gz::sim::kNullEntity};

  gz::sim::Entity spindleJointEntity{gz::sim::kNullEntity};

  std::string rosNamespace{"/multisense"};

  double multiCameraFrameRate{30.0};
  double multiCameraExposureTime{0.001};
  double multiCameraGain{1.0};
  int imagerMode{1};

  double spindleSpeed{0.0};
  double spindleMaxRPM{50.0};
  double spindleMinRPM{0.0};
  bool spindleOn{true};
  gz::math::PID spindlePID;

  sensor_msgs::msg::JointState jointStates;

  std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  std::mutex controlMutex;

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread rosSpinThread;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointStates;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subSetSpindleSpeed;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subSetSpindleState;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subSetMultiCameraFrameRateOld;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subSetMultiCameraFrameRate;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subSetMultiCameraResolution;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subSetMultiCameraExposureTime;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subSetMultiCameraGain;
};
}  // namespace drcsim_gazebo_ros_plugins
#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__MULTISENSESLPLUGIN_HPP_
