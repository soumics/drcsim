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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__DRCVEHICLEROSPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__DRCVEHICLEROSPLUGIN_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <drcsim_gazebo_plugins/DRCVehiclePlugin.hpp>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Thin ROS wrapper around `drcsim_gazebo_plugins::DRCVehiclePlugin`:
/// subscribes to `<model>/{hand_wheel,hand_brake,gas_pedal,brake_pedal}/cmd`
/// and `<model>/{key,direction}/cmd`, calling straight into the base
/// class's existing setters, and periodically publishes the corresponding
/// `.../state` topics. Ported from the original Gazebo-Classic
/// `DRCVehicleROSPlugin` (drcsim, ROS 1 era) to gz-sim's System interface
/// for Gazebo Harmonic, and from roscpp to rclcpp.
///
/// Design notes vs. the original:
/// - **Cross-package subclassing.** This is the first plugin in the
///   migration where one plugin's `.so` needs another plugin package's
///   class definition at link time (`DRCVehiclePlugin` lives in
///   `drcsim_gazebo_plugins`, built and installed *before* this package).
///   That required adding a proper ament CMake target export
///   (`ament_export_targets`) to `drcsim_gazebo_plugins`'s `CMakeLists.txt`
///   -- previously nothing in that package needed to be linked against by
///   another package, only loaded by gz-sim directly as a `.so`.
/// - **Mutex added around the base class's plain cached command
///   fields** (`handWheelCmd`, `gasPedalCmd`, `directionState`, ...): the
///   original had a genuine, if benign, data race here too (ROS's
///   callback-queue thread writes these `double`/enum members directly,
///   Classic's physics thread reads them in `OnUpdate()`, no lock either
///   side) -- tolerated in the original because torn reads/writes of a
///   `double` are harmless in practice on real hardware. Rather than
///   preserve that specific race, this port wraps every ROS callback and
///   the `PreUpdate` call into the base class in one `cmdMutex`, matching
///   this migration's established pattern (`AtlasPlugin`'s `controlMutex`,
///   etc.) of protecting cross-thread-shared plain state even where the
///   original didn't bother.
/// - **`IsValidConfig()` was added to `DRCVehiclePlugin`** (previously
///   private, no accessor) so this subclass can replicate the original's
///   `try { DRCVehiclePlugin::Load(...) } catch (...) { return; }` --
///   skip setting up the ROS interface at all if the base plugin failed
///   to configure (unresolved joints, wrong SDF, ...). gz-sim doesn't use
///   exceptions for this class of failure (see `DRCVehiclePlugin`'s own
///   design notes), so a boolean getter is the direct equivalent.
/// - Same `VRC_CHEATS_ENABLED` gate as the original: with it unset (the
///   default), this class behaves identically to a plain
///   `DRCVehiclePlugin` -- no ROS node, no topics at all. Directly
///   commanding the vehicle's pedals/wheel over ROS, bypassing physical
///   actuation by the robot, is considered a "cheat".
/// - `ros::CallbackQueue`/its own polling thread replaced with the
///   standard `rclcpp::executors::SingleThreadedExecutor` spun on a
///   dedicated thread, matching every other ROS-coupled plugin in this
///   migration.
class DRCVehicleROSPlugin
  : public drcsim_gazebo_plugins::DRCVehiclePlugin
{
public:
  DRCVehicleROSPlugin() = default;

  ~DRCVehicleROSPlugin() override;

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
  /// \brief Sets the state of the key switch (0 = OFF, 1 = ON).
  void SetKeyState(const std_msgs::msg::Int8::SharedPtr _msg);

  /// \brief Sets the state of the direction switch (-1 = REVERSE,
  /// 0 = NEUTRAL, 1 = FORWARD).
  void SetDirectionState(const std_msgs::msg::Int8::SharedPtr _msg);

  /// \brief Set the steering wheel angle (radians).
  void SetHandWheelState(const std_msgs::msg::Float64::SharedPtr _msg);

  /// \brief Set the hand brake position, as a percentage (0-1) of its
  /// range of travel.
  void SetHandBrakePercent(const std_msgs::msg::Float64::SharedPtr _msg);

  /// \brief Set the gas pedal position, as a percentage (0-1) of its
  /// range of travel.
  void SetGasPedalPercent(const std_msgs::msg::Float64::SharedPtr _msg);

  /// \brief Set the brake pedal position, as a percentage (0-1) of its
  /// range of travel.
  void SetBrakePedalPercent(const std_msgs::msg::Float64::SharedPtr _msg);

  /// \brief Publish the hand wheel, pedal, hand brake, key, and direction
  /// states, throttled to `rosPublishPeriod`.
  void RosPublishStates(const std::chrono::steady_clock::duration & _simTime);

  /// \brief Guards every field `DRCVehiclePlugin` caches from a ROS
  /// callback and reads back in its own `PreUpdate` (see the class-level
  /// design note).
  std::mutex cmdMutex;

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread rosSpinThread;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubHandWheelState;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubHandBrakeState;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubGasPedalState;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubBrakePedalState;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubKeyState;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubDirectionState;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subHandWheelCmd;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subHandBrakeCmd;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subGasPedalCmd;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subBrakePedalCmd;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subKeyCmd;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subDirectionCmd;

  /// \brief Are cheats enabled? Gates whether any ROS interface is
  /// created at all (see the class-level design note).
  bool cheatsEnabled{false};

  std::chrono::steady_clock::duration rosPublishPeriod{std::chrono::milliseconds(50)};
  std::chrono::steady_clock::duration lastRosPublishTime{
    std::chrono::steady_clock::duration::zero()};
};
}  // namespace drcsim_gazebo_ros_plugins
#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__DRCVEHICLEROSPLUGIN_HPP_
