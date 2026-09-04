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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__SANDIAHANDPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__SANDIAHANDPLUGIN_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <gz/msgs/contacts.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>

#include <atlas_msgs/srv/get_joint_damping.hpp>
#include <atlas_msgs/srv/set_joint_damping.hpp>
#include <osrf_msgs/msg/joint_commands.hpp>
#include <sandia_hand_msgs/msg/raw_tactile.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Drives a Sandia Hand model: per-joint PID position/velocity
/// control from osrf_msgs/JointCommands, sensor_msgs/JointState feedback,
/// an IMU feed, a joint-damping get/set service pair, and a synthesized
/// tactile sensor array built from contact data. Ported from the original
/// Gazebo-Classic SandiaHandPlugin (drcsim, ROS 1 era) to gz-sim's System
/// interface for Gazebo Harmonic, and from roscpp to rclcpp.
///
/// Design notes vs. the original:
/// - IMU: the original read gazebo::sensors::ImuSensor (with its
///   configured noise model). gz-sim's IMU sensor only exposes its output
///   via a gz-transport topic, not a readable ECS component, and needs the
///   gz-sim-imu-system world plugin loaded. This port instead reads the
///   IMU link's raw physics state directly off the ECS (Link::WorldPose/
///   WorldAngularVelocity/WorldLinearAcceleration, world-frame vectors
///   rotated into the link's own frame) -- simpler, has no dependency on
///   the Imu system plugin, but drops the SDF-configured noise model.
/// - Joint damping service: the original applied the requested damping
///   coefficient straight to the physics joint (physics::Joint::
///   SetDamping()). gz-sim has no well-supported runtime joint-damping
///   mutation API (same situation as DRCVehiclePlugin's dropped
///   Set*Limits, see drcsim_gazebo_plugins), so this only updates a
///   cached, clamped value; the service's request/response contract is
///   unchanged.
/// - Tactile data: the original subscribed to the model's implicit
///   aggregate contact topic and filtered incoming contacts by substring
///   matching on collision name. gz-sim has no such default per-model
///   contact topic (same gap ContactModelPlugin's port already worked
///   around); this instead force-creates a components::ContactSensorData
///   component on each of this hand's own finger/palm collisions (the
///   same technique) and matches contacts back to them by
///   gz::msgs::Entity id (exact, not substring matching).
/// - ROS integration: owns its own rclcpp::Node, spun on a dedicated
///   thread with a SingleThreadedExecutor. Calls rclcpp::init() only if
///   rclcpp::ok() is false, and never calls rclcpp::shutdown() from the
///   destructor -- multiple ROS-coupled plugins (this one, and future
///   ones: RobotiqHandPlugin, IRobotHandPlugin, MultiSenseSLPlugin, ...)
///   share one process and one global rclcpp context; only this
///   instance's own executor/thread/node are torn down.
/// - PID gains: the original pulled p/i/d/i_clamp per joint off the ROS
///   parameter server (populated externally by a launch file this
///   migration hasn't ported yet). Declared as ROS 2 parameters instead
///   (gains.f<N>_j<M>.{p,i,d,i_clamp}, default 0.0), settable the same
///   way once a launch file provides them.
class SandiaHandPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  SandiaHandPlugin();
  ~SandiaHandPlugin() override;

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
  struct ErrorTerms
  {
    double positionError{0.0};
    double positionErrorDerivative{0.0};
    double positionErrorIntegral{0.0};
    double velocityError{0.0};
  };

  /// \brief Per-collision tactile-array bucket, precomputed once so
  /// FillTactileData() doesn't need to re-parse collision names per
  /// contact. Mirrors the classification the original derived from
  /// collision-name substring matching each time.
  struct TactileCollisionInfo
  {
    bool isPalm{false};
    int fingerIdx{-1};
    int fingerColIdx{-1};
    int palmIdx{-1};
  };

  void Load(gz::sim::EntityComponentManager & _ecm);
  void UpdateStates(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm);
  void FillTactileData(
    const gz::sim::EntityComponentManager & _ecm,
    const std::vector<gz::msgs::Contact> & _contacts,
    sandia_hand_msgs::msg::RawTactile & _tactileMsg);
  static double FirstOrZero(const std::optional<std::vector<double>> & _values);
  static void CopyVectorIfValid(
    const std::vector<double> & _from, std::vector<double> & _to);

  void SetJointCommands(const osrf_msgs::msg::JointCommands::SharedPtr _msg);
  void SetJointDamping(
    const std::shared_ptr<atlas_msgs::srv::SetJointDamping::Request> _req,
    std::shared_ptr<atlas_msgs::srv::SetJointDamping::Response> _res);
  void GetJointDamping(
    const std::shared_ptr<atlas_msgs::srv::GetJointDamping::Request> _req,
    std::shared_ptr<atlas_msgs::srv::GetJointDamping::Response> _res);

  gz::sim::Model model{gz::sim::kNullEntity};
  std::shared_ptr<const sdf::Element> sdfConfig;
  bool initialized{false};
  bool validConfig{false};
  bool hasStumps{false};

  std::string side;
  std::string imuLinkName;
  gz::sim::Entity imuLinkEntity{gz::sim::kNullEntity};

  static constexpr unsigned int kNumJoints = 12;
  std::vector<std::string> jointNames;
  std::vector<gz::sim::Entity> jointEntities;
  std::vector<ErrorTerms> errorTerms;
  std::vector<double> jointDampingMin;
  std::vector<double> jointDampingMax;
  std::vector<double> jointDampingCurrent;

  osrf_msgs::msg::JointCommands jointCommands;
  sensor_msgs::msg::JointState jointStates;
  sandia_hand_msgs::msg::RawTactile tactile;

  int tactileFingerArraySize{18};
  int tactilePalmArraySize{32};
  int maxTactileOut{33500};
  int minTactileOut{26500};
  double palmColWidth[5]{};
  double palmColLength[5]{};
  int palmHorSize[5]{};
  int palmVerSize[5]{};
  double fingerColLength[2]{};
  double fingerColWidth[2]{};
  int fingerHorSize[2]{};
  int fingerVerSize[2]{};

  std::unordered_map<gz::sim::Entity, TactileCollisionInfo> tactileCollisions;

  std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  std::mutex commandMutex;

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread rosSpinThread;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointStates;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu;
  rclcpp::Publisher<sandia_hand_msgs::msg::RawTactile>::SharedPtr pubTactile;
  rclcpp::Subscription<osrf_msgs::msg::JointCommands>::SharedPtr subJointCommands;
  rclcpp::Service<atlas_msgs::srv::SetJointDamping>::SharedPtr setJointDampingService;
  rclcpp::Service<atlas_msgs::srv::GetJointDamping>::SharedPtr getJointDampingService;
};
}  // namespace drcsim_gazebo_ros_plugins
#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__SANDIAHANDPLUGIN_HPP_
