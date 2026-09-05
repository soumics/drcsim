/*
 * Copyright 2014 Open Source Robotics Foundation
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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__ROBOTIQHANDPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__ROBOTIQHANDPLUGIN_HPP_

#include <chrono>
#include <cstdint>
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

#include <atlas_msgs/msg/s_model_robot_input.hpp>
#include <atlas_msgs/msg/s_model_robot_output.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Drives a Robotiq 3-Finger Adaptive Gripper: a state machine over
/// the SModel Robot Output/Input protocol (activation, grasping mode,
/// per-finger position/speed/force targets, object-detection feedback),
/// with PID position control of the five actuated hinge joints. Ported from
/// the original Gazebo-Classic `RobotiqHandPlugin` (drcsim, ROS 1 era) to
/// gz-sim's System interface for Gazebo Harmonic, and from roscpp to rclcpp.
///
/// Design notes vs. the original:
/// - PID: `gazebo::common::PID` -> `gz::math::PID`. Same gains/limits
///   semantics, error convention (`error = current - target`), and
///   `Update(error, dt)` contract, just with `dt` as a
///   `std::chrono::duration<double>` instead of a bare double, and getters
///   without their old `Get` prefix (`PGain()`, `Errors()`, ...).
/// - Effort limits: the original read each actuated joint's effort limit
///   off physics directly (`Joint::GetEffortLimit(0)`) to size the PID's
///   command range. `gz::sim::Joint` has no such direct reader; read the
///   SDF-configured limit once at load time via `Joint::Axis(ecm)` ->
///   `sdf::JointAxis::Effort()` instead.
/// - Two joint vectors, preserved exactly from the original because they
///   encode a real mechanical distinction, not an implementation detail:
///   `fingerJoints` (5) are the actuated hinges `SetForce()` is called on;
///   `joints` (12) are every joint whose position/velocity is meaningful to
///   read, used both for full `JointState` publishing and as the PID
///   feedback source. For the two simple revolute finger-spread joints
///   these are the same joint; for the three underactuated 4-bar-linkage
///   fingers they differ on purpose -- the PID reads the finger's actual
///   curl angle off `finger_N_joint_1` but drives torque through
///   `finger_N_joint_proximal_actuating_hinge`, the real motor DOF.
/// - `JointState.effort` is left at zero for the 10 joints that aren't a
///   plugin-driven hinge: gz-sim has no generic "last applied generalized
///   force" reader for an arbitrary joint (same gap noted for other
///   plugins' dropped `GetForce()`/damping-mutation calls), and for the
///   3 underactuated fingers the meaningful applied torque was commanded
///   on a *different* joint (the actuating hinge) than the one being read.
/// - `VerifyCommand()`: the original checked every field's range against
///   `_command->rACT` (a copy-paste bug -- since valid `rACT` is always
///   0 or 1, every other range check silently always passed). Fixed here
///   to check each field against itself; this can only make invalid
///   commands (out of a field's own declared range) newly get rejected,
///   it changes nothing for any command that was already valid.
/// - ROS integration: same shared-process/shared-rclcpp-context pattern as
///   SandiaHandPlugin/IRobotHandPlugin -- owns its own rclcpp::Node (name
///   includes the side), spun on a dedicated thread via a
///   SingleThreadedExecutor. The original's TCP_NODELAY transport hint on
///   the command subscription has no direct DDS-QoS equivalent and is
///   dropped (affects only latency/jitter under load, not correctness).
class RobotiqHandPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  RobotiqHandPlugin();
  ~RobotiqHandPlugin() override;

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
  enum State
  {
    Disabled = 0,
    Emergency,
    ICS,
    ICF,
    ChangeModeInProgress,
    Simplified
  };

  enum GraspingMode
  {
    Basic = 0,
    Pinch,
    Wide,
    Scissor
  };

  void Load(gz::sim::EntityComponentManager & _ecm);
  bool FindJoints(gz::sim::EntityComponentManager & _ecm);
  bool GetAndPushBackJoint(
    gz::sim::EntityComponentManager & _ecm,
    const std::string & _jointName,
    std::vector<gz::sim::Entity> & _joints);

  void SetHandleCommand(
    const atlas_msgs::msg::SModelRobotOutput::SharedPtr _msg);
  void UpdatePIDControl(gz::sim::EntityComponentManager & _ecm, double _dt);
  void GetAndPublishHandleState(gz::sim::EntityComponentManager & _ecm);
  void GetAndPublishJointState(
    const gz::sim::EntityComponentManager & _ecm, const rclcpp::Time & _stamp);
  void UpdateStates(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm);

  void ReleaseHand();
  void StopHand();
  bool IsHandFullyOpen(gz::sim::EntityComponentManager & _ecm);

  uint8_t GetObjectDetection(
    gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _jointEntity,
    int _index, uint8_t _rPR, uint8_t _prevRPR);
  uint8_t GetCurrentPosition(
    gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _jointEntity);

  bool VerifyField(const std::string & _label, int _min, int _max, int _v);
  bool VerifyCommand(const atlas_msgs::msg::SModelRobotOutput & _command);

  static double FirstOrZero(const std::optional<std::vector<double>> & _values);

  gz::sim::Model model{gz::sim::kNullEntity};
  std::shared_ptr<const sdf::Element> sdfConfig;
  bool initialized{false};
  bool validConfig{false};
  std::string side;

  static constexpr int kNumJoints = 5;
  static constexpr double kVelTolerance = 0.002;
  static constexpr double kPoseTolerance = 0.002;
  static constexpr double kMinVelocity = 0.176;
  static constexpr double kMaxVelocity = 0.88;

  std::vector<std::string> jointNames;
  std::vector<gz::sim::Entity> fingerJoints;
  std::vector<gz::sim::Entity> joints;

  gz::math::PID posePID[kNumJoints];

  GraspingMode graspingMode{Basic};
  State handState{Disabled};

  atlas_msgs::msg::SModelRobotOutput handleCommand;
  atlas_msgs::msg::SModelRobotOutput lastHandleCommand;
  atlas_msgs::msg::SModelRobotOutput prevCommand;
  atlas_msgs::msg::SModelRobotOutput userHandleCommand;
  atlas_msgs::msg::SModelRobotInput handleState;

  sensor_msgs::msg::JointState jointStates;

  std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  std::mutex controlMutex;

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread rosSpinThread;

  rclcpp::Publisher<atlas_msgs::msg::SModelRobotInput>::SharedPtr pubHandleState;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointStates;
  rclcpp::Subscription<atlas_msgs::msg::SModelRobotOutput>::SharedPtr subHandleCommand;
};
}  // namespace drcsim_gazebo_ros_plugins
#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__ROBOTIQHANDPLUGIN_HPP_
