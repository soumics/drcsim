/*
 * Copyright 2013 Open Source Robotics Foundation
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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__IROBOTHANDPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__IROBOTHANDPLUGIN_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>

#include <handle_msgs/msg/handle_control.hpp>
#include <handle_msgs/msg/handle_sensors.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Drives an iRobot/HANDLE hand model: tendon-actuated finger flex,
/// spread, and a thumb antagonist DOF, from a handle_msgs/HandleControl
/// command stream, with sensor_msgs/JointState and handle_msgs/HandleSensors
/// feedback. Ported from the original Gazebo-Classic IRobotHandPlugin
/// (drcsim, ROS 1 era) to gz-sim's System interface for Gazebo Harmonic, and
/// from roscpp to rclcpp.
///
/// Design notes vs. the original:
/// - Passive joint springs: the original faked the flexure/base joints'
///   passive springiness using gazebo::physics::Joint::SetStiffnessDamping()
///   (an ODE-specific joint-spring feature) and re-called it every step in
///   UpdatePIDControl() with a dynamically adjusted damping value ("hack to
///   reduce jitter"). gz-sim's Joint wrapper has no equivalent runtime
///   stiffness/damping mutation API (same gap as DRCVehiclePlugin's dropped
///   Set*Limits and SandiaHandPlugin's dropped damping service). A spring is
///   just a PD controller (force = -kp*(position - preload) - kd*velocity),
///   so this port computes that force explicitly every step and adds it to
///   the active tendon force in a single SetForce() call per joint, instead
///   of trying to mutate a physics-engine spring feature that doesn't exist
///   here.
/// - Thumb antagonist limit: the original also called
///   Joint::SetUpperLimit() every step to give the thumb base joint a hard
///   physics stop at the antagonist-adjusted position. gz-sim has no runtime
///   joint-limit mutation API either, so only the control-level check
///   the original already had independently (skip applying tendon force
///   once the joint has reached the effective limit) is kept; there is no
///   hard physics stop backing it up anymore.
/// - ROS integration: same shared-process/shared-rclcpp-context pattern as
///   SandiaHandPlugin -- owns its own rclcpp::Node (name includes the side,
///   since two instances of this plugin, left and right, run in the same
///   process), spun on a dedicated thread via a SingleThreadedExecutor.
/// - No "stumps" fallback: unlike SandiaHandPlugin, the original aborts the
///   whole plugin load if even one of the 23 expected joints per hand is
///   missing, and this port preserves that -- there is no reduced-function
///   mode.
class IRobotHandPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  IRobotHandPlugin();
  ~IRobotHandPlugin() override;

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
  };

  /// \brief Per-joint passive-spring parameters (see the class-level design
  /// note on how these replace Joint::SetStiffnessDamping()).
  struct SpringParams
  {
    double kp{0.0};
    double kd{0.0};
    double preload{0.0};
  };

  void Load(gz::sim::EntityComponentManager & _ecm);
  bool FindJoints(gz::sim::EntityComponentManager & _ecm);
  bool GetAndPushBackJoint(
    gz::sim::EntityComponentManager & _ecm,
    const std::string & _jointName,
    std::vector<gz::sim::Entity> & _joints);
  void SetJointSpringParams();
  void UpdateStates(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm);
  void GetAndPublishHandleState(
    const gz::sim::EntityComponentManager & _ecm,
    const rclcpp::Time & _stamp);
  void UpdatePIDControl(gz::sim::EntityComponentManager & _ecm, double _dt);
  static double SpringForce(
    const SpringParams & _spring, double _position, double _velocity);
  static double HandleControlFlexValueToFlexJointAngle(int32_t _value);
  static double HandleControlSpreadValueToSpreadJointAngle(int32_t _value);
  static double FirstOrZero(const std::optional<std::vector<double>> & _values);

  void SetHandleCommand(const handle_msgs::msg::HandleControl::SharedPtr _msg);

  gz::sim::Model model{gz::sim::kNullEntity};
  std::shared_ptr<const sdf::Element> sdfConfig;
  bool initialized{false};
  bool validConfig{false};
  std::string side;

  static constexpr int kNumFingers = 3;
  static constexpr int kNumFlexLinks = 2;

  std::vector<gz::sim::Entity> fingerBaseJoints;
  std::vector<gz::sim::Entity> fingerBaseRotationJoints;
  std::vector<std::vector<gz::sim::Entity>> flexureFlexJoints;
  std::vector<std::vector<gz::sim::Entity>> flexureTwistJoints;

  std::vector<SpringParams> fingerBaseSprings;
  std::vector<SpringParams> fingerBaseRotationSprings;
  std::vector<std::vector<SpringParams>> flexureFlexSprings;
  std::vector<std::vector<SpringParams>> flexureTwistSprings;

  double thumbUpperLimit{0.0};
  double thumbLowerLimit{0.0};
  double thumbAntagonistAngle{0.0};

  std::vector<ErrorTerms> errorTerms;

  double kpPosition[5]{1.0, 1.0, 1.0, 1.0, 1.0};
  double kiPosition[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  double kdPosition[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  double iPositionEffortMin[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  double iPositionEffortMax[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  double kpVelocity[5]{0.1, 0.1, 0.1, 0.1, 0.1};
  double kiVelocity[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  double kdVelocity[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  double iVelocityEffortMin[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  double iVelocityEffortMax[5]{0.0, 0.0, 0.0, 0.0, 0.0};

  handle_msgs::msg::HandleControl handleCommand;
  handle_msgs::msg::HandleSensors handleState;
  sensor_msgs::msg::JointState jointStates;

  std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  std::mutex controlMutex;

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread rosSpinThread;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointStates;
  rclcpp::Publisher<handle_msgs::msg::HandleSensors>::SharedPtr pubHandleState;
  rclcpp::Subscription<handle_msgs::msg::HandleControl>::SharedPtr subHandleCommand;
};
}  // namespace drcsim_gazebo_ros_plugins
#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__IROBOTHANDPLUGIN_HPP_
