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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__ATLASPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__ATLASPLUGIN_HPP_

#define FIL_N_STEPS 2

#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>

#include <atlas_msgs/msg/atlas_command.hpp>
#include <atlas_msgs/msg/atlas_sim_interface_command.hpp>
#include <atlas_msgs/msg/atlas_sim_interface_state.hpp>
#include <atlas_msgs/msg/atlas_state.hpp>
#include <atlas_msgs/msg/controller_statistics.hpp>
#include <atlas_msgs/msg/force_torque_sensors.hpp>
#include <atlas_msgs/msg/synchronization_statistics.hpp>
#include <atlas_msgs/msg/test.hpp>
#include <atlas_msgs/srv/atlas_filters.hpp>
#include <atlas_msgs/srv/get_joint_damping.hpp>
#include <atlas_msgs/srv/reset_controls.hpp>
#include <atlas_msgs/srv/set_joint_damping.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <osrf_msgs/msg/joint_commands.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Drives the Atlas robot's joints: a 1kHz PID position/velocity/
/// effort controller fed from `atlas/atlas_command`, plus a "BDI behavior
/// library" (`AtlasSimInterface`) integration fed from
/// `atlas/atlas_sim_interface_command`, blended per-joint by a `k_effort`
/// weight (0 = pure BDI feedforward, 255 = pure PID -- matching the
/// original's `uint8` 0-255 convention). Ported from the original
/// Gazebo-Classic `AtlasPlugin` (drcsim, ROS 1 era, by far the largest
/// single file in this migration at ~3500 lines across the header and
/// source) to gz-sim's System interface for Gazebo Harmonic, and from
/// roscpp to rclcpp.
///
/// Design notes vs. the original:
/// - **The "proprietary AtlasSimInterface" risk flagged at the start of
///   this migration turned out not to exist.** All three bundled versions
///   (`drcsim_model_resources/AtlasSimInterface_{1.1.1,2.10.2,3.0.2}/src/
///   AtlasSimInterface.cc`) are byte-for-byte identical, Apache-2.0-
///   licensed OSRF stub code that prints `"Warning: Using Atlas Shim
///   interface. Atlas will be more or less uncontrolled"` on load. Every
///   "BDI behavior" function (`set_desired_behavior`, `get_current_
///   behavior`, walk/step/stand param processing, ...) is a no-op that
///   only returns `NO_ERRORS`; `process_control_input()` -- the one
///   function that does anything -- is itself just a joint-position/
///   velocity/effort PID identical in shape to the main controller below.
///   There is no real Boston Dynamics balance controller anywhere in this
///   codebase, no proprietary binary to link against, and therefore no
///   ABI/toolchain risk. This port drops the external-library integration
///   (`create_atlas_sim_interface()`, the version-conditional `#include
///   "AtlasSimInterface_X.Y.Z/AtlasSimInterface.h"`, the `AtlasControlInput
///   /AtlasControlOutput/AtlasRobotState` C-struct plumbing) entirely and
///   inlines the shim's exact PID formula as `UpdateAtlasSimInterface()`,
///   operating directly on `atlas_msgs` ROS types instead of those C
///   structs.
/// - **A close corollary**: since the shim's `get_desired_behavior()`/
///   `get_current_behavior()` never actually write their output-string
///   reference parameter, the original's `this->behaviorMap[behaviorStr]`
///   was looking up an always-empty key -- meaning `asiState.
///   current_behavior` was **always** silently reported as `NONE`,
///   regardless of what was actually requested, in the original too. Not
///   preserved: this port just sets `current_behavior = desired_behavior`
///   directly, since faithfully reproducing an always-wrong report serves
///   no one and this is clearly an artifact of the incomplete shim, not a
///   deliberate design choice (same judgment already applied once before,
///   to `RobotiqHandPlugin`'s copy-paste `VerifyCommand()` bug).
/// - **Also following from the same discovery**: the shim's
///   `AtlasControlOutput` (behavior_feedback/stand_feedback/step_feedback/
///   walk_feedback/manipulate_feedback) is only ever zero-initialized in
///   the original and never subsequently written by the shim -- so every
///   one of `AtlasSimInterfaceState`'s rich feedback fields was already
///   permanently zero/inert in the original, shim-backed build. This port
///   leaves them at their message-default zero values rather than
///   building out unused plumbing to compute values nothing upstream ever
///   populated.
/// - **Joint damping mutation**: same gap already established for
///   `DRCVehiclePlugin`/`SandiaHandPlugin`/`RobotiqHandPlugin` -- no
///   runtime joint-damping mutation API in gz-sim. Every place the
///   original calls `Joint::SetDamping()` (`SetAtlasCommand`,
///   `SetJointDamping` service, `SetExperimentalDampingPID`,
///   `UpdatePIDControl`'s "cfm damping pass-through" optimization) now
///   only updates a cached value; the surrounding control-law math (the
///   `kpVelocityDampingEffort` term used to shift the effort-clamp bounds)
///   is pure arithmetic and is preserved exactly, since it doesn't depend
///   on the mutation actually reaching physics.
/// - **Wrist/ankle force-torque "sensors"**: Classic's `Joint::
///   GetForceTorque(0u)` returned a `body1Force/body1Torque/body2Force/
///   body2Torque` 2-body wrench pair. gz-sim's equivalent,
///   `Joint::TransmittedWrench()` (needs `EnableTransmittedWrenchCheck()`
///   called once first, same pattern as `EnablePositionCheck()`), gives
///   only a single force/torque pair expressed in the **joint frame**,
///   applied at the joint origin -- not the child-link frame Classic's
///   `body2Force`/`body2Torque` used. This port uses that single wrench
///   directly as the `body2` analog without the frame transform back to
///   the child link (an approximation -- exact for a joint whose frame
///   coincides with its child link's, which is common for a single-DOF
///   limb joint that follows the link's own axis convention, but not
///   guaranteed in general).
/// - **Foot contact sensors** (the `atlas/debug/{l,r}_foot_contact`
///   debug-only topics): the same force-created `components::
///   ContactSensorData` technique already established for
///   `ContactModelPlugin`/`SandiaHandPlugin`'s tactile sensing.
/// - **Controller synchronization delay** (`EnforceSynchronizationDelay()`,
///   a lockstep mechanism letting an external controller ask the sim to
///   wait, via a real-time delay budget, for a fresh `AtlasCommand` before
///   proceeding): pure `boost::condition`/`boost::mutex` logic with no
///   gz-sim-specific dependency, ported 1:1 onto `std::condition_variable`/
///   `std::mutex`. Off by default (`desired_controller_period_ms == 0`).
/// - **`jointCommands`/`ZeroJointCommands()`**: dead state in the original
///   -- `SetJointCommands()` (the `atlas/joint_commands` ROS callback)
///   writes its incoming message straight into `atlasCommand`/`atlasState`
///   and never touches the separately-declared `jointCommands` member at
///   all, so it was only ever written by its own zeroing function. Dropped
///   rather than carried forward as unreachable state.
/// - **ROS integration**: same shared-process/shared-rclcpp-context
///   pattern as every other Tier 2 ROS-coupled plugin. Unlike
///   `VRCPlugin`, none of this plugin's ROS callbacks need to create/
///   destroy entities or teleport a model -- every one of them (`SetAtlas
///   Command`, `SetJointCommands`, `SetASICommand`, `OnRobotMode`, `Tic`,
///   `SetExperimentalDampingPID`, and the four services) only ever updates
///   a mutex-protected cached value that the real work
///   (`UpdatePIDControl`/`UpdateAtlasSimInterface`, both sim-thread-only)
///   reads once per tick -- the same safe pattern already used in
///   `SandiaHandPlugin`'s `SetJointCommands`, just applied consistently
///   here instead of needing a `VRCPlugin`-style pending-action queue.
class AtlasPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  AtlasPlugin();
  ~AtlasPlugin() override;

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
  void LoadROS();
  bool GetAtlasVersion();
  std::string FindJoint(
    gz::sim::EntityComponentManager & _ecm, const std::string & _st1,
    const std::string & _st2);
  std::string FindJoint(
    gz::sim::EntityComponentManager & _ecm, const std::string & _st1,
    const std::string & _st2, const std::string & _st3);

  void UpdateStates(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm);
  void GetAndPublishRobotStates(
    gz::sim::EntityComponentManager & _ecm, const rclcpp::Time & _stamp);
  void GetIMUState(gz::sim::EntityComponentManager & _ecm, const rclcpp::Time & _stamp);
  void GetForceTorqueSensorState(gz::sim::EntityComponentManager & _ecm);
  void UpdatePIDControl(gz::sim::EntityComponentManager & _ecm, double _dt);
  void UpdateAtlasSimInterface(
    gz::sim::EntityComponentManager & _ecm, const rclcpp::Time & _stamp);
  void CalculateControllerStatistics(const rclcpp::Time & _stamp);
  void PublishControllerStatistics(const rclcpp::Time & _stamp);
  void EnforceSynchronizationDelay(const rclcpp::Time & _stamp);
  void InitFilter();
  void Filter(std::vector<float> & _aState, std::vector<double> & _jState);
  void LoadPIDGainsFromParameter();
  void ZeroAtlasCommand();
  void OnLContactUpdate(gz::sim::EntityComponentManager & _ecm);
  void OnRContactUpdate(gz::sim::EntityComponentManager & _ecm);

  // ROS callbacks -- each only updates a mutex-protected cached value (see
  // the class-level design note); the real work happens once per tick from
  // UpdateStates(), on the sim thread only.
  void SetAtlasCommand(const atlas_msgs::msg::AtlasCommand::SharedPtr _msg);
  void SetJointCommands(const osrf_msgs::msg::JointCommands::SharedPtr _msg);
  void SetASICommand(
    const atlas_msgs::msg::AtlasSimInterfaceCommand::SharedPtr _msg);
  void OnRobotMode(const std_msgs::msg::String::SharedPtr _mode);
  void Tic(const std_msgs::msg::String::SharedPtr _msg);
  void SetExperimentalDampingPID(const atlas_msgs::msg::Test::SharedPtr _msg);

  void AtlasFilters(
    const std::shared_ptr<atlas_msgs::srv::AtlasFilters::Request> _req,
    std::shared_ptr<atlas_msgs::srv::AtlasFilters::Response> _res);
  void ResetControls(
    const std::shared_ptr<atlas_msgs::srv::ResetControls::Request> _req,
    std::shared_ptr<atlas_msgs::srv::ResetControls::Response> _res);
  void SetJointDamping(
    const std::shared_ptr<atlas_msgs::srv::SetJointDamping::Request> _req,
    std::shared_ptr<atlas_msgs::srv::SetJointDamping::Response> _res);
  void GetJointDamping(
    const std::shared_ptr<atlas_msgs::srv::GetJointDamping::Request> _req,
    std::shared_ptr<atlas_msgs::srv::GetJointDamping::Response> _res);

  static double FirstOrZero(const std::optional<std::vector<double>> & _values);

  struct ErrorTerms
  {
    double qP{0.0};
    double dQpDt{0.0};
    double kIqI{0.0};
  };

  gz::sim::Model model{gz::sim::kNullEntity};
  std::shared_ptr<const sdf::Element> sdfConfig;
  bool initialized{false};
  bool validConfig{false};
  bool cheatsEnabled{false};

  int atlasVersion{5};
  int atlasSubVersion{0};

  std::vector<std::string> jointNames;
  std::vector<gz::sim::Entity> joints;
  std::vector<double> effortLimit;

  gz::sim::Entity imuLinkEntity{gz::sim::kNullEntity};
  std::string imuLinkName{"imu_link"};

  gz::sim::Entity lAnkleJointEntity{gz::sim::kNullEntity};
  gz::sim::Entity rAnkleJointEntity{gz::sim::kNullEntity};
  gz::sim::Entity lWristJointEntity{gz::sim::kNullEntity};
  gz::sim::Entity rWristJointEntity{gz::sim::kNullEntity};

  /// \brief Foot collisions with a force-created components::
  /// ContactSensorData, keyed by collision entity, for the debug contact
  /// topics (see the class-level design note).
  std::vector<gz::sim::Entity> lFootCollisions;
  std::vector<gz::sim::Entity> rFootCollisions;

  // Controller state, protected by controlMutex.
  std::mutex controlMutex;
  atlas_msgs::msg::AtlasCommand atlasCommand;
  atlas_msgs::msg::AtlasState atlasState;
  sensor_msgs::msg::JointState jointStates;
  std::vector<ErrorTerms> errorTerms;

  // AtlasSimInterface (shim) state, protected by asiMutex.
  std::mutex asiMutex;
  atlas_msgs::msg::AtlasSimInterfaceCommand asiCommand;
  atlas_msgs::msg::AtlasSimInterfaceState asiState;
  std::vector<ErrorTerms> asiErrorTerms;
  int startupStep{0};
  enum StartupSteps
  {
    FREEZE = 0,
    USER = 1,
    NOMINAL = 2,
  };

  // Joint damping: cached only, no runtime mutation API in gz-sim (see the
  // class-level design note).
  std::vector<double> jointDampingModel;
  std::vector<double> jointDampingMax;
  std::vector<double> jointDampingMin;
  std::vector<double> lastJointCFMDamping;

  // Filters.
  std::mutex filterMutex;
  bool filterVelocity{false};
  bool filterPosition{false};
  // Only FIL_N_STEPS coefficients are ever used (the original oversized
  // these to FIL_MAX_FILT_COEFF == 10 but only ever touched indices 0/1).
  double filCoefA[FIL_N_STEPS];
  double filCoefB[FIL_N_STEPS];
  std::vector<std::vector<double>> unfilteredIn;
  std::vector<std::vector<double>> unfilteredOut;

  // Controller synchronization delay.
  std::condition_variable delayCondition;
  double delayWindowSize{5.0};
  double delayMaxPerWindow{0.25};
  double delayMaxPerStep{0.025};
  std::chrono::steady_clock::time_point delayWindowStart;
  double delayInWindow{0.0};
  atlas_msgs::msg::SynchronizationStatistics delayStatistics;

  // Controller statistics.
  std::vector<double> atlasCommandAgeBuffer;
  std::vector<double> atlasCommandAgeDelta2Buffer;
  unsigned int atlasCommandAgeBufferIndex{0};
  double atlasCommandAgeBufferDuration{1.0};
  double atlasCommandAgeMean{0.0};
  double atlasCommandAgeVariance{0.0};
  double atlasCommandAge{0.0};
  rclcpp::Time atlasCommandStamp;
  double statsUpdateRate{1000.0};
  double lastControllerStatisticsTime{0.0};

  std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread rosSpinThread;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pubLFootContact;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pubRFootContact;
  rclcpp::Publisher<atlas_msgs::msg::SynchronizationStatistics>::SharedPtr
    pubDelayStatistics;
  rclcpp::Publisher<atlas_msgs::msg::ControllerStatistics>::SharedPtr
    pubControllerStatistics;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu;
  rclcpp::Publisher<atlas_msgs::msg::ForceTorqueSensors>::SharedPtr
    pubForceTorqueSensors;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointStates;
  rclcpp::Publisher<atlas_msgs::msg::AtlasState>::SharedPtr pubAtlasState;
  rclcpp::Publisher<atlas_msgs::msg::AtlasSimInterfaceState>::SharedPtr pubASIState;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subTic;
  rclcpp::Subscription<atlas_msgs::msg::Test>::SharedPtr subTest;
  rclcpp::Subscription<atlas_msgs::msg::AtlasCommand>::SharedPtr subAtlasCommand;
  rclcpp::Subscription<osrf_msgs::msg::JointCommands>::SharedPtr subJointCommands;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subAtlasControlMode;
  rclcpp::Subscription<atlas_msgs::msg::AtlasSimInterfaceCommand>::SharedPtr
    subASICommand;

  rclcpp::Service<atlas_msgs::srv::AtlasFilters>::SharedPtr atlasFiltersService;
  rclcpp::Service<atlas_msgs::srv::ResetControls>::SharedPtr resetControlsService;
  rclcpp::Service<atlas_msgs::srv::SetJointDamping>::SharedPtr setJointDampingService;
  rclcpp::Service<atlas_msgs::srv::GetJointDamping>::SharedPtr getJointDampingService;
};
}  // namespace drcsim_gazebo_ros_plugins
#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__ATLASPLUGIN_HPP_
