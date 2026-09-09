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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__VRCPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__VRCPLUGIN_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>

#include <rclcpp/rclcpp.hpp>

#include <atlas_msgs/msg/atlas_behavior_step_data.hpp>
#include <atlas_msgs/msg/atlas_command.hpp>
#include <atlas_msgs/msg/atlas_sim_interface_command.hpp>
#include <atlas_msgs/msg/atlas_sim_interface_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief "VRC cheats" world plugin: pin/teleport the robot, planar cmd_vel
/// teleop while pinned, vehicle enter/exit, fire-hose grab, and a fake
/// AtlasSimInterface (BDI behavior library) that turns STAND/FREEZE/WALK/
/// STEP commands into pin/PID/teleport tricks. Ported from the original
/// Gazebo-Classic `VRCPlugin` (drcsim, ROS 1 era, a `WorldPlugin`) to
/// gz-sim's System interface for Gazebo Harmonic, and from roscpp to rclcpp.
///
/// This is the largest and most speculative port in this migration -- unlike
/// every other plugin ported so far, several of its core mechanisms rely on
/// Gazebo-Classic APIs with **no direct gz-sim runtime-mutation equivalent**.
/// Each gap below was individually researched against the installed
/// gz-sim/gz-physics/sdformat headers (and, where headers alone were
/// ambiguous, the shipped `.so` plugins' symbol tables) before deciding on a
/// substitute -- these aren't guesses:
///
/// - **Per-link runtime gravity toggle** (Classic's `Link::SetGravityMode()`
///   / `Model::SetGravityMode()`): `gz::sim::components::GravityEnabled`
///   exists, but evidence from the installed physics/dartsim plugin
///   binaries (the SDF getter/native-engine-call pairing they exhibit) says
///   it is only consulted when a link is first constructed, not read every
///   step -- mutating it on a live link is very unlikely to do anything.
///   Substitute: track a set of "gravity disabled" link entities and apply
///   an explicit per-step counter-force each `PreUpdate`
///   (`Link::AddWorldForce(ecm, -mass * gravity)`), the same
///   "reimplement the missing mutation as an explicit per-step force"
///   pattern already used for joint springs in `IRobotHandPlugin`/
///   `RobotiqHandPlugin`. `AddWorldForce` itself is documented as applying
///   for one step only (it feeds `components::ExternalWorldWrenchCmd`,
///   which the physics system consumes and clears every update), so this
///   must be (and is) re-applied every `PreUpdate` for as long as gravity
///   should stay disabled for that link -- exactly like a PD force.
/// - **Per-model runtime static/kinematic toggle** (Classic's
///   `Link::SetLinkStatic()`, used only as a simbody/dart-specific
///   alternative pinning path in the original): `components::Static` exists
///   but the same evidence pattern (SDF getter + one-shot native engine
///   call, e.g. `dart::dynamics::Skeleton::setMobile()`) says it's also
///   construction-time only. Not needed here anyway: the *primary* pinning
///   path in the original (a real dynamically-created joint, the
///   ode/bullet branch) is what this port always uses -- gz-sim's
///   joint-creation mechanisms (below) work regardless of which physics
///   engine plugin is loaded, unlike Classic's per-engine branching, so the
///   simbody/dart-specific fallback branch is dropped outright as
///   unreachable-by-construction rather than ported.
/// - **Per-link/collision runtime "collide mode" toggle** (Classic's
///   `Link::SetCollideMode("all"|"none"|"fixed")`, used by `SetFeetCollide`
///   during pin/unpin so floating/teleporting feet don't generate spurious
///   contacts): no component-flag equivalent exists at all in gz-sim (only
///   entity-lifecycle removal/recreation of the whole collision entity is
///   evidence-supported, which is heavy for what is a cosmetic anti-glitch
///   measure, not core behavior). Dropped: `SetFeetCollide()` is kept as a
///   method (for interface/call-site fidelity) but is now a documented
///   no-op. The original's `AddJoint(..., _disableCollision)` parameter is
///   dropped from the signature entirely -- it was `false` at every call
///   site in the original anyway, so nothing is lost by removing it.
/// - **Runtime model spawning from an SDF/URDF string** (Classic's
///   `World::InsertModelString()`, used to spawn the Atlas robot if it
///   wasn't already included in the world file): gz-sim's
///   `SdfEntityCreator` does this directly against the ECM from inside a
///   System plugin -- `sdf::Root::LoadSdfString()` (which auto-detects and
///   converts URDF, no separate conversion step) followed by
///   `SdfEntityCreator::CreateEntities(root.Model())` + `SetParent()`. This
///   is exactly the mechanism gz-sim's own `UserCommands` system uses
///   behind the `/world/<world>/create` service; being already inside the
///   server process, this port calls it directly instead of round-tripping
///   through that transport service. Unlike Classic's async spawn-then-poll
///   (`SPAWN_QUEUED` -> `CheckGetModel()`), `CreateEntities()` is
///   synchronous, so the `SPAWN_QUEUED` startup-sequence state is now
///   unreachable and `CheckGetModel()` isn't needed; both are kept as
///   enum/method stubs for fidelity with the original's state machine.
/// - **Runtime joint creation between two existing links** (Classic's
///   `PhysicsEngine::CreateJoint()`-based `AddJoint()` helper, used for
///   pinning, the robot-grabs-fire-hose cheat, and the vehicle-seat weld):
///   both the pin-to-world case and the cross-model case now use the same
///   mechanism, `gz::sim::components::DetachableJoint`
///   (`{parentLink, childLink, jointType="fixed"}`) on a fresh, otherwise
///   empty entity. Confirmed via the physics system's own binary symbols
///   that this needs no other components, works across entirely different
///   models (both links are raw resolved `Entity` ids, no model-scoping
///   check), and needs no extra world-level system plugin loaded -- the
///   stock `gz-sim-physics-system` handles it directly, the same as
///   gz-sim's own (separate, topic-triggered)
///   `gz-sim-detachable-joint-system` does internally. Removal in all
///   cases is uniform: `_ecm.RequestRemoveEntity(jointEntity)`.
///   - *Pin to world* used to be a real SDF `fixed` joint with parent name
///     `"world"`, built via `sdf::Joint` setters and turned into a live
///     entity with `SdfEntityCreator::CreateEntities(&jointSdf, true)` --
///     that creates and holds correctly (confirmed: the pinned robot does
///     visibly stay in place), but its **removal silently doesn't detach
///     the physics constraint**: `RequestRemoveEntity` erases the ECS
///     entity, but gz-physics's dartsim plugin logs `No joint named
///     [<name>_world_pin_joint] for modelID [N]` and leaves the actual
///     dartsim-level weld in place forever -- confirmed the hard way, by
///     watching a real launch where Atlas's pelvis never moved again after
///     its very first pin, no matter what commanded every other joint.
///     Near-certainly because "weld to world" is normally a Skeleton-
///     construction-time operation in dartsim, and joints added this way
///     *after* the model already exists don't get registered as a normal,
///     later-removable joint object. This is likely a genuine gz-physics/
///     dartsim gap for *dynamically added* world joints, not something
///     fixable from the SDF-authoring side. Fixed by sidestepping it
///     entirely: `EnsureWorldPinAnchor()` looks up the `link` link of the
///     world file's own `ground_plane` model -- declared in the world SDF
///     and loaded through the normal world-loading path, so it is
///     unambiguously static from tick zero, unlike a first attempt that
///     spawned a brand-new "static" model at runtime via
///     `SdfEntityCreator`, whose `Static` component gz-physics evidently
///     does not reliably honor for entities created after the simulation
///     has already started (observed as the pinned robot flying/
///     teleporting/spinning uncontrollably, consistent with the "anchor"
///     actually being a free ~1 kg dynamic body rigidly welded to the
///     pelvis) -- and pin-to-world now welds to *that real link* via
///     the exact same `DetachableJoint` mechanism already proven reliable
///     (creation *and* removal) for the cross-model welds below.
///   - *Fire hose <-> standpipe screw-thread docking*: the original creates
///     a real Classic "screw" joint with a settable thread pitch so the
///     connection can be reversed by "unscrewing" it (reading the joint's
///     live angle). gz-physics/dartsim genuinely implements screw joints,
///     but only reachable via the same intra-model `SdfEntityCreator`
///     joint path used for the world-pin case above -- and this connection
///     is inherently cross-model (standpipe model <-> fire hose model), so
///     that path doesn't apply, and `DetachableJoint` only supports
///     `"fixed"`. **Dropped**: docking is now a fixed weld (via
///     `DetachableJoint`, same as the other cross-model welds), auto-created
///     by the exact same proximity/alignment check as the original
///     (`CheckThreadStart()`, unchanged -- needs no new API). The
///     "unscrew to disconnect" gameplay mechanic has no substitute and is
///     not ported; disconnecting the hose now requires an external action
///     rather than a physical unscrewing motion.
/// - **World pause / "disable physics for a moment while I teleport
///   something" transaction** (Classic's `World::SetPaused()` +
///   `World::EnablePhysicsEngine()`, wrapped around most of the original's
///   pose-mutating actions): `gz::sim::World` carries neither method, and
///   the only real substitute -- emitting `gz::sim::events::Pause` through
///   the `EventManager` -- only takes effect starting the *next* iteration,
///   not instantaneously, since every system (including this one) still
///   runs its `PreUpdate` even while paused. This port sidesteps needing
///   it at all: every ROS callback that needs to touch the ECM
///   (`SetRobotPose`, `SetRobotMode`, `SetFakeASIC`, `RobotEnterCar`,
///   `RobotExitCar`, `RobotGrabFireHose`, `RobotReleaseLink`) runs on this
///   plugin's own rclcpp executor thread, never the sim thread -- so, same
///   as every other ROS-coupled plugin in this migration, these callbacks
///   only ever record a pending request under `controlMutex`; the actual
///   `EntityComponentManager` mutations happen exclusively inside
///   `PreUpdate()` on the sim thread, one at a time, with no concurrent
///   physics step able to interleave with them. That was always true of
///   every plugin so far (e.g. `SandiaHandPlugin`'s `SetJointCommands` only
///   copies into a cached struct) -- it's just that VRCPlugin's original
///   directly mutated Gazebo Classic physics objects *from inside* its ROS
///   callbacks (safe there only because Classic's physics objects have
///   their own internal locking, and *that's* what the pause/unpause
///   dance was actually protecting against: a ROS callback thread and the
///   physics thread touching the same object at once). Once every mutation
///   is confined to the sim thread the way this port confines it, that
///   race can't happen in the first place, and the pause/unpause
///   bracketing the original built around it becomes unnecessary rather
///   than merely hard to port.
/// - **Downward raycast "what's the ground height below the robot"**
///   (Classic's `Entity::GetNearestEntityBelow()`, used only by the rarely
///   invoked `"harnessed"` startup mode): `gz::sim::components::RaycastData`
///   is a real, physics-system-backed mechanism, but its result carries no
///   entity-identity field (point/fraction/normal only) and it wasn't clear
///   from research which entity's frame the ray coordinates are resolved
///   against. Given `"harnessed"` mode is already excluded from the
///   original's own "available modes" help text and the original *itself*
///   already falls back to a flat `z = 0` ground assumption whenever the
///   raycast fails to find anything, this port always takes that same
///   fallback rather than attempting the raycast.
/// - **Instant "teleport all joints to this pose" snap**
///   (Classic's `Model::SetJointPositions()`, used by `SetPIDStand`/
///   `SetSeatingConfiguration`/`SetStandingConfiguration` for an immediate
///   visual snap to a stand/seated pose, in addition to publishing an
///   `AtlasCommand` for the *real* joint-space PID controller -- which
///   lives in `AtlasPlugin`, not yet ported in this migration -- to
///   converge to): gz-sim's `Joint` wrapper has a position *reader*
///   (`Position()`) but no direct position-teleport *writer*. Dropped: only
///   the `AtlasCommand` publish is kept, so these stand/seat poses will
///   only actually take effect once `AtlasPlugin` exists downstream to
///   consume it and PID the robot there -- consistent with the fact that
///   this plugin's fake-ASIC/stand machinery was already designed to hand
///   off to `AtlasPlugin` via ROS topics, not to move the robot itself.
class VRCPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  VRCPlugin();
  ~VRCPlugin() override;

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

  //////////////////////////////////////////////////////////////////////////
  // List of available actions (ROS callbacks -- record a pending request
  // only; see the class-level design note on why these never touch the ECM
  // directly).
  //////////////////////////////////////////////////////////////////////////
  void SetRobotCmdVel(const geometry_msgs::msg::Twist & _cmd, double _duration);
  void SetRobotCmdVelTopic(const geometry_msgs::msg::Twist::SharedPtr _cmd);
  void SetRobotPose(const geometry_msgs::msg::Pose::SharedPtr _cmd);
  void SetRobotConfiguration(const sensor_msgs::msg::JointState::SharedPtr _cmd);
  void SetRobotModeTopic(const std_msgs::msg::String::SharedPtr _str);
  void SetRobotMode(const std::string & _str);
  void SetFakeASIC(
    const atlas_msgs::msg::AtlasSimInterfaceCommand::SharedPtr _asic);
  void RobotEnterCar(const geometry_msgs::msg::Pose::SharedPtr _pose);
  void RobotExitCar(const geometry_msgs::msg::Pose::SharedPtr _pose);
  void RobotGrabFireHose(const geometry_msgs::msg::Pose::SharedPtr _cmd);
  void RobotReleaseLink(const geometry_msgs::msg::Pose::SharedPtr _cmd);

private:
  //////////////////////////////////////////////////////////////////////////
  // Generic tools for manipulating models
  //////////////////////////////////////////////////////////////////////////
  gz::sim::Entity AddJoint(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
    gz::sim::Entity _modelEntity, gz::sim::Entity _link1, gz::sim::Entity _link2);
  /// \brief Look up (once) or return the cached link entity of the world
  /// file's own `ground_plane` model, used as the `DetachableJoint` anchor
  /// for "pin to world" -- see the class-level design note on why this
  /// replaced both a directly-world-parented SDF fixed joint and a
  /// runtime-spawned "static" anchor model.
  gz::sim::Entity EnsureWorldPinAnchor(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr);
  void RemoveJoint(
    gz::sim::EntityComponentManager & _ecm, gz::sim::Entity & _joint);
  void Teleport(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
    gz::sim::Entity _pinLink, gz::sim::Entity & _pinJoint,
    const gz::math::Pose3d & _pose);
  void SetModelGravityMode(
    gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _modelEntity,
    bool _enabled);
  void SetLinkGravityMode(gz::sim::Entity _linkEntity, bool _enabled);
  void ApplyGravityCompensation(gz::sim::EntityComponentManager & _ecm);

  /// \brief gz-sim only exposes an instant world-pose teleport command at
  /// the whole-*model* level (`Model::SetWorldPoseCmd()`) -- there is no
  /// per-link equivalent (see the class-level design note). This computes
  /// the model pose that places a given one of its links at the desired
  /// world pose (using that link's currently-solved, rigid offset from the
  /// model), then issues that as the model's pose command -- the same
  /// operation Classic's `Model::SetLinkWorldPose()` performed internally.
  static gz::math::Pose3d ModelPoseForLinkWorldPose(
    const gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _modelEntity,
    gz::sim::Entity _linkEntity, const gz::math::Pose3d & _desiredLinkPose);
  static void SetLinkWorldPose(
    gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _modelEntity,
    gz::sim::Entity _linkEntity, const gz::math::Pose3d & _pose);

  void LoadRobotROSAPI();
  void LoadVRCROSAPI();
  void CheckThreadStart(gz::sim::EntityComponentManager & _ecm);
  void DeferredLoad(gz::sim::EntityComponentManager & _ecm);
  void PinAtlas(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
    bool _withGravity);
  void UnpinAtlas(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr);
  void SetFeetCollide(const std::string & _mode);
  void StepDataToTwist(
    gz::sim::EntityComponentManager & _ecm,
    const atlas_msgs::msg::AtlasBehaviorStepData & _step, double _dt,
    geometry_msgs::msg::Twist & _twist);
  static gz::math::Pose3d ToGzPose(const geometry_msgs::msg::Pose & _pose);
  void UpdateStates(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr);

  // Actually perform whichever ROS-requested action is pending; called
  // from UpdateStates() on the sim thread only.
  void ProcessPendingActions(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr);
  void DoSetRobotPose(
    gz::sim::EntityComponentManager & _ecm, const gz::math::Pose3d & _pose);
  void DoSetRobotMode(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
    const std::string & _str);
  void DoSetFakeASIC(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
    const atlas_msgs::msg::AtlasSimInterfaceCommand & _asic);
  void DoRobotEnterCar(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
    const gz::math::Pose3d & _pose);
  void DoRobotExitCar(
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr,
    const gz::math::Pose3d & _pose);
  void DoRobotGrabFireHose(
    gz::sim::EntityComponentManager & _ecm, const gz::math::Pose3d & _cmd);
  void DoRobotReleaseLink(gz::sim::EntityComponentManager & _ecm);

  static double FirstOrZero(const std::optional<std::vector<double>> & _values);

  //////////////////////////////////////////////////////////////////////////
  // Atlas properties and states
  //////////////////////////////////////////////////////////////////////////
  class Robot
  {
public:
    Robot();

    void InsertModel(
      gz::sim::World & _world, gz::sim::EntityComponentManager & _ecm,
      gz::sim::EventManager & _eventMgr,
      const sdf::ElementPtr & _pluginSdf,
      const rclcpp::Node::SharedPtr & _rosNode);
    bool CheckGetModel(gz::sim::World & _world, gz::sim::EntityComponentManager & _ecm);

    gz::math::Pose3d spawnPose;
    gz::sim::Entity modelEntity{gz::sim::kNullEntity};
    gz::sim::Entity pinLinkEntity{gz::sim::kNullEntity};
    gz::sim::Entity pinJointEntity{gz::sim::kNullEntity};

    std::string modelName;
    std::string pinLinkName;

    /// \brief keep initial pose of robot to prevent z-drifting when
    /// teleporting the robot.
    gz::math::Pose3d initialPose;

    /// \brief Pose of robot relative to vehicle.
    gz::math::Pose3d vehicleRelPose;

    /// \brief  At t-t0 < startupStandPrepDuration seconds, pinned.
    /// At t - t0 = startupStandPrepDurationl, start StandPrep mode.
    double startupStandPrepDuration;

    /// \brief at t - t0 = startupNominal, start Nominal mode.
    double startupNominal;

    /// \brief at t - t0 = startupStand, start Stand mode.
    double startupStand;

    enum BDIStandSequence
    {
      BS_NONE = 0,
      BS_PID_PINNED = 1,
      BS_STAND_PREP_PINNED = 2,
      BS_STAND_PREP = 3,
      BS_INITIALIZED = 4
    };
    int bdiStandSequence;

    enum PinnedSequence
    {
      PS_NONE = 0,
      PS_PINNED = 1,
      PS_INITIALIZED = 2
    };
    int pinnedSequence;

    std::chrono::steady_clock::duration startupBDIStandStartTime{0};

    std::string startupMode;

    enum StartupSequence
    {
      NONE = 0,
      SPAWN_QUEUED = 1,
      SPAWN_SUCCESS = 2,
      INIT_MODEL_SUCCESS = 3,
      INITIALIZED = 4
    };
    int startupSequence;

    double startupHarnessDuration;
    bool startInVehicle{false};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subTrajectory;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subPose;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subConfiguration;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subMode;
    rclcpp::Subscription<atlas_msgs::msg::AtlasSimInterfaceCommand>::SharedPtr
      subFakeASIC;
    rclcpp::Publisher<atlas_msgs::msg::AtlasSimInterfaceState>::SharedPtr pubFakeASIS;

    int currentBehavior;
    int currentStepIndex;
    int lastStepIndex;
  } atlas;

  //////////////////////////////////////////////////////////////////////////
  // DRC Vehicle properties and states
  //////////////////////////////////////////////////////////////////////////
  class Vehicle
  {
public:
    void Load(
      gz::sim::World & _world, gz::sim::EntityComponentManager & _ecm,
      const sdf::ElementPtr & _pluginSdf);

    gz::sim::Entity modelEntity{gz::sim::kNullEntity};
    gz::math::Pose3d initialPose;
    gz::sim::Entity seatLinkEntity{gz::sim::kNullEntity};

    bool isInitialized{false};
  } drcVehicle;

  //////////////////////////////////////////////////////////////////////////
  // DRC Fire Hose (and Standpipe)
  //////////////////////////////////////////////////////////////////////////
  class FireHose
  {
public:
    void Load(
      gz::sim::World & _world, gz::sim::EntityComponentManager & _ecm,
      const sdf::ElementPtr & _pluginSdf);

    gz::sim::Entity fireHoseModelEntity{gz::sim::kNullEntity};
    gz::sim::Entity standpipeModelEntity{gz::sim::kNullEntity};
    gz::sim::Entity valveModelEntity{gz::sim::kNullEntity};
    gz::sim::Entity valveJointEntity{gz::sim::kNullEntity};

    /// \brief entity of the cross-model weld between the fire hose and the
    /// standpipe once docked (see the class-level design note: a fixed
    /// weld, not a real screw joint).
    gz::sim::Entity screwJointEntity{gz::sim::kNullEntity};
    double threadPitch{0.0};

    gz::sim::Entity couplingLinkEntity{gz::sim::kNullEntity};
    gz::sim::Entity spoutLinkEntity{gz::sim::kNullEntity};
    gz::math::Pose3d couplingRelativePose;
    gz::math::Pose3d initialFireHosePose;

    bool isInitialized{false};
  } drcFireHose;

  //////////////////////////////////////////////////////////////////////////
  // Robot Joint Controller
  //////////////////////////////////////////////////////////////////////////
  class AtlasCommandController
  {
public:
    AtlasCommandController();

    void InitModel(
      gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _modelEntity,
      const rclcpp::Node::SharedPtr & _rosNode);

    std::string FindJoint(
      gz::sim::EntityComponentManager & _ecm, const std::string & _st1,
      const std::string & _st2);
    std::string FindJoint(
      gz::sim::EntityComponentManager & _ecm, const std::string & _st1,
      const std::string & _st2, const std::string & _st3);

    void GetJointStates(const sensor_msgs::msg::JointState::SharedPtr _js);

    void SetPIDStand();
    void SetBDIFREEZE();
    void SetBDIStandPrep();
    void SetBDIStand();
    void SetSeatingConfiguration();
    void SetStandingConfiguration();

    gz::sim::Entity modelEntity{gz::sim::kNullEntity};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subJointStates;
    rclcpp::Publisher<atlas_msgs::msg::AtlasCommand>::SharedPtr pubAtlasCommand;
    rclcpp::Publisher<atlas_msgs::msg::AtlasSimInterfaceCommand>::SharedPtr
      pubAtlasSimInterfaceCommand;

    rclcpp::Node::WeakPtr rosNode;

    atlas_msgs::msg::AtlasCommand ac;

    sensor_msgs::msg::JointState::SharedPtr js;
    bool jsValid{false};

    std::vector<std::string> jointNames;

    int atlasVersion;
    int atlasSubVersion;
  } atlasCommandController;

  //////////////////////////////////////////////////////////////////////////
  // Private variables
  //////////////////////////////////////////////////////////////////////////
  gz::sim::World world{gz::sim::kNullEntity};
  /// \brief Stashed from Configure() -- EventManager isn't passed to
  /// PreUpdate(), but this plugin needs it there too (pin-to-world joint
  /// creation via SdfEntityCreator, and emitting events::Pause). Both the
  /// ECM and the EventManager are long-lived objects owned by the
  /// SimulationRunner and passed by reference on every call across a
  /// System's lifetime, so holding this pointer across calls is safe --
  /// the same assumption every other plugin in this migration already
  /// makes about the ECM reference it re-receives each call, just made
  /// explicit here because this is the first plugin needing it beyond
  /// Configure().
  gz::sim::EventManager * eventMgr{nullptr};
  /// \brief A mutable clone of the plugin's SDF config, taken in Configure()
  /// -- Configure() itself only receives a `const shared_ptr<const
  /// Element>`, but every nested-block accessor (`GetElement()`) that this
  /// plugin's `<atlas>`/`<drc_vehicle>`/`<drc_fire_hose>` blocks need is
  /// non-const in this sdformat version (only leaf `Get<T>()` reads are
  /// const), so a const pointer can't be handed to `Vehicle::Load()`/
  /// `FireHose::Load()`/`Robot::InsertModel()` directly.
  sdf::ElementPtr sdfConfig;
  bool initialized{false};
  bool validConfig{false};

  bool warpRobotWithCmdVel{false};
  std::chrono::steady_clock::duration warpRobotStopTime{0};
  std::chrono::steady_clock::duration lastUpdateTime{0};
  /// \brief Sim time as of the start of the most recent UpdateStates() call,
  /// kept under controlMutex so SetRobotCmdVel() can read "now" whether it
  /// was called from the sim thread (internally, e.g. from DoSetFakeASIC)
  /// or from the ROS executor thread (SetRobotCmdVelTopic).
  std::chrono::steady_clock::duration currentSimTime{0};
  geometry_msgs::msg::Twist robotCmdVel;

  gz::sim::Entity vehicleRobotJoint{gz::sim::kNullEntity};
  gz::sim::Entity grabJoint{gz::sim::kNullEntity};
  /// \brief Link entity `EnsureWorldPinAnchor()` resolves (ground_plane's
  /// `link`) on first use; `gz::sim::kNullEntity` until then.
  gz::sim::Entity worldPinAnchorLinkEntity{gz::sim::kNullEntity};

  /// \brief links currently receiving an explicit counter-gravity force
  /// each step (see the class-level design note on gravity compensation).
  std::set<gz::sim::Entity> gravityDisabledLinks;

  bool cheatsEnabled{false};
  double cmdVelTopicTimeout{0.1};

  std::mutex controlMutex;

  // Pending ROS-requested actions, drained on the sim thread in
  // ProcessPendingActions() -- see the class-level design note on why ROS
  // callbacks never touch the ECM directly.
  bool pendingSetRobotPose{false};
  geometry_msgs::msg::Pose pendingPose;
  bool pendingSetRobotMode{false};
  std::string pendingMode;
  bool pendingFakeASIC{false};
  atlas_msgs::msg::AtlasSimInterfaceCommand pendingASIC;
  bool pendingEnterCar{false};
  geometry_msgs::msg::Pose pendingEnterCarPose;
  bool pendingExitCar{false};
  geometry_msgs::msg::Pose pendingExitCarPose;
  bool pendingGrabFireHose{false};
  geometry_msgs::msg::Pose pendingGrabPose;
  bool pendingReleaseLink{false};

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread rosSpinThread;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subRobotGrab;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subRobotRelease;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subRobotEnterCar;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subRobotExitCar;
};
}  // namespace drcsim_gazebo_ros_plugins
#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__VRCPLUGIN_HPP_
