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
#ifndef DRCSIM_GAZEBO_ROS_PLUGINS__VRCSCORINGPLUGIN_HPP_
#define DRCSIM_GAZEBO_ROS_PLUGINS__VRCSCORINGPLUGIN_HPP_

#include <chrono>
#include <filesystem>
#include <fstream>
#include <list>
#include <memory>
#include <optional>
#include <string>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>

#include <atlas_msgs/msg/vrc_score.hpp>

namespace drcsim_gazebo_ros_plugins
{
/// \brief Implements the VRC/qualifier scoring algorithms (gate crossings,
/// vehicle entry, drill-in-bin, fire hose docking/connection/valve) and
/// writes a running score log plus an `atlas/vrc_score`-style ROS topic
/// (`vrc_score`, unchanged name). Ported from the original Gazebo-Classic
/// `VRCScoringPlugin` (a `WorldPlugin`, ~1300 lines across header and
/// source) to gz-sim's System interface for Gazebo Harmonic, and from
/// roscpp to rclcpp.
///
/// Design notes vs. the original:
/// - **No thread, no mutex.** The original defers most of its setup
///   (finding the `atlas` model, opening the score file, standing up ROS)
///   to a background thread that polls once a second and blocks until
///   `atlas` appears, then connects to `WorldUpdateBegin`. That defensive
///   wait-for-atlas behavior is preserved here (see the `atlasReady`
///   member below), but implemented by retrying the lookup once per
///   `PreUpdate` tick instead of a dedicated sleeping thread -- there are
///   no incoming ROS callbacks in this plugin (it only ever publishes),
///   so unlike `VRCPlugin`/`AtlasPlugin` there is no cross-thread ECM
///   access to guard against and no pending-action queue is needed.
/// - **`ISystemPreUpdate` is used for exactly one thing**: the one-time
///   transition from "atlas not found yet" to "atlas found", which needs
///   a mutable `EntityComponentManager` to call `Link::EnableVelocityChecks`
///   (required before `Link::WorldLinearVelocity` will return a value, used
///   by `CheckFall`). Every other tick of real work happens in
///   `ISystemPostUpdate`, against a `const EntityComponentManager`, since
///   none of the scoring checks need to mutate simulation state -- this
///   plugin only ever reads poses/velocities/joint positions and publishes.
/// - **Bounding boxes are computed by hand from box-shaped collision
///   geometry** (`gz::sim::components::CollisionElement`, the same
///   component `VRCPlugin::CheckThreadStart` reads for the fire hose
///   coupler's cylinder) rather than via `Link::WorldAxisAlignedBox` (which
///   requires `Link::EnableBoundingBoxChecks`, a newer API not verified
///   available in the Harmonic-era gz-sim release this workspace targets).
///   Every real caller here (the VRC "gate" models, and the qualifier-2
///   "bin" model) uses box collisions exclusively, so this covers all
///   actual usage; a non-box collision is simply skipped, matching the
///   original's implicit assumption that these particular models are
///   axis-aligned boxes.
/// - **Fire hose "aligned" detection**: the original checks
///   `standpipe->GetChildJointsLinks()` for a non-empty list, which is
///   true only once `VRCPlugin` has created its (Classic) screw joint
///   between the standpipe spout and the hose coupler. `VRCPlugin`'s own
///   gz-sim port replaces that screw joint with a
///   `gz::sim::components::DetachableJoint` (see `VRCPlugin.hpp`'s design
///   notes for why: no real cross-model screw-joint path in gz-sim). This
///   plugin detects the same event by scanning all `DetachableJoint`
///   components in the ECM for one whose `parentLink` is the standpipe
///   link -- functionally identical to the original's check, and it
///   inherits the same downstream consequence noted in `VRCPlugin`: once
///   made, the connection cannot be "unscrewed", so `CheckHoseAligned`
///   will never observe a transition back to unaligned in practice.
/// - **Vehicle seat link name**: the original looks up a link named
///   `"polaris_ranger_ev::chassis"` -- Classic's double-colon nested-model
///   deep-name lookup syntax, which has no equivalent in gz-sim's ECS
///   (`Model::LinkByName` only searches immediate child links). This port
///   instead looks up model `"drc_vehicle"` / link `"chassis"`, matching
///   the naming `VRCPlugin`'s own gz-sim port already assumes for the same
///   vehicle (see `VRCPlugin::Vehicle::Load`'s `seatLinkName` default) --
///   both plugins must agree on how to find the same vehicle in the same
///   world, and only one of those two naming schemes can work under
///   gz-sim's ECS.
/// - `boost::filesystem`/`boost::lexical_cast`/`boost::algorithm::string`
///   are replaced with `std::filesystem` and small hand-rolled string
///   parsing (the gate-name format `"gate_<N>"`/`"vehiclegate_<N>"` is
///   fixed and simple enough not to need a string-split library).
/// - The original's `PubMultiQueue`/`PubQueue` machinery existed to avoid
///   blocking Gazebo Classic's single update thread on a ROS 1 publish
///   call under contention; rclcpp publishers are safe to call directly
///   from any thread and don't block the caller, so that machinery is
///   dropped entirely -- `pubScore->publish(...)` is called straight from
///   `PostUpdate`.
class VRCScoringPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  VRCScoringPlugin();

  ~VRCScoringPlugin() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  /// \brief The worlds that we might be scoring; each one can be
  /// slightly different.
  enum class WorldType
  {
    QUAL_1,
    QUAL_2,
    QUAL_3,
    QUAL_4,
    VRC_1,
    VRC_2,
    VRC_3
  };

  /// \brief Data about a gate.
  class Gate
  {
public:
    /// \brief Types of gates that we know about.
    enum class GateType
    {
      PEDESTRIAN,
      VEHICLE
    };

    Gate(
      const std::string & _name, GateType _type, unsigned int _number,
      const gz::math::Pose3d & _pose, double _width)
    : name(_name), type(_type), number(_number), pose(_pose), width(_width)
    {}

    /// \brief Less-than operator to allow sorting of a list of gates by
    /// number.
    bool operator<(const Gate & _other) const
    {
      return this->number < _other.number;
    }

    /// \brief Name of the gate.
    std::string name;

    /// \brief The type of the gate.
    GateType type;

    /// \brief Number of the gate.
    unsigned int number;

    /// \brief Pose of the center of the gate.
    gz::math::Pose3d pose;

    /// \brief Width of the gate.
    double width;

    /// \brief Have we passed through this gate yet?
    bool passed{false};
  };

  /// \brief Try to find the `atlas` model and its `head` link. On success,
  /// enables velocity checks on the head link (needed by `CheckFall`).
  /// \return true once atlas has been found.
  bool FindAtlas(gz::sim::EntityComponentManager & _ecm);

  /// \brief Stand up the ROS node and publisher. Called once, right after
  /// `FindAtlas` first succeeds. (The score file itself is opened
  /// unconditionally in `Configure`, matching the original, which opens
  /// it synchronously in `Load()` before deferring only the atlas lookup
  /// and ROS setup to a background thread.)
  void CompleteDeferredLoad();

  /// \brief Compute the world-frame axis-aligned bounding box of a
  /// box-shaped collision. Returns nullopt for non-box geometry or a
  /// missing collision.
  static std::optional<gz::math::AxisAlignedBox> CollisionWorldBox(
    const gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _collisionEntity);

  /// \brief Check the next gate to see if we've passed it.
  /// \param _msg Log messages (e.g., "passed gate") will be appended here.
  /// \return true if the next gate was passed, false otherwise.
  bool CheckNextGate(const gz::sim::EntityComponentManager & _ecm, std::string & _msg);

  /// \brief Check whether we've fallen.
  /// \param _simTime Current simulation time.
  /// \param _msg Log messages will be appended here.
  /// \return true if we've fallen, false otherwise.
  bool CheckFall(
    const gz::sim::EntityComponentManager & _ecm,
    const std::chrono::steady_clock::duration & _simTime, std::string & _msg);

  /// \brief Check whether Atlas is in the vehicle.
  bool CheckAtlasInVehicle(const gz::sim::EntityComponentManager & _ecm, std::string & _msg);

  /// \brief Check whether the drill is in the bin.
  bool CheckDrillInBin(const gz::sim::EntityComponentManager & _ecm, std::string & _msg);

  /// \brief Check whether the hose is off the table.
  bool CheckHoseOffTable(const gz::sim::EntityComponentManager & _ecm, std::string & _msg);

  /// \brief Check whether the hose is aligned with the standpipe.
  bool CheckHoseAligned(const gz::sim::EntityComponentManager & _ecm, std::string & _msg);

  /// \brief Check whether the hose is connected to the standpipe.
  bool CheckHoseConnected(const gz::sim::EntityComponentManager & _ecm, std::string & _msg);

  /// \brief Check whether the valve is turned.
  bool CheckValveOpen(const gz::sim::EntityComponentManager & _ecm, std::string & _msg);

  /// \brief Start the clock, used in computing elapsed time for the run.
  void StartClock(
    const std::chrono::steady_clock::duration & _simTime,
    const std::chrono::system_clock::time_point & _wallTime, std::string & _msg);

  /// \brief Stop the clock, used in computing elapsed time for the run.
  void StopClock(
    const std::chrono::steady_clock::duration & _simTime,
    const std::chrono::system_clock::time_point & _wallTime, std::string & _msg);

  /// \brief Write intermediate score data.
  /// \param _force If true, write output; otherwise write output only if
  /// enough time has passed since the last write.
  void WriteScore(
    const std::chrono::steady_clock::duration & _simTime,
    const std::chrono::system_clock::time_point & _wallTime,
    const std::string & _msg, bool _force);

  /// \brief Find the gates in the world and store them in this->gates.
  bool FindGates(gz::sim::EntityComponentManager & _ecm);

  /// \brief Find stuff needed for scoring Qual 2.
  bool FindQual2Stuff(gz::sim::EntityComponentManager & _ecm);

  /// \brief Find stuff needed for scoring VRC 1.
  bool FindVRC1Stuff(gz::sim::EntityComponentManager & _ecm);

  /// \brief Find stuff needed for scoring VRC 3.
  bool FindVRC3Stuff(gz::sim::EntityComponentManager & _ecm);

  /// \brief Is the given robot pose "in" the given gate pose?
  /// \return If not "in" the gate, return 0; else return -1 if "before" the
  /// gate, 1 if "after" the gate.
  static int IsPoseInGate(
    const gz::math::Pose3d & _robotWorldPose, const gz::math::Pose3d & _gateWorldPose,
    double _gateWidth);

  /// \brief The world being scored.
  gz::sim::World world;

  /// \brief Which type of world we're scoring.
  WorldType worldType{WorldType::QUAL_1};

  /// \brief True once Configure() has successfully found everything the
  /// current worldType needs (other than atlas).
  bool validConfig{false};

  /// \brief True once atlas has been found and the score file/ROS
  /// publisher have been set up.
  bool atlasReady{false};

  /// \brief Entity of Atlas.
  gz::sim::Entity atlasEntity{gz::sim::kNullEntity};

  /// \brief Entity of Atlas's head link.
  gz::sim::Entity atlasHeadEntity{gz::sim::kNullEntity};

  /// \brief Entity of the drill. (Q2)
  gz::sim::Entity drillEntity{gz::sim::kNullEntity};

  /// \brief The bin that will receive the drill. (Q2)
  gz::math::AxisAlignedBox bin;

  /// \brief Entity of the vehicle. (V1)
  gz::sim::Entity vehicleEntity{gz::sim::kNullEntity};

  /// \brief Entity of the "seat" collision. (V1)
  gz::sim::Entity vehicleSeatCollision{gz::sim::kNullEntity};

  /// \brief Entity of the "seat_back" collision. (V1)
  gz::sim::Entity vehicleSeatBackCollision{gz::sim::kNullEntity};

  /// \brief Entity of the hose coupler link. (V3)
  gz::sim::Entity hoseCouplerEntity{gz::sim::kNullEntity};

  /// \brief Entity of the standpipe link. (V3)
  gz::sim::Entity standpipeEntity{gz::sim::kNullEntity};

  /// \brief Entity of the valve joint. (V3)
  gz::sim::Entity valveJointEntity{gz::sim::kNullEntity};

  /// \brief Whether the hose is currently aligned to the standpipe. (V3)
  bool isHoseAligned{false};

  /// \brief Whether the hose is currently connected to the standpipe. (V3)
  bool isHoseConnected{false};

  /// \brief Pose of the hose coupler at the time of initial alignment. (V3)
  gz::math::Pose3d hoseCouplerAlignedPose;

  /// \brief List of all the gates in the world. We assume that gates have
  /// the names: gate_1, gate_2, ..., gate_n.
  std::list<Gate> gates;

  /// \brief Which gate is expected next, expressed as an iterator into
  /// this->gates.
  std::list<Gate>::iterator nextGate;

  /// \brief Which side of the next gate we were the last time we checked.
  int nextGateSide{0};

  /// \brief The absolute wall time when the run started.
  std::chrono::system_clock::time_point runStartTimeWall;

  /// \brief Sim time at which Atlas passed through the first gate. Zero
  /// means "not yet set", matching the original's `common::Time::Zero`
  /// sentinel (with the same t=0 edge case that implies).
  std::chrono::steady_clock::duration startTimeSim{std::chrono::steady_clock::duration::zero()};

  /// \brief Wall time at which Atlas passed through the first gate.
  std::chrono::system_clock::time_point startTimeWall;

  /// \brief Sim time at which Atlas achieved the last checkpoint.
  std::chrono::steady_clock::duration stopTimeSim{std::chrono::steady_clock::duration::zero()};

  /// \brief Wall time at which Atlas achieved the last checkpoint.
  std::chrono::system_clock::time_point stopTimeWall;

  /// \brief The completion score, called 'C' in the VRC docs.
  int completionScore{0};

  /// \brief How much acceleration must be experienced at the robot's
  /// center of mass to be considering damaging.
  double fallAccelThreshold{1000.0};

  /// \brief How many big falls we've taken.
  int falls{0};

  /// \brief Name of the file that we're writing score data to.
  std::filesystem::path scoreFilePath;

  /// \brief The stream associated with scoreFilePath.
  std::ofstream scoreFileStream;

  /// \brief When we last wrote score data to disk.
  std::chrono::steady_clock::duration prevScoreTime{std::chrono::steady_clock::duration::zero()};

  /// \brief Last time that we detected a fall.
  std::chrono::steady_clock::duration prevFallTime{std::chrono::steady_clock::duration::zero()};

  /// \brief Last time that we calculated acceleration.
  std::chrono::steady_clock::duration prevVelTime{std::chrono::steady_clock::duration::zero()};

  /// \brief Velocity at last time we calculated acceleration.
  gz::math::Vector3d prevLinearVel{gz::math::Vector3d::Zero};

  /// \brief Most recently seen sim time, cached so the destructor can
  /// write a final score line (gz-sim System destructors get no
  /// EntityComponentManager/UpdateInfo).
  std::chrono::steady_clock::duration lastSimTime{std::chrono::steady_clock::duration::zero()};

  /// \brief ROS node.
  rclcpp::Node::SharedPtr rosNode;

  /// \brief Publisher of vrc_score.
  rclcpp::Publisher<atlas_msgs::msg::VRCScore>::SharedPtr pubScore;

  /// \brief Elapsed sim time after task completion when we stop counting
  /// falls. It's non-zero to avoid having people dive across the finish
  /// line.
  const std::chrono::steady_clock::duration postCompletionQuietTime{std::chrono::seconds(5)};
};
}  // namespace drcsim_gazebo_ros_plugins

#endif  // DRCSIM_GAZEBO_ROS_PLUGINS__VRCSCORINGPLUGIN_HPP_
