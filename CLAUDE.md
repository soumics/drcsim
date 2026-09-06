# drcsim → ROS 2 Jazzy / Gazebo Harmonic migration

## What this is

`src/drcsim` is a fork (`soumics/drcsim`) of OSRF's DRC Simulator, originally built for
catkin + ROS Fuerte/Groovy + Gazebo Classic. We are porting it to **ROS 2 Jazzy +
Gazebo Harmonic**, one package at a time, each with its own tests, keeping the
workspace buildable throughout.

## Hard constraints

- **Only ever edit files under `src/drcsim/`.** Never touch `../../sandia-hand/` or
  `../../vigir_atlas_common/` (sibling dirs at the workspace root) — those are
  read-only reference material, used only to diff/compare against when a package in
  `src/drcsim` has a duplicate there (see "Duplicate source packages" below).
- Never run `git config` (ask the user to set `user.name`/`user.email` if commits fail).
- Never `sudo apt install` anything — **no host installs, ever.**

## Environment / workflow

- **No Docker access from this shell.** The user runs everything in their own
  container: `osrf/ros:jazzy-desktop-full` image, with `drcsim_jazzy_ws/src` (which is
  `src/drcsim` plus nothing else) bind-mounted to `/root/ros2_ws/src`. Because it's a
  bind mount, files I edit on the host are immediately visible inside their container,
  and files their container writes (build artifacts, reformatted files) are
  immediately visible to me on the host — no need to paste file contents back and
  forth, only command *output* (build/test logs) needs to be relayed by the user.
- I cannot run `colcon build`, `colcon test`, `gdb`, etc. myself. **Always hand the
  user the exact command(s) to run and wait for pasted output.**
- Environment is ROS 2 Jazzy at `/opt/ros/jazzy`, Gazebo Harmonic installed via
  `gz-harmonic` apt package (side-by-side with a newer default gz-sim10 — Harmonic
  must be referenced explicitly, e.g. `gz-sim8`, `gz-plugin2`, `gz-math7`,
  `gz-common5`, `sdformat14` in CMake/package.xml, not bare `gz-sim`).
- `.gitignore` exists (`__pycache__/`, `*.pyc`, `build/`, `install/`, `log/`) — a
  stray `.pyc` from a container pytest run showed up as untracked once; don't let
  build artifacts get committed.

## Branching strategy

- Integration branch: **`ros2-jazzy-harmonic`**, cut from `master` (which stays as
  the untouched ROS 1 / Gazebo Classic historical reference).
- Per-package branches: `port/<package-name>`, cut from `ros2-jazzy-harmonic`, merged
  back with `--no-ff` once the package builds and its tests pass. (In practice a few
  follow-up fixup commits ended up landing directly on `ros2-jazzy-harmonic` instead
  of a port branch when a package's work was still "current" — harmless since it's
  already on the integration branch, just not strictly following the convention. Not
  worth going back to fix; just try to use `port/<pkg>` branches for *new* package work.)

## Duplicate source packages (description packages)

`atlas_description`, `multisense_sl_description`, `irobot_hand_description`,
`robotiq_hand_description`, `sandia_hand_description` exist in `src/drcsim` **and** in
the sibling `vigir_atlas_common/` (and `sandia_hand_description` also in
`sandia-hand/ros/`). Per the user's explicit direction: **when they differ, the copy
outside `src/`  is the base/reference** — diff first, then either use it as-is or pull
its differing content into `src/drcsim`'s copy. When content is byte-identical (common
— the outside copies often just add redundant `.obj` mesh exports alongside existing
`.dae`/`.STL`), just convert the existing `src/drcsim` files in place.

## Progress (Tier 0 — no in-repo deps beyond this tier)

All of these are converted, merged into `ros2-jazzy-harmonic`, and passing
`colcon test` (100% green) unless noted:

1. ✅ `handle_msgs`
2. ✅ `osrf_msgs` (was a nested git clone under `osrf-common/`; flattened — removed its
   `.git` and catkin toplevel stub before converting)
3. ✅ `sandia_hand_msgs`
4. ✅ `multisense_sl_description`
5. ✅ `irobot_hand_description` (fixed a real pre-existing bug: missing
   `<link name="world"/>` for a `parent="world"` joint)
6. ✅ `robotiq_hand_description`
7. ✅ `sandia_hand_description` (was untracked in git; first commit of it here. Fixed a
   pre-existing broken path in `upload.launch`/`visualize.launch` — pointed at
   `robots/sandia_hand.urdf.xacro`, which doesn't exist — retargeted at
   `sandia_hand_left_on_box.urdf.xacro`)
8. ✅ `drcsim_gazebo_plugins` — **done, 100% (8/8 checks: 2 gtests +
   copyright/cppcheck/cpplint/lint_cmake/uncrustify/xmllint).** Full architectural
   rewrite (Gazebo Classic `ModelPlugin` → gz-sim `System`
   `ISystemConfigure`/`ISystemPreUpdate`), not a mechanical port — the hardest
   package in this migration. See "drcsim_gazebo_plugins — design decisions and
   lessons" below for everything learned; keep it as reference for Tier 2's
   `drcsim_gazebo_ros_plugins` (same API family, same gotchas will recur).
9. ✅ `drcsim_model_resources` — **done, 100% (7/7 checks).** Worlds (49) + models
   (~28) + 3 AtlasSimInterface shim libraries. See "`drcsim_model_resources` —
   design decisions and lessons" below — the AtlasSimInterface binary-linking risk
   flagged in the original migration plan turned out not to apply (see below), but
   real pre-existing content bugs in the worlds/models did turn up and got fixed.

**Tier 0 is now fully complete — all 9 packages done, 100% passing.**

## Progress (Tier 1)

1. ✅ `atlas_msgs` — **done, 100% (7/7 checks), merged into `ros2-jazzy-harmonic`.**
   23 `.msg`, 4 `.srv`, 1 `.action`. Notes:
   - 8 files needed bare `Header` → `std_msgs/Header`: `Test.msg`, `AtlasCommand.msg`,
     `AtlasState.msg`, `ControllerStatistics.msg`, `AtlasSimInterfaceState.msg`,
     `WalkDemo.action`, `AtlasSimInterfaceCommand.msg`, `ForceTorqueSensors.msg`.
   - **New gotcha found**: ROS 2 constant names must be **UPPER_CASE**
     (`is_valid_constant_name` in `rosidl_adapter/parser.py` enforces
     `^[A-Z]([A-Z0-9_]?[A-Z0-9]+)*$`) — this is a hard parse error, not a lint
     warning. `AtlasState.msg` had ~50 lowercase joint-index constants
     (`back_lbz = 0`, `l_leg_hpy = 6`, etc., accessed in ROS1 as
     `atlas_msgs::AtlasStates::back_lbz`) — uppercased them all
     (`BACK_LBZ`, `L_LEG_HPY`, ...) preserving the same values/comments. Check any
     future package for this same pattern before assuming "no camelCase" means "safe".
   - `SModelRobotInput.msg`/`SModelRobotOutput.msg` (Robotiq S-Model raw driver
     registers) had field names with embedded uppercase (`gACT`, `rPRA`, ...) —
     these are **fields**, not constants, so the opposite rule bit: ROS 2 field
     names must be lower_snake_case. Renamed `gACT`→`g_act`, `rPRA`→`r_pra`, etc.
     (lowercased, `_` inserted after the leading `g`/`r`). Flag for Tier 2: if
     `RobotiqHandPlugin`/a driver node reads these by the old register names,
     it needs updating to match.
   - Dropped `sensor_msgs`, `trajectory_msgs`, `actionlib_msgs`, `osrf_msgs`,
     `sandia_hand_msgs`, `control_msgs` as dependencies — grepped every `.msg`/
     `.srv`/`.action` field type and none of them are actually referenced; only
     `std_msgs` (Header) and `geometry_msgs` (Pose/Wrench/Vector3/Quaternion) are
     real dependencies. This also resolves the pre-existing catkin inconsistency
     where `control_msgs` was a build dep but missing from
     `catkin_package(CATKIN_DEPENDS...)`.
   - `actionlib_msgs` not needed at all in ROS 2 — native `rosidl_generate_interfaces`
     action support doesn't require it.
   - **Another new gotcha found**: ROS 1's bare `time`/`duration` builtin types
     don't exist in ROS 2 — `rosidl_adapter` fails with
     `KeyError processing template 'struct.idl.em': 'time'` (a CMake-time error,
     not even a clear message pointing at the field). `VRCScore.msg` had 4
     `time` fields (`wall_time`, `sim_time`, ...) — replaced with
     `builtin_interfaces/Time` and added `builtin_interfaces` as a dependency.
     grep for `^time ` / `^duration ` in any future package's `.msg` files
     before building, alongside the bare-`Header` check.
   - Round-trip gtest added (`test/test_atlas_msgs.cpp`) covering messages, all 4
     services, and the `WalkDemo` action's Goal/Result/Feedback, following the
     established pattern. 100% on first real `colcon test` run after two small
     fixups (line-length + `time` type) — cheapest package so far per round trip.

2. ✅ `atlas_description` — **done, 100% (6/6 checks), merged into
   `ros2-jazzy-harmonic`, first-try pass (no round trips needed).** 25 top-level
   `robots/*.urdf.xacro` variants. Notes:
   - Diffed against `vigir_atlas_common/atlas_description` per the duplicate-source
     rule and found real pre-existing left/right mirroring bugs (not just noise):
     `atlas_v4`/`v5_robotiq_hands.urdf.xacro` and `atlas_v4`/`v5_sandia_hands.urdf.xacro`
     had the left-hand mount using the *same* xyz/rpy signs as the right hand
     instead of mirrored ones — confirmed against the base (non-versioned)
     `atlas_robotiq_hands.urdf.xacro`, which already had it right, so this was a
     copy-paste bug isolated to the v4/v5 variants. Fixed using vigir's values.
   - `urdf/atlas_v4_no_wry2_simple_shapes.urdf` (a raw hand-committed URDF
     fragment `<xacro:include>`d by `robots/atlas_v4_no_wry2*.urdf.xacro`, not a
     build artifact) had `l_clav` pointing at the **right** clavicle mesh
     (`r_clav.dae`) plus other right-side offsets on left-side links — replaced
     wholesale with vigir's corrected version (`atlas_v5_simple_shapes.urdf` too).
   - Did *not* pull in vigir's extra content that's outside original drcsim scope:
     `robots/vigir_atlas*.urdf.xacro`, `robots/hands/`, `robots/multisense/`,
     `launch/` — those are ViGIR-specific additions (self-filter, 7dof variant),
     not part of the package being ported.
   - Left `<gazebo><plugin filename="libXXXPlugin.so">` blocks (SandiaHandPlugin,
     RobotiqHandPlugin, IRobotHandPlugin, MultiSenseSLPlugin, AtlasPlugin)
     untouched — syntactically these don't need to change for xacro/URDF
     validity; the actual `.so` targets get renamed/rebuilt in Tier 2
     (`drcsim_gazebo_ros_plugins`), not here.
   - No launch/ directory added — neither original drcsim's `atlas_description`
     nor the reference had one (vigir's `launch/` is RViz/self-filter-specific,
     out of scope per above), unlike `robotiq_hand_description` which added one.

**Tier 1 is now fully complete — both packages done, 100% passing.**

## Tier 2 — `drcsim_gazebo_ros_plugins` (in progress, one plugin per branch)

Huge package (~13 plugin libraries + 8 CLI executables in one catkin package,
~14k lines total). Per the user's explicit direction: port **one plugin per
branch** (`port/<plugin_name>`), merging each into `ros2-jazzy-harmonic` as it
passes, rather than one giant branch for the whole package. Each branch adds
one more `add_library()` target to the package's `CMakeLists.txt`/`package.xml`,
same incremental-buildup pattern as Tier 0.

**Not-yet-ported plugins/executables are excluded from the C++ linters**, not
left to fail: `ament_lint_auto_find_test_dependencies()` scans the whole
package tree, so every plugin still in original ROS1/Gazebo-Classic style
fails `copyright`/`cpplint`/`uncrustify` immediately. Fixed the same way as
`drcsim_model_resources`'s vendored source — manual `ament_copyright(EXCLUDE
...)`/`ament_cpplint(EXCLUDE ...)`/`ament_uncrustify(EXCLUDE ...)` calls with
an `AMENT_LINT_EXCLUDE` list computed as "everything under include/+src/ minus
`AMENT_LINT_PORTED`". **Each new plugin branch must add its own files to
`AMENT_LINT_PORTED`** in `drcsim_gazebo_ros_plugins/CMakeLists.txt` — it does
not happen automatically.

1. ✅ `ContactModelPlugin` — **done, 100% (7/7 checks), merged into
   `ros2-jazzy-harmonic`.** Despite living in `_ros_plugins`, has zero ROS
   coupling — pure gz-transport, republishes contact info for an arbitrary
   named set of a model's collisions. Notes:
   - Gazebo Classic's `physics::ContactManager::CreateFilter` (arbitrary
     named-collision-set filter) has **no equivalent** in gz-sim. Instead,
     contact data only populates once a collision carries a
     `components::ContactSensorData` component (normally attached via SDF
     `<sensor type="contact">`). Reproduced the original's "watch these
     collisions without a dedicated sensor" behavior by force-creating that
     component directly (`_ecm.CreateComponent(colEntity,
     components::ContactSensorData())`) on each matched collision — the same
     technique gz-sim's own example `TouchPlugin` uses (fetched and read its
     actual upstream source before writing this, not guessed).
   - SDF interface unchanged: `<contact><collision>link::collision</collision>
     ...<topic>...</topic></contact>` — confirmed against real usage in
     `sandia_hand_description/urdf/sandia_hand.gazebo.xacro`.
   - ROS 1's bare `time`-family lesson doesn't apply here (no ROS messages),
     but a **new, expensive lesson** did — see "`.cc` vs `.cpp`: the real
     cause of the `ament_uncrustify` template-call saga" below. At the time
     this package was ported the root cause wasn't known yet, so its test
     works around the symptom (`EntityHasComponentType` instead of
     `Component<T>()`) rather than the cause; that workaround is harmless
     and hasn't been revisited, but a `.cc`→`.cpp` rename would probably
     let it use `Component<T>()` directly if ever revisited.

2. ✅ `SandiaHandPlugin` — **done, 100% (9/9 checks), merged into
   `ros2-jazzy-harmonic`.** The first ROS-coupled plugin in this package
   (`ContactModelPlugin` had none despite the package name): 12-joint PID
   position/velocity control from `osrf_msgs/JointCommands`,
   `sensor_msgs/JointState` feedback, an IMU feed, a
   `SetJointDamping`/`GetJointDamping` service pair, and a tactile sensor
   array synthesized from contact data. Test coverage: a stumps-mode test
   (no hand joints present, just checks the ROS interface stands up) plus
   a full 12-joint test (`test_sandia_hand_plugin_joints.cpp`, a real
   4-finger hand model) that publishes a `JointCommands` message and
   confirms the joint actually moved, and confirms a tactile value
   changes from default when a dynamic object rests under gravity on a
   world-anchored palm collision. That second test's world went through
   one real bug fix: a fixed-joint-anchored link touching a `<static>`
   object produces **exactly zero** contact force (nothing for the
   solver to resist), and the plugin's own force→tactile-value scaling
   formula maps zero force to precisely `minTactileOut` — indistinguishable
   from "no contact" at all. Fixed by making the touching object dynamic
   and gravity-loaded instead of static, so there's a real, sustained
   normal force to report. **If a future contact-based test's assertion
   passes-when-it-shouldn't or fails-when-contact-looks-real, check
   whether both colliding bodies are actually free to generate a
   nonzero constraint force before assuming the detection logic is
   broken.** Notes:
   - **Established the ROS-integration pattern every remaining ROS-coupled
     Tier 2 plugin should reuse**: each plugin owns its own `rclcpp::Node`,
     spun on a dedicated thread via a `SingleThreadedExecutor`. Calls
     `rclcpp::init()` only if `rclcpp::ok()` is false, and **never** calls
     `rclcpp::shutdown()` from its destructor — multiple such plugins
     (RobotiqHandPlugin, IRobotHandPlugin, MultiSenseSLPlugin, VRCPlugin,
     AtlasPlugin, DRCVehicleROSPlugin, all still to come) share one process
     and one global rclcpp context; only cancel/join this instance's own
     executor thread and reset its own node/pub/sub/service handles.
   - No `gazebo::sensors::ImuSensor` equivalent readable off the ECS in
     gz-sim (IMU sensor output is transport-only, and needs the
     `gz-sim-imu-system` world plugin loaded). Reads the IMU link's raw
     physics state directly instead: `Link::WorldPose`/
     `WorldAngularVelocity`/`WorldLinearAcceleration` (the latter two need
     `EnableVelocityChecks`/`EnableAccelerationChecks` called once first),
     world-frame vectors rotated into the link's own frame via
     `pose.Rot().RotateVectorReverse(worldVec)`. Trades away the
     SDF-configured sensor noise model for a much simpler implementation.
   - No runtime joint-damping mutation API in gz-sim (same gap as
     `DRCVehiclePlugin`'s already-dropped `Set*Limits`) — the damping
     service now only updates a cached, clamped value; request/response
     contract unchanged.
   - No default per-model aggregate contact topic in gz-sim — reused
     `ContactModelPlugin`'s force-create-`ContactSensorData` technique on
     this hand's own finger/palm collisions, matching contacts back to
     them by `gz::msgs::Entity` id (`contact.collision1().id()`, a raw
     `uint64` equal to the `gz::sim::Entity` value) instead of the
     original's collision-name substring matching — exact instead of
     fuzzy, and a good default technique for any future plugin needing
     "which of my own collisions is this contact about".
   - PID gains: originally pulled per-joint `p`/`i`/`d`/`i_clamp` off the
     ROS param server (populated by a launch file this migration hasn't
     reached yet). Declared as ROS 2 parameters instead
     (`gains.f<N>_j<M>.{p,i,d,i_clamp}`, default `0.0`) — settable the
     same way once a launch file exists to provide them.

3. ✅ `IRobotHandPlugin` — **done, 2/2 gtest cases passing (81/81 package
   checks total), merged into `ros2-jazzy-harmonic`.** 23 joints per hand (2 base-rotation + 3 base +
   3×(3 flex + 3 twist)); unlike `SandiaHandPlugin`, the original has **no
   stumps fallback** — `FindJoints()` aborts the whole load the first time any
   one expected joint name is missing, and this port preserves that exactly.
   Control is tendon-based: a PID computes a torque per DOF (index flex,
   middle flex, thumb flex, thumb antagonist, spread), split between a finger
   base joint and its flexure-flex joints. Topics keep the original's
   inconsistent prefix scheme on purpose (not normalized): joint states on
   `irobot_hands/<l/r>_hand/joint_states`, but handle sensors/control on bare
   `<left/right>_hand/sensors/raw` and `<left/right>_hand/control` (full word
   side, no `l_`/`r_` shorthand). Notes:
   - **Passive joint springs reimplemented as an explicit PD force**: the
     original faked flexure/base-joint springiness with
     `gazebo::physics::Joint::SetStiffnessDamping()` (ODE-specific, no gz-sim
     equivalent — same gap as `DRCVehiclePlugin`'s dropped `Set*Limits` and
     `SandiaHandPlugin`'s dropped damping mutation), re-called every step with
     a **dynamically overwritten damping value** ("hack to reduce jitter" per
     the original's own comment) alongside the active tendon force. Since a
     spring is just `force = -kp*(pos - preload) - kd*vel`, this port computes
     that explicitly each step — including the dynamic-damping override — and
     adds it to the tendon force in one `Joint::SetForce()` call per joint.
     The flexure **twist** joints and finger-base-rotation joint 1 are
     passive-only in the original (never touched by `UpdatePIDControl`) and
     keep a fixed (non-dynamic) spring here.
   - **Thumb antagonist limit: control-level check only, no hard physics
     stop.** The original also called `Joint::SetUpperLimit()` every step to
     move the thumb base joint's hard physics limit as the antagonist value
     changed; gz-sim has no runtime joint-limit mutation API, so only the
     control-level check the original already had independently (skip
     applying tendon force once past the effective limit) survives.
   - `gz::sim::Joint` has **no `UpperLimit()`/`LowerLimit()` reader methods**
     (only `Set*Limits()` setters) — read the thumb's static SDF-configured
     limits once at load time via `Joint::Axis(ecm)` →
     `sdf::JointAxis::Lower()`/`Upper()` instead (needs
     `#include <sdf/JointAxis.hh>`).
   - Reuses `SandiaHandPlugin`'s rclcpp node/executor/thread pattern exactly
     (own node named `irobot_hand_plugin_<side>` since two instances share
     the process).
   - **Caught before it ever reached the user**: first draft computed the PID
     derivative term *after* overwriting `positionError` with the new error,
     making `(qP - positionError)/dt` always evaluate to zero. Fixed by
     computing the derivative first, then updating `positionError` —
     `SandiaHandPlugin.cpp`'s `UpdateStates()` already does this in the right
     order and was the reference for the fix.
   - Test coverage so far: a cheap negative test (`irobot_hand_missing_joints_test.sdf`,
     no finger joints at all) confirming the no-stumps-fallback abort path
     doesn't crash and never stands up a ROS interface, plus a full 23-joint
     test (`irobot_hand_full_test.sdf`, generated with a small Python script
     rather than hand-typed — see `/tmp/gen_irobot_world.py` pattern for any
     future large kinematic-chain test world) confirming the joint_states/
     handle sensors publishers and the control subscription all stand up.
     No test yet exercises actual tendon control moving a joint (the
     `SandiaHandPlugin`-style follow-up test) — a natural next step if the
     user wants deeper coverage once the ROS-interface test is green.

4. ✅ `RobotiqHandPlugin` — **done, 94/94 package checks passing, merged into
   `ros2-jazzy-harmonic`.** A state machine over the SModel Robot
   Output/Input protocol (`atlas_msgs::msg::SModelRobotOutput`/`Input`,
   already ported in Tier 1) driving a 3-finger adaptive gripper: activation,
   4 grasping modes (Basic/Pinch/Wide/Scissor), per-finger position/speed/
   force targets, and object-detection feedback, with PID position control
   on the 5 actuated hinge joints. Notes:
   - **`gazebo::common::PID` → `gz::math::PID`** — this plugin's PID needs
     (position control with configurable gains/effort limits) are the same
     ones already met by `DRCVehiclePlugin`'s `gz::math::PID` usage in Tier 0,
     so this reuses that exact library rather than reimplementing a PID.
     Nearly a drop-in replacement: same `Init(p,i,d,imax,imin,cmdMax,cmdMin)`
     argument order and `error = current - target` convention, but getters
     dropped their `Get` prefix (`PGain()`, `Errors()`, ...) and `Update()`
     takes a `std::chrono::duration<double>` instead of a bare double.
   - **Two joint vectors, preserved deliberately, not simplified**:
     `fingerJoints` (5, actuated, `SetForce()` target) vs. `joints` (12,
     every joint worth reading, used for both `JointState` publishing and PID
     feedback). For the 2 simple spread joints these are the same joint; for
     the 3 underactuated 4-bar-linkage fingers they're genuinely different —
     the PID reads the finger's actual curl angle off `finger_N_joint_1` but
     drives torque through `finger_N_joint_proximal_actuating_hinge`, the
     real motor DOF. Losing this distinction would silently break the
     underactuated fingers' control.
   - Effort limits: no `Joint::GetEffortLimit()` reader in gz-sim (same
     `Joint::Axis(ecm)` → `sdf::JointAxis` pattern as `IRobotHandPlugin`'s
     limit reading, this time `.Effort()` instead of `.Lower()`/`.Upper()`).
   - **Fixed a copy-paste bug found in the original while porting**:
     `VerifyCommand()` checked every field's range against `rACT` instead of
     its own field (e.g. `VerifyField("rMOD", 0, 3, _command->rACT)`) — since
     valid `rACT` is always 0 or 1, this silently made every other range
     check a no-op. Fixed to check each field against itself; can only newly
     reject commands that were already out of their own field's declared
     range, doesn't change behavior for any command that was valid before.
   - `JointState.effort` left at zero for the 10 non-actuated-hinge joints —
     no generic "last applied generalized force" reader in gz-sim, and for
     the 3 underactuated fingers the meaningful torque was commanded on a
     different joint than the one being read anyway (see above).
   - A pre-existing behavior deliberately **not** "fixed": `UpdatePIDControl`
     computes a `targetSpeed` value in the Scissor grasping branch that is
     never actually read afterward in the original either — looks like
     vestigial support for a velocity-mode path that was never wired up.
     Preserved as dead code with a comment rather than removed or wired up,
     since guessing at unimplemented original intent is worse than leaving
     it alone.
   - Test coverage so far: a cheap negative test (no finger joints at all,
     confirms the no-stumps-fallback abort is graceful) plus a positive test
     against a simplified (non-4-bar-coupled) test world that commands Wide
     grasping mode and confirms `l_palm_finger_1_joint` — one of the two
     joints where the informative and actuated joint are literally the same
     entity, so it moves correctly even without the real linkage geometry —
     swings measurably off zero. **First run failed** (moved only 0.0105 rad,
     well under the 0.05 rad threshold): the command publisher started
     publishing immediately with no confirmation that DDS discovery had
     actually matched it to the plugin's subscription, so most of the timed
     loop likely ran with `handState == Disabled` (zero commanded force)
     before the first command ever arrived. Fixed by adding a
     `count_subscribers()` wait before the timed loop — same graph-
     introspection-discovery-wait pattern already noted in the lessons below,
     re-confirmed here as something to reach for by default whenever a test
     publishes a command and then immediately expects it to have taken
     effect.

5. ✅ `MultiSenseSLPlugin` — **done, 106/106 package checks passing, merged
   into `ros2-jazzy-harmonic`.** Drives the MultiSense SL head sensor head: a
   velocity-PID spindle joint (rotates the head lidar), a head IMU feed,
   spindle joint-state publishing, and ROS topics for the stereo camera's
   frame rate/resolution/exposure/gain. Renamed from the original's bare
   `MultiSenseSL` class name to `MultiSenseSLPlugin` for consistency with
   every other ported plugin's class-name-matches-file-name convention.
   Notes:
   - IMU handled via the same "read the link's raw physics state, rotate
     into local frame" technique as `SandiaHandPlugin` (no ECM-readable IMU
     sensor object in gz-sim).
   - **Camera control has no live backing anymore**: the original mutated a
     `sensors::MultiCameraSensor` found via the global `SensorManager`
     singleton (frame rate, resolution, image dimensions). gz-sim has no
     runtime camera-reconfiguration API reachable from a `System` plugin, so
     `SetMultiCameraFrameRate`/`SetMultiCameraResolution` now only update a
     cached value — same "cache but don't mutate" pattern as the
     joint-damping/limit gaps in earlier plugins, just applied to a sensor
     instead of a joint this time.
   - `atlas_version` (decides `/multisense` vs `/multisense_sl` namespace)
     was read from a shared ROS 1 param-server value in the original; no
     such shared server in ROS 2, so it's declared as this node's own
     parameter instead (default `5` → `/multisense`), same fix already
     applied to `SandiaHandPlugin`'s PID gains.
   - **A closer read of the original turned up more dead code than expected**:
     of the 7 `Set*` ROS callback methods the class defines, only 3
     (`SetSpindleSpeed`, the deprecated `~/fps` alias, `~/set_fps`) were
     ever actually wired to a subscription — `SetSpindleState`,
     `SetMultiCameraExposureTime`, and `SetMultiCameraGain` sat inside a
     `/* not implemented, not supported */` comment block in the original
     and were never advertised; a matching `SetSpindleSpeed`/`SetSpindleState`
     *service* overload pair (`std_srvs::Empty`-based) was similarly
     never advertised (`/* waiting for gen_srv */`). Preserved that split
     faithfully: the never-wired methods are ported (still just update a
     cached value) but still never bound to a subscription, and the fully
     inert service overloads were dropped entirely rather than ported, since
     there's nothing to preserve about code nobody could ever call. The one
     exception: `SetMultiCameraResolution`'s subscription *was* wired up
     here, even though the original commented it out too — but for a
     different, load-bearing reason ("currently causes simulation to
     crash"), a bug specific to old Gazebo/ROS 1 that doesn't carry over,
     and this callback is no riskier than the two neighboring ones that
     *are* live in the original.
   - A minor, deliberate non-preservation: the original only advanced its
     own `lastTime` (used for the spindle PID's `dt`) while the spindle was
     on, leaving it frozen while off and producing one oversized `dt` on the
     next re-enable. This port always advances `lastControllerUpdateTime`
     every step regardless, matching the convention every other plugin in
     this package already uses -- avoids that edge case rather than
     reproducing it, since it looks like an accidental side effect of the
     original's structure rather than a meaningful design choice.

6. ✅ `VRCPlugin` — **done, 118/118 package checks passing, merged into
   `ros2-jazzy-harmonic`. By far the largest and most speculative port in
   this migration** (2685 original lines; a `WorldPlugin`, not a
   `ModelPlugin` like everything else so far) — pin/teleport the robot, planar cmd_vel
   teleop while pinned, vehicle enter/exit, fire-hose grab, and a fake
   AtlasSimInterface that turns STAND/FREEZE/WALK/STEP commands into
   pin/PID/teleport tricks. **User explicitly chose the full port** (not a
   reduced-scope version) when asked, given the size/risk — see the
   extensive class-level design-note doc comment at the top of
   `VRCPlugin.hpp`, which is the authoritative record of every judgment
   call below; this entry summarizes it.
   - **Five separate gz-sim capability gaps were individually researched
     against installed headers and, where ambiguous, shipped `.so` binary
     symbol tables** (a first for this migration — every previous gap was
     resolved from headers/doc comments alone) before deciding on a
     substitute:
     1. **Per-link/model runtime gravity and static toggling**
        (`Link::SetGravityMode()`, `Link::SetLinkStatic()`): the matching
        `components::GravityEnabled`/`components::Static` exist, but binary
        evidence (an SDF getter paired 1:1 with a one-shot native
        engine call, e.g. `dart::dynamics::Skeleton::setMobile()`) says
        both are construction-time-only, not read every step. Gravity:
        substituted with an explicit per-step counter-force
        (`Link::AddWorldForce(ecm, -mass*gravity)`) on a tracked set of
        "gravity disabled" links — the same "reimplement the missing
        mutation as an explicit force" idiom already used for joint
        springs in `IRobotHandPlugin`/`RobotiqHandPlugin`. Static: not
        needed at all, since the *primary* pinning mechanism (a real
        joint) doesn't depend on it — only Classic's simbody/dart-specific
        fallback branch did, and that branch is dropped outright.
     2. **Per-link/collision collide-mode toggling** (`SetCollideMode()`,
        Classic's `SetFeetCollide()`): no component-flag equivalent exists
        at all (only full collision-entity removal/recreation, evidence-
        supported but heavy). Dropped to a no-op — cosmetic only (feet may
        briefly clip during pin/teleport), not functional.
     3. **Runtime model spawning from an SDF/URDF string**
        (`World::InsertModelString()`, used only for the Atlas robot if
        missing from the world file): fully achievable via
        `sdf::Root::LoadSdfString()` (auto-converts URDF) +
        `gz::sim::SdfEntityCreator::CreateEntities()` + `SetParent()` —
        confirmed to be the exact mechanism gz-sim's own `UserCommands`
        system uses behind `/world/<world>/create`. Synchronous (unlike
        Classic's async spawn-then-poll), so `SPAWN_QUEUED`/
        `CheckGetModel()` are now unreachable dead states, kept only for
        state-machine fidelity.
     4. **Runtime joint creation between two links** (a custom
        `PhysicsEngine::CreateJoint()`-based `AddJoint()` in the original):
        split by whether the weld is to the world or cross-model.
        World-pin: a real SDF `fixed` joint (`parent` name literally
        `"world"`) via `SdfEntityCreator`, parented under the pinned
        link's own model — the same `<parent>world</parent>` idiom this
        project's test worlds already use, built at runtime instead of
        loaded from text. Cross-model rigid welds (robot hand↔fire hose,
        vehicle seat↔robot pelvis): `gz::sim::components::DetachableJoint`
        on a fresh bare entity — confirmed via the physics system's own
        binary symbols to need no other components, work across
        completely different models (`parentLink`/`childLink` are raw
        `Entity` ids, no scoping check), and need no extra world-level
        system plugin loaded. The fire-hose↔standpipe docking connection
        is *also* cross-model but needed a real screw joint (dartsim
        genuinely implements screw joints, confirmed by binary symbol
        inspection down to `dart::dynamics::ScrewJoint::setPitch`) — but
        only reachable via the intra-model `SdfEntityCreator` joint path,
        which doesn't apply cross-model, and `DetachableJoint` only
        supports `"fixed"`. **Dropped**: docking is now a fixed weld too
        (same proximity/alignment auto-attach check as the original,
        `CheckThreadStart()`, unchanged), permanently losing the
        "unscrew to disconnect" mechanic (no live joint angle to read
        anymore).
     5. **Instant world-pose teleport** (`Model::SetWorldPose()`/
        `SetLinkWorldPose()`, used constantly: pin, enter/exit car, grab
        hose, cmd_vel warp): **not** a direct `components::Pose` write
        (that's a physics *output*, silently clobbered next step) —
        the real mechanism is `Model::SetWorldPoseCmd(ecm, pose)`, a
        *command* component (`components::WorldPoseCmd`) the physics
        system consumes via `gz::physics::SetFreeGroupWorldPose`,
        confirmed by binary inspection and by `gz-sim-user-commands-
        system`'s own `/world/<world>/set_pose` service being built on
        the identical component. **Model-level only** — no per-link
        teleport exists, so repositioning one link of a multi-link model
        means computing what model pose would place that link at the
        desired pose (via the link's currently-solved rigid offset from
        the model) and commanding that instead — the same transform
        Classic's `SetLinkWorldPose()` did internally. A related, smaller
        gap: no instant "snap all joints to this pose" writer either
        (`Model::SetJointPositions()`), used for BDI-stand/seated-pose
        visual snaps — dropped; only the `AtlasCommand` publish survives,
        so those poses only take effect once `AtlasPlugin` (not yet
        ported) exists downstream to PID toward them.
   - **A sixth, architectural change, not a missing-API gap**: the
     original directly mutated Gazebo Classic physics objects *from
     inside* ROS callbacks — safe there only because Classic's physics
     objects have their own internal locking. gz-sim's ECM has no such
     protection, so (matching what every other ROS-coupled plugin in this
     migration already did, just made explicit and rigorous here given how
     much more `VRCPlugin` does) every ROS callback that needs to touch
     the ECM now only records a pending request (`pendingSetRobotMode`,
     `pendingFakeASIC`, `pendingEnterCar`, etc., all under
     `controlMutex`); a new `ProcessPendingActions()`, called once per
     `PreUpdate` on the sim thread, copies the pending data out under the
     mutex, releases it, then performs the actual `Do*` (`DoSetRobotMode`,
     `DoSetFakeASIC`, `DoRobotEnterCar`, ...) methods with real ECM access
     — releasing the mutex first is required since some `Do*` methods
     (e.g. `DoSetFakeASIC`) internally call other public methods
     (`SetRobotCmdVel`) that themselves lock the same mutex, which would
     otherwise deadlock. A side effect: this also makes the original's
     `World::SetPaused()`/`EnablePhysicsEngine()` pause/unpause dance —
     which existed specifically to guard against a ROS-callback thread and
     the physics thread touching the same object at once — unnecessary
     rather than merely hard to port, since that race can no longer occur.
   - The rarely-used `"harnessed"` startup mode's downward raycast for
     ground height (`Entity::GetNearestEntityBelow()`) is replaced with the
     same flat-ground fallback the original already used whenever its own
     raycast found nothing — `components::RaycastData` is real but its
     result carries no entity-identity, and this mode was already excluded
     from the original's own "available modes" help text.
   - Test coverage: one test bringing up a minimal free-floating "atlas"
     stand-in (utorso + fixed-jointed feet, already present in the world
     file so the SDF-string-spawn path isn't exercised yet), confirming the
     ROS interface (AtlasCommand publisher, cmd_vel subscriber) stands up,
     and — the more meaningful check — reading utorso's world pose directly
     off the ECM via `TestFixture::OnPostUpdate()` to confirm the default
     startup sequence's automatic "pinned" mode actually holds the 50kg
     body almost exactly at its spawn height under real gravity, which only
     works if the world-pin joint creation *and* the counter-gravity force
     are both actually functioning. (The fake-ASIS state topic was tried
     first for this check and dropped: it only starts publishing once
     `Robot::INITIALIZED`, which — by design — is well *after* the default
     5-second harness duration auto-unpins the robot again, so it can never
     usefully confirm "still pinned.")
   - **Real bugs the build/test round actually caught** (beyond the
     speculative-API risk already flagged above): (1) `sdf::Element::
     GetElement()` is non-const in this sdformat version, but `Configure()`
     only hands a plugin a `const shared_ptr<const sdf::Element>` — needed
     `_sdf->Clone()` into a mutable `sdf::ElementPtr` before any code that
     digs into nested `<atlas>`/`<drc_vehicle>`/`<drc_fire_hose>` blocks.
     (2) `components::CollisionElement` (the sdf::Collision-wrapping
     component) is declared inside `components/Collision.hh`, not its own
     `CollisionElement.hh`. (3) **Two separate ROS 2
     `declare_parameter()`-is-one-shot crashes** — one from a per-joint
     parameter name colliding whenever `FindJoint()` returned an empty
     string for more than one missing DOF, one from a *fixed* parameter
     name declared from inside `UpdateStates()`'s per-tick state machine
     instead of a true one-time setup path. See the new "Runtime gotcha...
     declare_parameter() is one-shot" section below — this is now a
     general lesson, not VRCPlugin-specific, and worth watching for in
     `AtlasPlugin` given how much startup-parameter machinery it likely
     has too.

7. ✅ `AtlasPlugin` — **done, 130/130 package checks passing on the first
   real correction round (only 2 trivial uncrustify indentation nits, no
   gtest failures at all), merged into `ros2-jazzy-harmonic`. The largest
   single file in this migration** (~3500 lines across header + source,
   narrowly beating `VRCPlugin`) — the 1kHz joint
   PID controller that actually holds the robot up, fed from
   `atlas/atlas_command`, plus a BDI-behavior-library ("AtlasSimInterface")
   integration fed from `atlas/atlas_sim_interface_command`, blended
   per-joint by a `k_effort` weight. **This is what was missing the whole
   time `VRCPlugin` existed without it** — VRCPlugin only ever *published*
   `AtlasCommand`/pinned the pelvis; nothing consumed those messages or
   drove the other ~28 joints, so a spawned Atlas had nothing holding it
   up and collapsed exactly as the user observed.
   - **The single biggest finding of this entire migration**: the
     "proprietary AtlasSimInterface, likely won't link against a modern
     toolchain" risk flagged in the very first planning pass **does not
     exist**. All three bundled versions
     (`drcsim_model_resources/AtlasSimInterface_{1.1.1,2.10.2,3.0.2}/src/
     AtlasSimInterface.cc`) are byte-for-byte identical (confirmed via
     `diff`), Apache-2.0-licensed OSRF stub code that prints `"Warning:
     Using Atlas Shim interface. Atlas will be more or less
     uncontrolled"` on load. Every "BDI behavior" function
     (`set_desired_behavior`, `get_current_behavior`, walk/step/stand
     param processing) is a no-op returning only `NO_ERRORS`;
     `process_control_input()` — the one function that does anything — is
     itself just a joint PID identical in shape to `AtlasPlugin`'s own
     main controller. There is no real Boston Dynamics balance controller
     anywhere in this codebase, no proprietary binary, no ABI risk. Dropped
     the entire external-library integration (`create_atlas_sim_interface
     ()`, the version-conditional `#include "AtlasSimInterface_X.Y.Z/
     AtlasSimInterface.h"`, the `AtlasControlInput/AtlasControlOutput/
     AtlasRobotState` C-struct plumbing) and inlined the shim's exact PID
     formula as `UpdateAtlasSimInterface()`, operating directly on
     `atlas_msgs` ROS types.
   - **Two corollaries of that same discovery, both changing behavior
     deliberately**: (1) the shim's `get_desired_behavior()`/
     `get_current_behavior()` never write their output-string reference
     parameter, so the original's `behaviorMap[behaviorStr]` lookup was
     always against an empty key — `asiState.current_behavior` was
     **always** silently reported as `NONE` in the original too,
     regardless of what was requested. Not preserved: this port just sets
     `current_behavior = desired_behavior` directly (same judgment already
     applied once, to `RobotiqHandPlugin`'s copy-paste `VerifyCommand()`
     bug — faithfully reproducing an always-wrong report serves no one).
     (2) The shim's output struct's rich per-behavior feedback fields
     (stand/step/walk/manipulate feedback) were only ever zero-initialized
     and never subsequently written by the shim, so they were already
     permanently inert in the original's real, shim-backed build — this
     port leaves them at their message-default zeros rather than building
     unused plumbing to compute values nothing upstream ever populated.
   - **Joint damping mutation**: same gap as `DRCVehiclePlugin`/
     `SandiaHandPlugin`/`RobotiqHandPlugin` — every `Joint::SetDamping()`
     call (`SetAtlasCommand`, the `set_joint_damping` service,
     `SetExperimentalDampingPID`, `UpdatePIDControl`'s cfm-damping
     pass-through) now only updates a cached value; the surrounding
     control-law math (the `kpVelocityDampingEffort` clamp-shift term)
     is pure arithmetic, preserved exactly since it never depended on the
     mutation reaching physics.
   - **Wrist/ankle force-torque "sensors"**: Classic's `Joint::
     GetForceTorque(0u)` gave a `body1`/`body2` two-body wrench pair;
     gz-sim's `Joint::TransmittedWrench()` (needs `EnableTransmittedWrench
     Check()`, same pattern as `EnablePositionCheck()`) gives only one
     force/torque pair, in the **joint frame**, at the joint origin —
     used directly as the `body2` analog without transforming back to the
     child-link frame Classic used (an approximation, exact only when the
     joint frame coincides with the child link's).
   - **Foot contact sensors** (`atlas/debug/{l,r}_foot_contact`,
     cheats-gated): the same force-created `components::
     ContactSensorData` technique already established for
     `ContactModelPlugin`/`SandiaHandPlugin`.
   - **Controller synchronization delay** (a lockstep mechanism letting an
     external controller ask the sim to wait, via a real-time delay
     budget, for a fresh `AtlasCommand`): pure `boost::condition`/
     `boost::mutex` logic with no gz-sim-specific dependency at all —
     ported 1:1 onto `std::condition_variable`/`std::mutex`. Off by
     default.
   - **`jointCommands`/`ZeroJointCommands()` dropped**: dead state in the
     original — `SetJointCommands()` (the `atlas/joint_commands` callback)
     writes straight into `atlasCommand`/`atlasState` and never touches
     the separately-declared `jointCommands` member, which only its own
     zeroing function ever wrote.
   - **Mutex discipline**: unlike every plugin before it, this one has
     *two* mutexes (`controlMutex` for PID/command state, `asiMutex` for
     the BDI-shim state) that some methods need data from both sides of.
     Established a hard rule while writing this: **never hold both
     mutexes at once** — where a value from the other side is needed
     (e.g. `UpdateAtlasSimInterface()` needs `atlasState.k_effort`),
     snapshot it under a short-lived, independent lock, release it, then
     take the other mutex — rather than nesting, which would create a
     lock-ordering deadlock risk against any other call site that happens
     to acquire the same two mutexes in the opposite order (which,
     mirroring the original's own already-non-nested structure in
     `SetASICommand`/`ResetControls`, is exactly what a couple of ROS
     callbacks do).
   - Unlike `VRCPlugin`, **no pending-action queue was needed at all** —
     every one of this plugin's ROS callbacks (`SetAtlasCommand`,
     `SetJointCommands`, `SetASICommand`, `OnRobotMode`, `Tic`,
     `SetExperimentalDampingPID`, and all four services) only ever updates
     a mutex-protected cached value; none of them create/destroy entities
     or teleport a model, so the same safe pattern `SandiaHandPlugin`'s
     `SetJointCommands` already used sufficed everywhere here too.
   - Test coverage: a 30-joint test world (leg/arm/spine kinematic chains
     built with a small Python generator script, matching the pattern
     already used for `IRobotHandPlugin`/`RobotiqHandPlugin`), confirming
     the ROS interface stands up and that commanding a nonzero position
     on `back_bkz` via `atlas/atlas_command` actually moves that joint
     there under real PID control.

**Remaining Tier 2 plugins** (each its own `port/<name>` branch):
VRCScoringPlugin, DRCVehicleROSPlugin (**subclasses** `DRCVehiclePlugin` —
see below), then the 8 CLI executables + `actionlib_server` +
`gz_model_teleport` + `test_ros_plugin`.
ContactModelPlugin ✅, SandiaHandPlugin ✅, IRobotHandPlugin ✅,
RobotiqHandPlugin ✅, MultiSenseSLPlugin ✅, VRCPlugin ✅, AtlasPlugin ✅.

### `.cc` vs `.cpp`: the real cause of the `ament_uncrustify` template-call saga

**If a test file (or any file) needs an explicit template call like
`std::make_shared<T>(...)`, `create_client<T>(...)`, or
`EntityComponentManager::Component<T>(...)`, name it `.cpp`, not `.cc`.**
This was a multi-package, multi-day mystery — `ament_uncrustify` kept
misparsing these calls as comparison expressions (wanting spaces around `<`
and `>`, which `cpplint` then rejects) no matter how the surrounding code was
restructured: return-vs-assign, free function vs. lambda vs. out-of-line
`ClassName::Method()`, long vs. short type names, one-line vs. multi-line,
even a plain `std::vector<T>` parameter declaration with no relation to any
of the above. **Every failing instance, across both `ContactModelPlugin` and
`SandiaHandPlugin`'s test files, was in a `.cc` file. Every passing instance
was in a `.cpp` file** — including the identical call, verbatim, in both:
`test_sandia_hand_plugin.cc` failed on `std::make_shared<rclcpp::Node>(...)`
in every restructuring tried; renaming the file to
`test_sandia_hand_plugin.cpp` with **no other change** fixed it immediately.
`ament_uncrustify`'s language/config detection apparently treats `.cc`
differently from `.cpp` in this toolchain, in a way that breaks template
disambiguation specifically. `test_contact_model_plugin.cc` never got this
fix applied (its workaround — avoiding `Component<T>()` via
`EntityHasComponentType` instead — already shipped and merged, and is
harmless as-is), but any *new* `.cc` file that needs an explicit template
call should just be a `.cpp` file from the start.

## `drcsim_gazebo_plugins` — design decisions and lessons (done, keep as reference)

Two plugins, `DRCBuildingPlugin` (door+handle, small) and `DRCVehiclePlugin`
(~1000 lines, pedals/wheel/handbrake/FNR + 4 drive wheels — the single largest,
riskiest file in this whole migration). Both now build.

**Design decisions already made and applied (don't re-litigate these):**
- Dropped ODE-specific `SetParam("stop_erp"/"stop_cfm", ...)` wheel hard-lock and
  door-latch joint-limit toggling — no gz-sim equivalent; the torque-based
  braking / PID-holds-near-zero already in the code handles it, confirmed
  sufficient with the user.
- `DRCVehiclePlugin`'s public API (`SetVehicleState`, `SetHandWheelState`,
  `GetKeyState`, etc.) is kept as **plain C++ methods**, not a gz-transport
  interface. The only real consumer, `DRCVehicleROSPlugin` (Tier 2, not yet
  ported), **subclasses** `DRCVehiclePlugin` (`class DRCVehicleROSPlugin: public
  DRCVehiclePlugin`) and calls these methods directly via normal C++ inheritance —
  confirmed via repo-wide grep before deciding this (an earlier assumption that a
  *separate* plugin held a raw pointer to this one was wrong; don't reintroduce a
  transport-based design for this).
- `Set*Limits` methods now only update the plugin's *cached* limits, not physics
  joint limits (gz-sim has no well-supported runtime joint-limit mutation API, and
  grep confirmed nothing in the repo calls the Set*Limits methods anyway).
- Files renamed `.hh`/`.cc` → `.hpp`/`.cpp`: `ament_uncrustify` parses `.hh` as
  plain C (not C++), chokes on the base-class-list colon. `.hpp`/`.cpp` fixed it.

**Known-good API shapes learned from real compiler feedback (sdformat14 / gz-sim8,
i.e. Harmonic component versions) — trust these over any earlier assumption:**
- `sdf::Element::Get<T>(key, default)` (two-arg) returns **`std::pair<T, bool>`**,
  not `T` — need `.first`. The one-arg `Get<T>(key)` (used with an `HasElement()`
  guard) returns `T` directly.
- `sdf::Geometry::CylinderShape()`/`SphereShape()` need explicit
  `#include <sdf/Cylinder.hh>` / `<sdf/Sphere.hh>` — `Geometry.hh` only
  forward-declares them.
- `gz::sim::Joint::Position(ecm)`/`Velocity(ecm)` return
  `std::optional<std::vector<double>>` where the **optional can be present but
  contain an empty vector** (not just `nullopt`) — `.value_or(fallback)[0]` is
  **undefined behavior** in that case (confirmed via gdb: a real segfault, fault
  address inside the plugin's own `PreUpdate`, matching an inlined
  out-of-bounds-vector-index crash shape exactly). Always check
  `opt && !opt->empty()` before indexing `[0]`. Fixed via a local `FirstOrZero()`
  helper in both plugin `.cpp` files.
- `Joint::EnablePositionCheck(ecm)` / `EnableVelocityCheck(ecm)`, `Model::Valid(ecm)`,
  `Model::JointByName`/`LinkByName`, `Link::Collisions(ecm)`, `gz::sim::worldPose()`,
  `components::JointAxis`/`ChildLinkName`/`Geometry`, `gz::math::PID::Init()`/
  `.Update(error, std::chrono::duration<double>)`, `GZ_ADD_PLUGIN`/
  `GZ_ADD_PLUGIN_ALIAS` from `<gz/plugin/Register.hh>` — all confirmed working as
  used in the current source.

**`ament_uncrustify` vs `ament_cpplint`: they disagree with each other, repeatedly.**
Running `ament_uncrustify --reformat` blindly is *not* safe — it fixed one style
issue and simultaneously reintroduced others cpplint rejects, and even introduced
a genuine syntax error once (dropped a `;` after a wrapped `return {0.0, 0.0}`).
Patterns that came up, and how each was resolved for good (i.e. verified both
tools pass simultaneously afterward):
- Inline per-member `public: Foo();` labels: uncrustify rewrites these to
  standalone `public:` labels *with a blank line after*, which cpplint rejects.
  Fix: use conventional **grouped** visibility sections (one `public:`, one
  `private:` per class) instead of a label per member — sidesteps the conflict
  entirely, don't reintroduce per-member labels.
- Anonymous `namespace { ... }` blocks: uncrustify wants their contents
  **indented**; cpplint's "do not indent within a namespace" rejects that
  regardless of named/anonymous. Fix: don't use an anonymous namespace for
  file-local helpers in a single-TU `.cc`/`.cpp` — just mark them `static`
  (internal linkage) at file scope instead.
- A multi-line-condition `else if (...)`: cpplint wants the block's opening
  brace attached to the end of the last condition line; uncrustify wants it on
  its own new line — direct opposite. If the two branches are provably mutually
  exclusive (check the actual conditions before assuming this applies), just
  drop the `else` and use two independent `if`s — identical behavior, and there's
  no `else` left for the tools to disagree about the bracing of.
- **After any structural rewrite, re-verify with a fresh `colcon test` rather
  than assuming a fix is complete** — this package went through ~5 build/test
  round trips after the initial compile succeeded, several of them from
  uncrustify's own reformat introducing new problems.

**A real segfault, and what it taught about `Position()`/`Velocity()`:**
`.value_or(std::vector<double>{0.0})[0]` on a `gz::sim::Joint::Position()`/
`Velocity()` result is **undefined behavior** if the returned optional is
*present but contains an empty vector* — `value_or()` only substitutes its
fallback when the optional itself is `nullopt`, not when it holds an empty
vector. gdb confirmed this: fault address squarely inside the plugin's own
`PreUpdate` (not inside gz-sim), the shape of an inlined out-of-bounds vector
index. Fix pattern used everywhere now: a small `FirstOrZero()` helper that
checks `values && !values->empty()` before indexing `[0]`. **Apply this pattern
to any future `Joint::Position()`/`Velocity()` read** — don't reintroduce the
unsafe `.value_or(...)[0]` shortcut.

**Testing a gz-sim `System` plugin:** use `gz::sim::TestFixture` (from
`<gz/sim/TestFixture.hh>`) with a minimal hand-written SDF world under
`test/worlds/*.sdf`. Key mechanics that worked:
- Point `GZ_SIM_SYSTEM_PLUGIN_PATH` at the plugin's own build output directory
  via `setenv()` in the test binary itself (a file-scope `static` struct whose
  constructor calls `setenv`, using a `PLUGIN_BUILD_DIR` compile definition set
  via CMake's `$<TARGET_FILE_DIR:...>` generator expression) — this makes the
  test self-contained, not dependent on the package's environment hook having
  been sourced (it hasn't, straight out of the build tree before install).
- `TestFixture` only gives black-box `EntityComponentManager` access via
  `OnPostUpdate`/etc. callbacks — there's no way to get a pointer to the loaded
  plugin instance itself, so you can't call its public C++ API directly from a
  test. Only default/SDF-driven behavior is testable this way.
- **Anchor any test-world model that shouldn't be moving to `world` via a fixed
  joint** (`<joint type="fixed"><parent>world</parent><child>...</child></joint>`).
  Forgetting this cost a full debug cycle: an unconstrained model free-falls
  under gravity for the whole test run, and that free-fall couples into any
  attached joint as spurious motion, confounding a test that's meant to check
  controller behavior, not rigid-body dynamics.
- A joint's `Position()`/`Velocity()` will only ever be populated if something
  called `EnablePositionCheck`/`EnableVelocityCheck` on it — check what the
  *plugin itself* actually enables (not what seems obviously relevant) before
  writing a test assertion that reads either one; a continuously-rotating wheel
  joint's plugin only enables velocity, never position, for example.

## `drcsim_model_resources` — design decisions and lessons (done, keep as reference)

**AtlasSimInterface risk was overestimated in the original plan.** Only v1.1.1 ships
a real prebuilt proprietary binary (a stripped ELF `.so` from ~2013); v2.10.2/v3.0.2
are pure open-source "shim" source (`src/AtlasSimInterface.cc`, byte-identical across
all 3 versions, confirmed via `diff`) that OSRF itself wrote as a stand-in, no
proprietary content, no ABI risk. Decision (confirmed with the user): build the shim
for **all three** versions, don't attempt the real v1 binary at all — zero
proprietary-binary-linking risk, matches what v2/v3 always did anyway. Built as
proper exported ament targets (`AtlasSimInterface1/2/3`, matching the library names
`drcsim_gazebo_ros_plugins`'s CMakeLists.txt already expects for Tier 2) via
`ament_export_targets`/`ament_export_include_directories`, so Tier 2 can
`find_package(drcsim_model_resources)` the modern way instead of the old catkin
`CFG_EXTRAS` variable convention.

**Bulk-installed worlds/models/media** (same pattern as description packages):
removed all 134 nested per-model/per-world `CMakeLists.txt` files (every one was a
plain `install(FILES ...)`, confirmed via grep for `add_library`/`add_executable`/
`configure_file`/`execute_process`/`add_custom` — none had custom logic) in favor of
three `install(DIRECTORY ...)` calls. `GZ_SIM_RESOURCE_PATH` hook needed **two**
entries here (`.../gazebo_models` and `.../worlds`), not just `share` like the
description packages — confirmed via grep that world files use bare `model://golf_cart`
URIs (no package-name prefix), so the resource path has to include the
`gazebo_models` directory itself.

**`ament_cpplint`/`ament_uncrustify` EXCLUDE wants file paths, not directory
paths** — passing the 3 vendored `AtlasSimInterface_*/` directories directly was
silently ignored (files inside them still got scanned and flagged; confirmed from
real test output). Fix: `file(GLOB_RECURSE ...)` to enumerate every actual file
under those directories, then pass that list to `EXCLUDE`. And once truly excluded,
both tools **error on zero remaining files to check** rather than passing trivially
(confirmed: `ament_uncrustify` returned "No files found" / exit 1) — this package
has no first-party C++ source at all once the vendored shim is excluded, so both
linters were dropped entirely rather than fought.

**`gz sdf --check` run standalone can never resolve `model://` URIs, with or
without `GZ_SIM_RESOURCE_PATH` set** — confirmed by testing both ways, identical
failures either way. The error is explicit about why: `sdf::findFile()`'s
URI-resolution callback is only ever registered by `gz-sim`'s own runtime server at
startup; the generic `sdf` CLI tool never wires one up under any circumstance. Every
`<include><uri>model://...</uri></include>` a file has reliably fails with
`Error Code 14: ... Unable to find uri[...]`, regardless of whether the referenced
model genuinely exists. **This is a permanent limitation of checking one file in
isolation, not something fixable from the test** — `test_sdf_files_check.py` parses
`gz sdf --check`'s stderr for `Error Code N:` markers and tolerates *only* code 14
(plus, only when 14 is present, codes 17/25 — see below), failing on any other code
as a genuine problem. Don't try another env var or flag to "fix" code 14 — it's been
tried.

**Pure nested-model wrappers cascade error 14 into 17/25.** A model that's just
`<include>` of an external model + a `<plugin>` (e.g. `drc_vehicle`/
`drc_vehicle_xp900`, composing in `polaris_ranger_ev`/`xp900` and attaching vehicle
control) can't have its own links counted once the include fails to resolve — the
checker can't see inside an unresolved include to find the links it would
contribute, surfacing as `Error Code 17` ("must have at least one link") and `25`
(frame graph) *alongside* 14. The test tolerates 17/25 **only when 14 is also
present** (confirming that root cause) — a file with 17/25 and no 14 is a genuine
standalone bug (this is exactly what caught `block_angle_steps`/`block_level_steps`
as real bugs, see below).

**Real pre-existing content bugs found and fixed** (all confirmed via actual
`gz sdf --check` output before fixing, not guessed) — useful as a checklist if more
turn up when Tier 3's `drcsim_gazebo` world-heavy testing happens:
- `<script><name>X</name></script>` with no `<uri>` (Error 9) — add
  `<uri>file://media/materials/scripts/gazebo.material</uri>` for standard
  `Gazebo/*` material names (this repo's own established convention, confirmed via
  grep against files that already had it right).
- `<script><uri>.../scripts</uri><uri>.../textures</uri></script>` with no `<name>`
  (Error 8) — check the model's own `materials/scripts/*.material` file for the
  actual `material <name>` line rather than guessing.
- A model composed purely of `<include>`s with no link of its own, and genuinely
  static (Error 17/25, **without** 14 present) — add `<static>true</static>`,
  matching a sibling model with the identical pattern if one exists in the repo
  (`block_angle_base` was the working precedent for `block_angle_steps`/
  `block_level_steps`).
- Root element `<gazebo version='1.2'>...</gazebo>` (Error 1/40) — the
  pre-SDFormat Gazebo 1.x XML schema, from before SDF existed as its own spec.
  Convert to `<sdf version="1.4">...</sdf>` (add the `<?xml version="1.0"?>`
  prolog too if missing).
- Stray/duplicate/mismatched closing tags (Error 1, XML parse errors) — e.g. an
  extra `</link>` after a `<plugin>` block, an orphaned `</physics>` with no
  matching open tag, a duplicated `</model>` at file end. **XML parsers stop at the
  first fatal error** — fixing one such bug can unmask a second, previously-hidden
  one in the same file (happened with `vrc_standpipe/model.sdf`); re-parse after
  each fix rather than assuming one fix means the file's done.
- Literal leftover text after the document's closing tag (e.g. the word `Success`
  pasted in after `</gazebo>`) — "junk after document element."
- Inertia tensor violating the triangle inequality every real rigid body must
  satisfy (`izz <= ixx + iyy`, etc.) (Error 19, "invalid inertia") — e.g.
  `ixx=0.1, iyy=0.1, izz=1.0` (1.0 > 0.2). Reduce the offending value to something
  physically valid; for small decorative sub-parts with placeholder mass anyway, a
  symmetric inertia (all three equal) is a safe default.
- Sibling `<collision>`/`<visual>` elements sharing one literal `name` (Error, "Non-
  unique names detected") — give the collision a distinct name (e.g. append
  `_collision`), leaving the visual name as-is, matching how other pairs in the same
  link were already named uniquely.

## Established per-package conversion pattern (message packages)

- `package.xml`: format 2 → 3, correct element order matters for `xmllint`:
  `name, version, description, maintainer, license, url, author, depends...`
  (license/url *before* author — got this wrong once, xmllint caught it).
- `catkin` → `ament_cmake`; `add_message_files`/`generate_messages` →
  `rosidl_generate_interfaces`; `message_generation`/`message_runtime` →
  `rosidl_default_generators`/`rosidl_default_runtime`.
- ROS 1's bare `Header header` → must be fully qualified `std_msgs/Header header`
  (ROS 2 dropped the shorthand) — add `std_msgs` as an explicit dependency.
- ROS 1's bare `time`/`duration` builtin types don't exist in ROS 2 — grep for
  `^time ` / `^duration ` and replace with `builtin_interfaces/Time` /
  `builtin_interfaces/Duration` (+ dependency). Fails at CMake time with an
  opaque `KeyError: 'time'`, not a clear per-field error.
- ROS 2 message/field names must be lower snake_case (no camelCase) — rename and
  flag any downstream C++ consumers that used the old field names. This bit
  `IRobotHandPlugin.cpp` in `drcsim_gazebo_ros_plugins` (Tier 2) when that
  plugin was finally ported — its rewrite uses the already-snake_cased
  `handle_msgs` fields throughout, see the Tier 2 section above.
- ROS 2 **constant** names (a `TYPE NAME = value` line, as opposed to a plain
  field) must be **UPPER_CASE** — this is a hard rosidl parse error, the mirror
  image of the field-name rule above. grep every `.msg`/`.srv`/`.action` for
  `= <number>` lines and check both cases: lowercase constants need
  uppercasing, uppercase-containing *fields* (no `=`) need lowercasing.
- Add a gtest that constructs, serializes, and round-trips each message/service
  type via `rclcpp::Serialization<T>` — needs both
  `<rclcpp/serialization.hpp>` **and** `<rclcpp/serialized_message.hpp>` (the first
  alone leaves `SerializedMessage` an incomplete type).
- Add a copyright header (Apache-2.0 block, matches this repo's existing convention
  despite some `package.xml`s saying `license: BSD`) to any new source file, or
  `ament_copyright`/`cpplint` will fail.

## Established per-package conversion pattern (description/xacro packages)

- Same `package.xml`/`CMakeLists.txt` catkin→ament_cmake conversion.
- Drop `make_standalone_models()`/`tools/URDF_helpers.cmake` (old non-ROS
  `GAZEBO_MODEL_PATH` standalone-model tree) — add an `ament_environment_hooks()`
  call with a `hooks/<pkg>.dsv.in` file containing
  `prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share` instead, so gz-sim can find
  `model://` resources.
- `$(find package)/...` inside `xacro:include filename=` **does still work** in
  ROS 2 Jazzy's xacro (verified empirically — don't assume it's broken just because
  an old `manifest.xml` comment claims it needs rospack; that comment is stale).
- `package://pkg/meshes/...` mesh URIs are still fine as-is in URDF for ROS 2.
- Remove dead nested `CMakeLists.txt` files (`meshes/CMakeLists.txt`,
  `materials/CMakeLists.txt`, etc.) — the new top-level `install(DIRECTORY ...)`
  installs those dirs wholesale and never `add_subdirectory()`s into them, so
  `ament_lint_cmake` flags the orphaned files.
- Convert any `launch/*.launch` (roslaunch XML) to `launch/*.launch.py`
  (`robot_state_publisher` node + `xacro` via `Command` substitution is the
  standard pattern for a bare "upload"/"visualize" launch file; use
  `ros_gz_sim`'s `gz_sim.launch.py` include + `ros_gz_sim create` node for a
  "spawn into a running world" launch file).
- Add a pytest (`ament_cmake_pytest`) that runs `xacro` on every top-level
  `robots/*.urdf.xacro` and structurally validates the output (well-formed XML,
  root is `<robot>`, every joint's parent/child link is actually defined) — this
  once caught a real pre-existing bug (missing `<link name="world"/>`).
- A `<link name="parent_name"/>` must exist for every joint's `parent`/`child`
  attribute, including conventionally-implicit ones like `"world"` — check for
  precedent elsewhere in the repo (e.g. `robotiq_hand_description` already did this
  correctly) before assuming a name like `"world"` is auto-provided.

## Lint gotchas that cost a round-trip each (avoid repeating)

- `ament_pep257` wants multi-line docstrings to start with a **blank first line**
  after the opening `"""` (D213 convention) — `"""Summary.\n\nDetails."""` fails;
  `"""\nSummary.\n\nDetails."""` passes.
- `ament_flake8`'s import-order plugin sorts **alphabetically by full module path**
  across both `import X` and `from X import Y` styles mixed together (e.g. `pathlib`
  sorts before `subprocess` sorts before `xml.etree.ElementTree`).
- flake8-quotes (Q000) prefers single quotes; only use double quotes for a
  specific string *literal* that itself contains an apostrophe — don't let that
  spread to sibling lines in a multi-line f-string/concatenation.
- `cpplint` wants namespace contents at the **same indentation as the `namespace`
  keyword** (no extra indent level for being inside a namespace), the header guard
  as `PKGNAME__FILENAME_HPP_` (double underscore), a trailing
  `#endif  // PKGNAME__FILENAME_HPP_` / `}  // namespace pkgname` comment, no blank
  line after inline `public:`/`private:` labels, `using foo::Bar;` instead of
  `using namespace foo;`, and `} else if (` on one line (not `}\nelse if (`).
- **Name any `.cc`/`.cpp` file that uses an explicit template call
  (`std::make_shared<T>()`, `create_client<T>()`, `Component<T>()`, ...)
  `.cpp`, never `.cc`** — `ament_uncrustify` misparses these as comparison
  expressions in `.cc` files specifically (see the Tier 2 section's
  "`.cc` vs `.cpp`" writeup for the multi-day version of this one-liner).

## Runtime gotcha found while porting `VRCPlugin`: `declare_parameter()` is one-shot

ROS 1's `NodeHandle::getParam()` can be called for the same name any number
of times; ROS 2's `Node::declare_parameter()` **throws
`rclcpp::exceptions::ParameterAlreadyDeclaredException` the second time it's
called for the same name on the same node**, full stop. This bit `VRCPlugin`
twice in the same file, in two different shapes:
- A per-joint parameter name built from a runtime-resolved string
  (`"atlas_controller.gains." + jointName + ".p"`) collided whenever two
  loop iterations produced the *same* string — here, whenever
  `AtlasCommandController::FindJoint()`'s "neither candidate name exists on
  this robot version" case returned an empty string for more than one DOF,
  every one of those collided on `"atlas_controller.gains..p"`. Fix: skip
  declaring anything for a name that turned out empty/invalid rather than
  assuming every loop iteration produces a distinct key.
- A **fixed** parameter name declared from inside a per-tick code path
  (`UpdateStates()`'s state machine) rather than a one-time load path — it
  worked on the first tick and threw on the second, since the surrounding
  `if`/`else if` state-machine branch it lived in doesn't necessarily
  transition away after just one tick. Fix: move any `declare_parameter()`
  call into a function that runs exactly once (`Load()`/`DeferredLoad()`/
  `InitModel()`-style setup), cache the resolved value in a member, and
  reference the member from the per-tick code instead of re-declaring.
General rule going forward: **before adding a `declare_parameter()` call,
check that the surrounding function is guaranteed to run exactly once for
that parameter name over the plugin's lifetime** — a loop over
runtime-computed keys needs those keys to be provably distinct, and any
call site inside a per-`PreUpdate`/per-tick path is almost certainly wrong
unless the name itself changes every time (which parameter names never
should).
