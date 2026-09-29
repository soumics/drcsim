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

8. ✅ `VRCScoringPlugin` — **done, 506/506 package checks passing (2
   pre-existing, unrelated errors in `drcsim_model_resources`'s
   cpplint/uncrustify result files, not this package), merged into
   `ros2-jazzy-harmonic`.** A `WorldPlugin` (~1300 lines across header +
   source) that implements the VRC/qualifier scoring algorithms (gate
   crossings, vehicle entry, drill-in-bin, fire hose docking/connection/
   valve) and writes a running score log plus a latched `vrc_score` ROS
   topic. By far the simplest Tier 2 plugin so far — it only ever *reads*
   simulation state (poses, velocities, joint positions, collision
   geometry) and publishes; it never mutates the ECM except once, to flip
   on velocity checks for Atlas's head link.
   - **No thread, no mutex, unlike every plugin since `VRCPlugin`.** The
     original defers most setup to a background thread that polls for the
     `atlas` model once a second and blocks until it appears. That
     defensive wait-for-atlas behavior is preserved, but implemented by
     retrying the lookup once per `ISystemPreUpdate` tick instead of a
     sleeping thread — this plugin has no incoming ROS callbacks at all
     (only an outgoing publisher), so there's no cross-thread ECM access
     to guard against and no pending-action queue is needed, the first
     time that's been true since `SandiaHandPlugin`/`MultiSenseSLPlugin`.
     `ISystemPreUpdate` is used for exactly one thing (the one-time
     atlas-found transition, which needs a mutable ECM for
     `Link::EnableVelocityChecks`); the real per-tick scoring work is all
     `ISystemPostUpdate`, against a `const EntityComponentManager`.
   - **Bounding boxes computed by hand from box-shaped collision
     geometry** (`components::CollisionElement`, the same component
     `VRCPlugin::CheckThreadStart` reads for the fire hose coupler's
     cylinder) rather than the newer `Link::WorldAxisAlignedBox`/
     `EnableBoundingBoxChecks` API, which wasn't verified available in
     this workspace's Harmonic-era gz-sim release. Every real caller here
     (the VRC "gate" models, the qualifier-2 "bin" model) uses box
     collisions exclusively, so this covers all actual usage.
   - **Fire hose "aligned" detection**: the original checks
     `standpipe->GetChildJointsLinks()` for a non-empty list, true once
     `VRCPlugin` creates its Classic screw joint. Since `VRCPlugin`'s own
     port replaces that screw joint with a `components::DetachableJoint`
     (a fixed weld — see `VRCPlugin.hpp`'s design notes), this plugin
     detects the same event by scanning the ECM for a `DetachableJoint`
     whose `parentLink` is the standpipe link. Same downstream
     consequence as noted for `VRCPlugin`: the connection can't be
     "unscrewed", so `CheckHoseAligned` will never see a transition back
     to unaligned in practice.
   - **Vehicle seat link name changed on purpose**: the original looks up
     `"polaris_ranger_ev::chassis"` — Classic's double-colon nested-model
     deep-name lookup, which has no equivalent in gz-sim's ECS
     (`Model::LinkByName` only searches immediate children). This port
     looks up model `"drc_vehicle"` / link `"chassis"` instead, matching
     the naming `VRCPlugin`'s own port already assumes for the same
     vehicle — both plugins have to agree on how to find it, and only one
     of the two naming schemes can work under gz-sim.
   - `boost::filesystem`/`boost::lexical_cast`/`boost::algorithm::string`
     replaced with `std::filesystem` and small hand-rolled string parsing
     (gate names are a fixed, simple `"gate_<N>"`/`"vehiclegate_<N>"`
     format, not worth a string-split library for).
   - The original's `PubMultiQueue`/`PubQueue` machinery (to avoid
     blocking Classic's single update thread on a ROS 1 publish call)
     is dropped entirely — rclcpp publishers don't block the caller.
   - Test coverage: a minimal two-gate `qual_task_1` world with gravity
     zeroed out, driven through both gates by a `TestFixture::OnPreUpdate`
     callback that teleports the stand-in "atlas" model via
     `SetWorldPoseCmd` (no real dynamics involved), checking the latched
     `vrc_score` topic reports `completion_score == 2` after crossing
     both.
   - **Real bugs the build/test round actually caught**:
     (1) `sdf::Geometry::BoxShape()` returns a forward-declared `sdf::Box`
     (only `<gz/sim/components/Collision.hh>` was included) — needed a
     direct `#include <sdf/Box.hh>` before calling `.Size()` on it, the
     same gap `VRCPlugin` already hit for the analogous `CylinderShape()`
     case. (2) **The much more interesting one**: `FindGates()` (and
     every other `Find*Stuff` variant) reliably found *nothing* when
     called once, synchronously, from `Configure()` — even though the
     gate models were declared in the very same SDF world file. Turns out
     a world-scoped gz-sim system's `Configure()` runs before sibling
     `<model>` entities from that same file are necessarily constructed;
     Classic gave the opposite guarantee (`WorldPlugin::Load()` only ever
     runs once every model in the world file already exists), which is
     exactly what the original's one-shot `Load()`-time lookup relied on.
     Fixed by giving `FindArenaStuff()` the same "retry every
     `PreUpdate` tick until it succeeds" treatment already used for the
     atlas lookup — **general lesson for any future world-scoped plugin
     that looks up a sibling SDF model: never assume it exists yet inside
     `Configure()`, only from `PreUpdate` onward.**

9. ✅ `DRCVehicleROSPlugin` — **done, 518/518 package checks passing across
   both `drcsim_gazebo_plugins` and `drcsim_gazebo_ros_plugins` on the
   first real attempt (2 pre-existing, unrelated errors in
   `drcsim_model_resources`'s cpplint/uncrustify result files, not
   either of these packages), merged into `ros2-jazzy-harmonic`.** A thin
   ROS wrapper (~430 lines across header + source, by far the smallest
   Tier 2 plugin) subclassing the already-ported
   `drcsim_gazebo_plugins::DRCVehiclePlugin`: subscribes to
   `<model>/{hand_wheel,hand_brake,gas_pedal,brake_pedal}/cmd` and
   `<model>/{key,direction}/cmd`, calling straight into the base class's
   existing setters, and periodically publishes the matching `.../state`
   topics. Same `VRC_CHEATS_ENABLED` gate as the original: unset (default),
   this class behaves exactly like a plain `DRCVehiclePlugin` — no ROS
   node, no topics at all (directly commanding pedals/wheel over ROS,
   bypassing physical actuation by the robot, is a "cheat").
   - **First cross-package plugin subclassing in this migration** —
     `DRCVehiclePlugin` lives in `drcsim_gazebo_plugins`, a different,
     already-"done" package. Previously nothing needed to *link against*
     a plugin package's library, only load its `.so` directly via
     gz-sim, so `drcsim_gazebo_plugins/CMakeLists.txt` had no CMake
     target export at all. Added one (`ament_export_targets`,
     `install(... EXPORT ...)`, `target_include_directories(... PUBLIC
     $<BUILD_INTERFACE:...> $<INSTALL_INTERFACE:include>)`) so
     `drcsim_gazebo_ros_plugins` can `find_package(drcsim_gazebo_plugins)`
     and link `drcsim_gazebo_plugins::DRCVehiclePlugin` properly. **Build
     note for the next round**: since this changes an already-built
     package's CMakeLists.txt/headers, `colcon build` needs to rebuild
     `drcsim_gazebo_plugins` too, not just `drcsim_gazebo_ros_plugins` —
     use `--packages-up-to drcsim_gazebo_ros_plugins` (or select both
     packages explicitly) for this round, not a bare `--packages-select
     drcsim_gazebo_ros_plugins`.
   - **Added `DRCVehiclePlugin::IsValidConfig()`** (previously private,
     no accessor) so this subclass can replicate the original's `try {
     DRCVehiclePlugin::Load(...) } catch (...) { return; }` — skip
     standing up the ROS interface entirely if the base plugin failed to
     configure. gz-sim doesn't use exceptions for this class of failure
     (logs via `gzerr` and returns instead — see `DRCVehiclePlugin`'s own
     design notes), so a boolean getter is the direct equivalent.
   - **Mutex added around the base class's plain cached command fields**
     (`handWheelCmd`, `gasPedalCmd`, `directionState`, ...) that the
     original left genuinely (if benignly) racy: ROS's callback-queue
     thread wrote these `double`/enum members directly, Classic's physics
     thread read them in `OnUpdate()`, no lock either side, tolerated
     because a torn `double` read/write is harmless in practice. Rather
     than preserve that race, every ROS callback and the `PreUpdate` call
     into the base class are wrapped in one `cmdMutex` here, matching
     this migration's established pattern (`AtlasPlugin`'s
     `controlMutex`, etc.) — a case of "no pending-action queue needed,
     but still worth a mutex" like `AtlasPlugin`, not because of any
     ECM access (there is none in the callbacks) but because of plain
     shared-state access, which the ECM-focused rule doesn't itself cover
     but the same underlying hazard applies to.
   - `ros::CallbackQueue` + polling thread replaced with the standard
     `rclcpp::executors::SingleThreadedExecutor` on its own thread,
     matching every other ROS-coupled plugin in this migration.
   - Test coverage: reuses `drcsim_gazebo_plugins`' own `vehicle_test.sdf`
     vehicle model (11 joints), swapping in `DRCVehicleROSPlugin` as the
     model's plugin with `VRC_CHEATS_ENABLED=1`; publishes full gas via
     `<model>/gas_pedal/cmd` and checks `<model>/gas_pedal/state`
     approaches 1.0 (fully pressed) — this exercises the ROS command path
     specifically, not the underlying PID (already covered by
     `DRCVehiclePlugin`'s own test).

10. ✅ CLI executables (`port/cli_executables`) — **done, 519/519 package
    checks passing on the first attempt (2 pre-existing, unrelated
    errors in `drcsim_model_resources`'s cpplint/uncrustify result
    files, not this package), merged into `ros2-jazzy-harmonic`.** The
    last item in Tier 2. Of the 9 remaining files, 6 were **dropped**
    after checking the whole repo for any real dependency on them (none
    found), and 3 were ported as plain ROS 2 nodes (`add_executable`,
    not gz-sim plugins).
    - **Dropped, no port**:
      - `pub_atlas_state.cpp` and `pub_joint_states.cpp` were **byte-for-
        byte identical files** — a latency-benchmark scratch tool full of
        commented-out debug code whose one real `publish()` call was
        itself commented out, so neither ever actually published
        anything. Confirmed via repo-wide grep: referenced by nothing.
      - `pub_atlas_joint_trajectory_test.cpp` published to a
        `joint_trajectory` topic that nothing in this repo (or the
        dropped `actionlib_server`, checked specifically) ever
        subscribed to — an orphaned one-off test script. Also used
        pre-rename joint names (`atlas::back_lbz` style) and a stray
        `#include <gazebo/math/Quaternion.hh>` it didn't even use.
      - `gz_model_teleport.cpp` was a standalone CLI using Classic's raw
        `gazebo::transport` client API to teleport a model by name —
        gz-sim/Harmonic already exposes this directly via its own
        `gz service -s /world/<world>/set_pose --reqtype gz.msgs.Pose
        --reptype gz.msgs.Boolean --req '...'`; porting a bespoke
        wrapper around a capability the platform now ships built-in
        would add nothing.
      - `test_ros_plugin.{hh,cc}` was a **completely empty** template
        `ModelPlugin` — `Load()` does basic bookkeeping, `UpdateStates()`
        is an empty function body, no ROS coupling despite the name,
        registered but never referenced by any world/launch file in the
        repo. Nothing to port.
      - `actionlib_server.{h,cpp}` (~740 lines, by far the largest of the
        9) implements a `WalkDemo` actionlib server driving Atlas via
        `AtlasSimInterfaceCommand`/`AtlasSimInterfaceState` — exactly the
        "BDI walk" mechanism `AtlasPlugin`'s own port already confirmed
        is an inert shim (every behavior function a no-op, `NONE`
        reported as current behavior, **even in the original
        codebase**). Porting it would faithfully reproduce a walk-goal
        interface that could never complete a walk, same as the
        original — there is no real balance controller anywhere in this
        codebase to eventually back it with. Its only repo references
        were two **not-yet-ported** Tier 3 launch files
        (`drcsim_gazebo/launch/keyboard_teleop.launch`,
        `drcsim_gazebo/test/vrc_task_1_commander.launch`), which will
        simply not reference it once those get ported. Its dependencies
        (`actionlib`, `tf`) were never even declared in this package's
        (already-ROS2-ified) `package.xml`, meaning this file hasn't
        actually compiled since the ROS 2 conversion began — further
        confirming it was already dead weight, not a working feature
        quietly waiting for its turn.
    - **Ported, plain ROS 2 nodes** (not gz-sim plugins — ordinary
      `add_executable` targets, runnable via `ros2 run
      drcsim_gazebo_ros_plugins <name>`):
      - `pub_atlas_command.cpp` / `pub_atlas_command_fast.cpp`: drive
        every Atlas joint through an arbitrary sine-wave trajectory over
        `atlas/atlas_command`, for exercising/latency-testing
        `AtlasPlugin`'s command/state round trip. Two genuinely distinct
        modes preserved from the original: `_fast` computes and
        publishes synchronously inside the `atlas_state` callback itself
        (tighter loop, round-trip-time testing); the plain version uses
        a separate worker thread decoupled from the callback (more
        representative of an external controller on its own schedule).
        `boost::thread`/`boost::mutex` → `std::thread`/`std::mutex`;
        `ros::Time`/`getParam` → `rclcpp::Time`/`declare_parameter`,
        otherwise a direct translation.
      - `pub_joint_commands.cpp`: same idea over the lower-level
        `osrf_msgs/JointCommands` topic (`AtlasPlugin::SetJointCommands`)
        instead. Per-joint PID gains are read from the same
        `atlas_controller.gains.<joint>.{p,i,d,i_clamp}` parameters
        `AtlasPlugin` itself loads (`LoadPIDGainsFromParameter`) — pass
        a matching `--ros-args --params-file` for these to be anything
        but zero. **Joint names updated from the original's stale,
        pre-rename `"atlas::back_lbz"`-style names to the current
        canonical bare names** (`back_bkz`, etc., matching
        `AtlasPlugin::Load`'s own preferred names) — functionally
        harmless either way since `SetJointCommands` matches arrays
        purely by length, never by the message's `name` field, but the
        old names would have pointed the parameter lookup at nonexistent
        parameter keys.

**Tier 2 complete.** All of `drcsim_gazebo_ros_plugins` is now ported:
ContactModelPlugin ✅, SandiaHandPlugin ✅, IRobotHandPlugin ✅,
RobotiqHandPlugin ✅, MultiSenseSLPlugin ✅, VRCPlugin ✅, AtlasPlugin ✅,
VRCScoringPlugin ✅, DRCVehicleROSPlugin ✅, CLI executables ✅.

## Tier 3 — `drcsim_gazebo` (✅ first deliverable done, 537/537 package checks
passing across `atlas_description`, `drcsim_gazebo_plugins`,
`drcsim_gazebo_ros_plugins`, `drcsim_model_resources`, `drcsim_gazebo`,
merged into `ros2-jazzy-harmonic`. `ros2 launch drcsim_gazebo
atlas.launch.py` brings up Atlas end to end: gz-sim starts, `VRCPlugin`
spawns the robot from `robot_description`, and `AtlasPlugin` runs under
the real per-joint PID gains — confirmed both automatically (querying
`atlas_plugin`'s own ROS parameters directly in `test_atlas_launch.py`,
not by trying to model what "correct" standing dynamics should look
like) and by hand: **the user watched Atlas actually stand in the
gz-sim GUI** on 2026-09-09, after two real bugs found via that same
interactive testing got fixed — see below.)
>
> **Verified by direct measurement (2026-09-28)**: this claim is correct —
> the real gains *are* applied. An earlier "correction" here said Atlas
> stood under zero active torque; that was wrong and has been retracted.
> `ZeroAtlasCommand()` does set `k_effort=0`, but
> `LoadPIDGainsFromParameter()` runs right after it and sets `k_effort=255`
> for every joint. Measured at rest via `atlas/atlas_state`: `k_effort=255`,
> setpoint = all zeros, and real holding torques (knee ≈ −59 N·m, hip pitch
> ≈ −81 N·m, ankle pitch ≈ −72 N·m) produced by a ~0.03 rad gravity sag
> below that setpoint (these gains have no integral term, so the sag *is*
> the holding error). See the `atlas_walking_demo` section, bug #8.

The goal of this tier: prove the *whole* simulation actually comes up
together (spawn Atlas, get it to hold a pose instead of collapsing —
the user's original "will it fall?" question), not mechanically port
all 64 launch files / 194 rostest files / 15 config YAMLs / 25 scripts
in `drcsim_gazebo`. Two research agents surveyed the whole package
first; see the plan file this branch was built from
(`.claude/plans/i-have-forked-this-optimized-lobster.md` at the time,
though that file gets reused for later plans too — this section is the
durable record).

**The root cause of "Atlas collapses even with `AtlasPlugin` fully
working" turned out to be infrastructure, not `AtlasPlugin` itself.**
Every ROS-coupled plugin (`VRCPlugin`, `AtlasPlugin`, `VRCScoringPlugin`,
`DRCVehicleROSPlugin`) constructs its internal `rclcpp::Node` via
`rclcpp::init(0, nullptr)` and default `NodeOptions()` — since none of
them are started via `ros2 run` (they're gz-sim System plugins loaded
as `.so`s into gzserver's single process, with no access to that
process's real `argv`), **none of them could ever receive parameters
from a launch file at all.** Not `robot_description` (already read by
`VRCPlugin::Robot::InsertModel`, which spawns it via `LoadSdfString` +
`CreateEntities` — already fully implemented, just never fed real
input), not `robot_initial_pose.*`/`atlas.startup_mode` (also already
read by `VRCPlugin`), and not
`atlas_controller.gains.<joint>.{p,i,d,i_clamp}` (already read by
`AtlasPlugin::LoadPIDGainsFromParameter`, silently defaulting to
**zero** every time) — every one of these already-correct
`declare_parameter(name, default)` calls was just never getting fed
anything but its own default.

- **Fix: `RosNodeOptionsFromEnv()`**
  (`drcsim_gazebo_ros_plugins/include/drcsim_gazebo_ros_plugins/
  RosNodeOptions.hpp`, new, ~15 lines). If the `DRCSIM_ROS_PARAMS_FILE`
  environment variable is set (by a launch file, in gz-sim's own
  environment, before it starts), every plugin's node is constructed
  with `NodeOptions().arguments({"--ros-args", "--params-file", path})`
  instead of default options. One shared YAML can hold a top-level
  section per node name (`vrc_plugin`, `atlas_plugin`,
  `vrc_scoring_plugin`, `drc_vehicle_ros_plugin`) simultaneously — each
  node only picks up its own matching section. Env-var-based, not an
  SDF element, matching the existing `VRC_CHEATS_ENABLED` precedent
  (`VRCPlugin`/`DRCVehicleROSPlugin` already use an env var for
  plugin-level config) rather than requiring the static world SDF to be
  templated/regenerated per launch. Without the env var set (gtests, a
  bare `gz sim world.sdf`), this is exactly the old default-constructed
  `NodeOptions` — unchanged behavior. All four plugins' `.cpp` files get
  the same ~2-line call-site change right before their
  `std::make_shared<rclcpp::Node>(...)`.
- **`atlas_description` fixes** (small, but nothing spawns without
  them — this is the actual `AtlasPlugin` attachment point, since the
  world file itself spawns no robot at all): `urdf/atlas{,_v3,_v4,
  _v4_no_wry2,_v5}.gazebo` still had Classic-era
  `<plugin name="atlas_plugin" filename="libAtlasPlugin.so"/>` (and
  per-version `libAtlasV3Plugin.so`/`libAtlasV4Plugin.so` — the Classic
  original had *separate* plugin classes per Atlas version; this port
  unified them into one `AtlasPlugin` that reads `atlas_version` as a
  parameter instead, so all five `.gazebo` files now point at the same
  `filename="AtlasPlugin" name="drcsim_gazebo_ros_plugins::AtlasPlugin"`)
  plus a dead `libgazebo_ros_joint_pose_trajectory.so` plugin reference
  (no gz-sim equivalent, no consumer anywhere in this repo) and an
  already-Classic-disabled commented-out `gazebo_ros_controller_manager`
  block — both dropped, not ported. Also dropped: the `<xacro:include
  .../atlas*.transmission" />` line from all 25 `robots/*.urdf.xacro`
  variants, and the 5 `.transmission` files themselves — confirmed dead
  `pr2_mechanism`-era transmission stubs; nothing in this port's
  architecture (`AtlasPlugin` is the whole-body controller, not a
  `ros2_control` hardware interface — confirmed via a research agent
  that `gz_ros2_control` isn't even installed in this environment, and
  nothing in `atlas_description` had any `<ros2_control>` tags to begin
  with) consumes `ros2_control` transmissions.
- **`drcsim_model_resources/worlds/atlas.world` fixes** (found during
  this work, not originally scoped in the plan — but blocking, so fixed
  anyway): this world predates Tier 2 (it was ported in Tier 0, before
  the plugins it references existed), so it still had
  `filename="libVRCPlugin.so" name="vrc_plugin"` /
  `filename="libVRCScoringPlugin.so" name="vrc_scoring"` (Classic
  names, not this port's `GZ_ADD_PLUGIN_ALIAS` names) — updated to
  `filename="VRCPlugin" name="drcsim_gazebo_ros_plugins::VRCPlugin"`
  and the `VRCScoringPlugin` equivalent, matching every hand-authored
  test world elsewhere in this migration. It also had **zero gz-sim
  system plugins at all** (confirmed: none of the 49 worlds in
  `drcsim_model_resources` do — Tier 0 only touched SDF-version/URI-path
  cleanliness, not this) — added explicit `Physics`,
  `SceneBroadcaster`, `UserCommands`, `Sensors`, `Contact` system
  plugin tags, matching what every test world elsewhere in this
  migration already does explicitly rather than relying on gz-sim's own
  default-server-config fallback. The pre-existing `<physics
  type="ode">` block with nested `<simbody>` params (structurally
  confused, but present since before this migration and not confirmed
  to actually break parsing) was left alone — out of scope here.
- **Real gains**: `drcsim_gazebo/config/atlas_v5_gains.yaml`, converted
  from the original's `config/whole_body_trajectory_controller_v5.yaml`
  `gains:` block (same 30 joints incl. both `wry2` wrists, same P/D
  values) into a flat-dotted-key ROS 2 params file for the
  `atlas_plugin` node (`atlas_controller.gains.<joint>.p: <value>`,
  etc. — matches `LoadPIDGainsFromParameter`'s built parameter names
  character-for-character; flat dotted keys chosen over nested YAML
  purely to keep 120 values easy to eyeball-verify against the
  original, not for any parsing reason). The original's `joints:`/
  `type: robot_mechanism_controllers/JointTrajectoryActionController`/
  `joint_trajectory_action_node:` machinery has no equivalent —
  `AtlasPlugin` is already the whole-body controller, so only the gain
  *values* carried over.
- **`drcsim_gazebo/launch/atlas.launch.py`** (new): replaces the
  original's `atlas.launch` → `atlas_no_controllers.launch` →
  `atlas_bringup.launch` → `atlas_v5_bringup.launch` chain for the
  default (v5, no hands) case. Renders `atlas_v5.urdf.xacro` via the
  `xacro` Python API, merges the result plus `robot_initial_pose.*`/
  `atlas.startup_mode`/delay-window params (as a `vrc_plugin` params
  section) with `atlas_v5_gains.yaml`'s `atlas_plugin` section into one
  combined YAML written to a temp file, sets `DRCSIM_ROS_PARAMS_FILE`
  to that path *before* including `ros_gz_sim`'s `gz_sim.launch.py`
  (env var must be set before gz-sim's process starts), and starts
  `robot_state_publisher` (remapped `joint_states` →
  `atlas/joint_states`, matching the original's own remap) plus a
  `ros_gz_bridge parameter_bridge` for `/clock`
  (`config/clock_bridge.yaml` — needed for `use_sim_time` and for the
  Tier 2 CLI tools' `now().seconds() > 0` busy-waits to ever return).
  No separate spawner node needed — `VRCPlugin` already spawns the
  robot itself from the `robot_description` parameter.
- **Test**: `test/test_atlas_launch.py`, a `launch_testing` test that
  includes `atlas.launch.py` and — deliberately **not** checking
  world-frame pelvis height (no world→robot localization exists in
  this architecture; `robot_state_publisher` only knows the robot's own
  joint-driven TF tree) — asserts every joint reported on
  `/atlas/joint_states` stays within 0.5 rad of its 0.0 default target
  over 60s. With no `atlas_command` publisher running,
  `AtlasPlugin::ZeroAtlasCommand()`'s target is 0.0 for every joint; if
  `DRCSIM_ROS_PARAMS_FILE` failed to deliver real gains, heavily-loaded
  joints (hips, knees) would droop well past this tolerance under
  gravity within a few seconds. This is the concrete, automated version
  of "does it stand."
- **Confirmed dead, deleted** (repo-wide dependency check, same bar as
  Tier 2's CLI-executable triage): `src/bdi_parser.cpp` +
  `cmake/SearchForTinyXML.cmake` (no `.cfg` input file exists anywhere
  in the workspace, not invoked by any build path, its own comment
  already claimed it was disabled); all 12 non-`whole_body_trajectory`
  `config/*.yaml` files plus `whole_body_trajectory_controller{,_v3,
  _v4,_v4_no_wry2,_v5}.yaml` (superseded `pr2_controller_manager`
  control architecture, fully replaced in function by
  `atlas_v5_gains.yaml` above); the 9 `run_gzserver*`/`run_gazebo*`
  shell scripts (Classic-only `libgazebo_ros_api_plugin.so`, superseded
  by `gz_sim.launch.py`'s own process management); `scripts/
  reset_pose.py` (per-joint-controller topics, dead — see above);
  `scripts/{keyboard_teleop,atlas_commander,qual_1_bridge,5steps,
  stand}.py` (all depend on the `WalkDemoAction` actionlib server
  already confirmed dropped in Tier 2 as an inert BDI shim).
- **Deferred, left in place untouched** (not confirmed dead, just not
  yet ported — same "leave it for its own turn" treatment every other
  tier's un-ported files got): the other ~60 `launch/*.launch` files
  (same template as `atlas.launch.py`, different `world`/pose/
  hand-suffix/version args — trivial to add on top when needed, not
  worth 60 near-duplicate ports now); the hand-variant bringups
  (sandia/irobot/robotiq state publishers + per-hand stereo pipeline);
  the 194-file rostest suite and its helper infra
  (`ros_api_checker`, `multicamera_subscriber`, `scoring_checker`,
  `gzlog_stop_checker.py`, `meanvar.py`, the per-task scoring-test
  scripts); `scripts/{arm_teleop,leg_teleop,nanokontrol,
  orientation_checker,performance_test1,eigen_arm,check_inertia_
  symmetry,test}.{py,bash}` and `gdbrun` (none confirmed dead —
  `arm_teleop.py`/`leg_teleop.py` in particular still target a real,
  live topic, `osrf_msgs/JointCommands`, just need a `rospy`→`rclpy`
  rewrite). All of this is stretch-phase follow-up, per the original
  top-level migration plan's own "Phase E (optional)" framing — noted
  here so it's a documented backlog, not a silent gap.
- **Real bugs the build/test round actually caught** (several rounds,
  first attempt was far from clean — this was the largest, most
  cross-cutting branch in the migration, spanning 5 packages):
  - **`--` inside an XML comment is illegal** (only allowed immediately
    before the closing `-->`) — used as a prose dash in a design-note
    comment added to `atlas.gazebo`, which broke `xacro`'s XML parser
    specifically (`not well-formed (invalid token)`) while gz-sim's own
    `sdformat` parser tolerates it silently. That asymmetry is exactly
    why this had never surfaced before: **five other test worlds from
    earlier plugins had the same bug sitting unnoticed** (found via a
    repo-wide `xmllint --noout` sweep once the pattern was identified)
    — `vrc_plugin_test.sdf`, `sandia_hand_full_test.sdf`,
    `irobot_hand_missing_joints_test.sdf`,
    `drc_vehicle_ros_plugin_test.sdf`, `vehicle_test.sdf`, all fixed.
    **General lesson: never write `--` as a prose dash in an SDF/URDF/
    XML comment** (or run new/edited SDF-family files through `xmllint
    --noout` before considering them done) — the mistake is invisible
    until something uses a strict parser on that exact file.
  - `atlas.world` relied on `<include><uri>model://sun|ground_plane
    ></uri></include>`, which needs Gazebo Fuel to resolve at load
    time — fails with no internet access. Inlined both (content matches
    gz-sim's own shipped `empty.sdf`). **General lesson: every
    hand-authored/edited world in this migration needs to be fully
    self-contained** (no Fuel-hosted `model://` includes), consistent
    with the design already used everywhere else, but this file
    predates that pattern being established.
  - `ament_lint_cmake()` does **not** take an `EXCLUDE` keyword the way
    `ament_copyright`/`ament_cpplint`/`ament_cppcheck`/
    `ament_uncrustify` all do (confirmed the hard way — passing
    `EXCLUDE <paths>` just lints "EXCLUDE", silently skipped as a
    nonexistent path, plus every path after it, the opposite of what
    was intended). It takes a plain list of paths *to check*; point it
    at just the package's own `CMakeLists.txt` to scope it the same way
    the other linters get scoped via `EXCLUDE`.
  - The combined gz-sim server+GUI process (`gz sim` with no `-s`) can
    die almost immediately with no display and no diagnostic beyond
    "process has finished cleanly" — zero simulation ever ran, zero
    topics ever published. Any `launch_testing` test that boots gz-sim
    needs an explicit headless/server-only path; `atlas.launch.py` now
    exposes this as a `headless` launch argument rather than hardcoding
    either choice.
  - `colcon test` **does not reconfigure/rebuild** — a `CMakeLists.txt`
    fix doesn't take effect until the next `colcon build`. Cost two
    redundant round-trips before the pattern was obvious; worth
    remembering for any future CMake-only fix.
  - Test flakiness under heavy combined load: running all 5 touched
    packages' test suites together (several of which spin up real
    `gz-sim` processes) intermittently produced a timing-sensitive
    `AtlasPlugin` gtest failure and a `ros_gz_bridge parameter_bridge`
    SIGABRT-on-shutdown race — both third-party/timing-sensitive, not
    code bugs, confirmed by a clean rerun of just the two affected
    packages immediately after. Worth knowing this environment can
    produce this class of flake under contention, rather than assuming
    a failure here is always a real regression.
  - **`atlas.world`'s `<pin_link>utorso</pin_link>` was simply wrong for
    the v5 model it's paired with here**, found by actually watching the
    robot in the GUI (not caught by any test): the user reported "flew
    up and then fell down" on the first real interactive run. Root
    cause, confirmed via the joint chain in `atlas_v5_simple_shapes.urdf`
    and `robot_state_publisher`'s own "root link pelvis" message:
    `pelvis` is the URDF's actual free-floating root; `utorso` is three
    real revolute joints up the spine (`pelvis -> back_bkz -> ltorso ->
    back_bky -> mtorso -> back_bkx -> utorso`), so gz-sim rejected the
    pin-to-`utorso` request outright (logged: `child link already has a
    parent joint of type [RevoluteJoint]`). With the pin silently never
    taking hold, the robot spent its "pinned" phase weightless (gravity
    compensation was still applied) but completely unconstrained, so
    `VRCPlugin`'s stand-prep PID commands — with nothing to react
    against — flung the whole free-floating body around. Fixed:
    `<pin_link>pelvis</pin_link>`. Some of the original repo's own
    later, VRC-Finals-era worlds (`vrc_final_task11-15`, `vrc_task_3*`,
    `qual_task_2`) already correctly used `pelvis`; this is a
    pre-existing inconsistency in the original repo (Classic's own
    kinematic convention changed between early and later Atlas
    versions), not something this port introduced. ~36 other
    Classic-era worlds likely have the same stale `utorso` value —
    deferred, since none of them are wired into a launch file yet.
  - **`atlas.startup_mode=bdi_stand` (the launch's original default) is
    not reliable yet, even with the pin fixed**: after fixing the
    `pin_link` bug above, a second real run still ended with the robot
    on the ground (not flying this time, just fallen over) partway
    through the `stand prep -> Nominal -> Dynamic Stand Behavior`
    sequence. Given the user's immediate goal was just seeing Atlas
    stand at all, **the launch's default `startup_mode` was switched to
    `"pinned"` instead** — the simpler pin/hold/auto-unpin path with no
    stand-prep choreography, exactly what `VRCPlugin`'s own
    already-passing gtest exercises (its test world never sets
    `atlas.startup_mode`, which defaults to `""`, also routed to this
    same simple path — `atlas.startup_mode` only ever branches on the
    literal string `"bdi_stand"`; anything else, including `""`, takes
    the "pinned" branch). **Confirmed working**: with `startup_mode=
    pinned`, Atlas spawns, holds pinned for `atlas.time_to_unpin`
    seconds, auto-unpins, and stands — watched directly in the gz-sim GUI
    under `AtlasPlugin`'s real gains (`k_effort=255`, setpoint all
    zeros — verified by direct measurement; see the correction note at
    the top of this Tier 3 section). `bdi_stand` is still selectable
    via the `startup_mode` launch argument for whoever wants to debug it
    further, but **treat it as known-unreliable, not a ready-to-use
    feature**, until someone actually root-causes the stand-prep/
    dynamic-stand transition itself (a real, currently open bug — worth
    a dedicated debugging session with the GUI actually open and watched
    step by step, not just log-reading).
  - **Correction to the note above**: the `No joint named
    [pelvis_world_pin_joint] for modelID [N]` warning was *not* harmless
    — it was the actual bug, just not visually obvious from a passive
    "is it standing" check. Running `pub_atlas_command` (the sine-wave
    diagnostic tool from Tier 2) against the "successfully standing"
    `pinned` run made it unmistakable: **the pelvis never moved at all**
    while every other joint flailed under the tool's aggressive
    commands — exactly what "still welded to the world" looks like, not
    "standing freely." Root cause, confirmed by watching it happen:
    `RemoveJoint()`'s `_ecm.RequestRemoveEntity(jointEntity)` erases the
    *ECS* entity for the world-pin joint, but gz-physics's dartsim
    plugin never actually detaches the underlying physics constraint —
    that warning is dartsim logging that it can't find a joint by that
    name to remove, and silently leaving the real weld in place forever.
    Near-certainly because "weld link to world" is normally a
    Skeleton-construction-time operation in dartsim; a *dynamically
    added* world joint (via `SdfEntityCreator::CreateEntities(&jointSdf,
    true)`, added to a model that already exists) apparently doesn't get
    registered as a normal, later-removable joint object, even though it
    *does* successfully constrain the body at creation time. This had
    never been caught before because no test in this migration ever
    exercised the *removal* path — `VRCPlugin`'s own gtest only checks
    that pinning holds, never unpins.

    **Fixed** by no longer creating that kind of joint at all:
    `VRCPlugin::EnsureWorldPinAnchor()` spawns one small, permanent,
    static "anchor" model the same way `Robot::InsertModel()` already
    spawns the robot itself (programmatic SDF + `SdfEntityCreator`, no
    world-file changes needed), and "pin to world" now welds to *that
    real link* via the same `gz::sim::components::DetachableJoint`
    mechanism already proven reliable — both to create *and to
    remove* — for every cross-model weld elsewhere in this plugin
    (fire hose docking, vehicle-seat, hand-grabs-fire-hose). `AddJoint()`
    no longer has two different code paths at all; both branches build
    a `DetachableJoint` now, differing only in which entity serves as
    the parent link. **General lesson**: dynamically creating a joint
    with parent name `"world"` via `SdfEntityCreator` after a model
    already exists in gz-sim/dartsim (Harmonic, gz-sim8) may create a
    joint that physically holds but can never be cleanly removed again
    — prefer `DetachableJoint` against a real (even trivial/static)
    anchor link for anything that needs to be un-pinned later, and
    reserve the `SdfEntityCreator`-fixed-joint-to-`"world"` approach (if
    used at all) for cases that are genuinely permanent for the life of
    the simulation.
  - **Second correction**: the `EnsureWorldPinAnchor`/`DetachableJoint`
    rework above passed the full test suite (537/537) but still failed
    for real in the interactive re-check — same symptom as before
    (upper body vibrating at spawn, then flying apart the instant
    `pub_atlas_command` ran), plus new `gz-physics` log lines the
    automated tests never surface (no GUI, so no visual + full log
    isn't captured by gtest): `Link's parent entity [N] not found on
    model map`, `DetachableJoint's parent link entity [N] not found in
    link map`, `Failed to find joint [N]`. Root cause:
    `EnsureWorldPinAnchor()` created the anchor model via
    `SdfEntityCreator::CreateEntities(&modelSdf)` but never called
    `creator.SetParent(anchorModelEntity, this->world.Entity())`
    afterward. `CreateEntities(const sdf::Model*)` builds the model's
    own entity tree but does **not** attach it to the world (only the
    `CreateEntities(const sdf::World*)` overload parents its child
    models automatically) — `Robot::InsertModel()` already does this
    `SetParent()` call right after its own `CreateEntities()`, which is
    why the robot itself spawns fine; the new anchor model was missing
    the same line. Without it, the anchor is an orphan entity gz-physics
    never registers in its internal model/link maps, so the
    `DetachableJoint` referencing its link can never actually attach —
    explaining why the pin never held at all (not "held forever," as
    the previous, wrong theory said the first fix would cause; this bug
    is *upstream* of that one, so the anchor never even worked in the
    session where it was first tested). **Fixed** by adding the missing
    `creator.SetParent(anchorModelEntity, this->world.Entity())` call.
    **General lesson, supersedes/extends the one above**: any time
    `SdfEntityCreator::CreateEntities()` is called on a bare
    `sdf::Model*`/`sdf::Light*` (i.e. not via the `sdf::World*`
    overload), the caller MUST follow it with an explicit
    `creator.SetParent(newEntity, parentEntity)` call, or the new entity
    is invisible to every other system (physics included) despite
    existing in the ECM.
  - **Third correction**: the `SetParent()` fix above did make the
    `Physics.cc` "not found in model/link map" errors disappear (`gz-sim`
    log confirmed clean of them), but the interactive re-check got
    *worse*, not better — Atlas flying/teleporting/spinning with
    nonsensical behavior from the moment it was pinned, not just when
    `pub_atlas_command` ran. Root cause (not yet re-confirmed
    interactively as of this writing, but high-confidence from source
    inspection): `EnsureWorldPinAnchor()`'s anchor model set
    `modelSdf.SetStatic(true)`, which does get faithfully written into the
    ECM as a `components::Static` component — but gz-physics's dartsim
    plugin evidently does not reliably honor `Static` for a model created
    at runtime via `SdfEntityCreator` *after* the simulation has already
    started, the way it does for a model declared in the world SDF from
    load time (Skeleton "is this body fixed to the world" is normally
    baked in at construction, same family of limitation as the original
    dynamically-added-world-joint bug two corrections up). If so, the
    "static" anchor was actually simulated as a free ~1 kg dynamic body,
    rigidly welded to Atlas's ~150 kg pelvis via `DetachableJoint` — i.e.
    not an anchor at all, just extra mass glued on with nothing holding
    *it* down, while `AtlasPlugin`'s stand-prep PID controller kept
    applying torques on the (wrong) assumption that the base really was
    immovable. That combination — an unconstrained system receiving
    torques tuned for a fixed base — is a textbook recipe for exactly
    "flying/teleporting/spinning" instability. **Fixed** by not creating
    any runtime model for the anchor at all:
    `VRCPlugin::EnsureWorldPinAnchor()` now just looks up the `link` link
    of the world file's own `ground_plane` model (`World::ModelByName()` +
    `Model::LinkByName()`) — a model declared in the world SDF and loaded
    through the normal world-loading path, so it is unambiguously,
    correctly static from tick zero, no runtime-creation uncertainty of
    any kind. Requires the world to define a model literally named
    `ground_plane` with a link literally named `link`; true of every
    hand-authored world in this migration (logs a clear error otherwise).
    **General lesson, supersedes the "spawn a static anchor" idea from two
    corrections up**: don't spawn a fresh "static" model at runtime to use
    as a physics anchor in gz-sim/dartsim (Harmonic, gz-sim8) — `Static`
    honoring for runtime-created entities is unproven/unreliable here;
    prefer welding to a link that already exists and was already static
    from world-load time (e.g. `ground_plane`) whenever one is available.
    **This theory was also wrong** — the user rebuilt, retested, and
    reported "nothing changed. same flying, teleporting, falling to the
    ground. all at the same time." The `ground_plane`-vs-runtime-anchor
    swap made zero observable difference, which rules out `Static`-
    honoring as the (sole) explanation, since `ground_plane` is
    unambiguously static and behavior was identical anyway.
  - **Fourth correction — reverted, not re-fixed.** Three attempts to
    replace the direct `sdf::Joint`-to-`"world"` pin mechanism with a
    `DetachableJoint`-based weld (runtime anchor, then `ground_plane`)
    each preserved or worsened the same flying/teleporting/spinning
    instability, and the automated test suite (537/537 passing every
    round) never caught any of it, since it has no interactive/visual
    check. Rather than guess a fourth time, `AddJoint()` was reverted to
    the exact `sdf::Joint`-to-`"world"` mechanism from before any of this
    — the one mechanism ever interactively confirmed to hold Atlas
    correctly (round 3, "Atlas successfully stand"). `EnsureWorldPinAnchor()`
    and the `worldPinAnchorLinkEntity` member are removed entirely; the
    known, real, but far more minor bug this mechanism has (pin can't be
    *fully* unpinned — `RequestRemoveEntity` doesn't detach the underlying
    dartsim constraint, see the class-level design note) is left as an
    open, documented, deferred issue rather than something to chase
    further today. **Awaiting interactive re-confirmation** that this
    reverted code is back to at least the round-3 baseline (stands
    without flying apart) before touching this mechanism again — and any
    future attempt at a real fix should start by getting the GUI open and
    watching frame-by-frame what happens at the instant of pinning, not
    reading logs after the fact or trusting the automated test suite,
    since that suite has now passed clean through four different (three
    broken) versions of this code without ever detecting the difference.
  - **Fifth attempt, this time genuinely different in kind.** Reverting to
    the round-3 `sdf::Joint`-to-`"world"` mechanism did *not* restore the
    round-3 baseline the way byte-identical code should have: this time
    interactive testing showed the pin holding fine while pinned (a little
    vibration, expected/normal), but the exact same old "pelvis frozen
    forever, everything else flailing" symptom the whole rework was
    originally meant to fix — meaning the previously-reported "success" in
    round 3 was likely never fully reliable in the first place, and the
    real, only-ever-partially-fixed bug is the one this whole design-note
    history is about: **gz-physics/dartsim will not cleanly detach a
    dynamically-added joint whose parent is `"world"`, full stop**, no
    matter which of the three joint-creation mechanisms tried. Given that,
    trying a *fourth* joint-based variant made no sense. Instead: **stopped
    using a physics joint for this case entirely.** `AddJoint()`'s
    world-pin branch now returns a bare, component-less placeholder entity
    (nothing else — no `sdf::Joint`, no `DetachableJoint`); the actual pin
    is a per-tick kinematic pose override, added to `VRCPlugin::
    UpdateStates()`, that forcibly re-applies `atlas.pinHoldPose` to
    `atlas.pinLinkEntity` via `Model::SetWorldPoseCmd()` — the exact same
    primitive `Teleport()`/`SetLinkWorldPose()` already use elsewhere in
    this same plugin — for as long as `atlas.pinJointEntity !=
    kNullEntity`. Unpinning is just `RemoveJoint()` clearing that entity,
    which stops the per-tick re-application; there is no physics-engine
    joint to fail to detach, because none is ever created. All four call
    sites that used to call `AddJoint()` for the world-pin case
    (`PinAtlas()`, `Teleport()`, and `DoSetRobotMode()`'s `"harnessed"`
    and `"pid_stand"` branches) now also set `atlas.pinHoldPose` to
    whatever pose should be held, at the same point they used to rely on
    the joint holding it. **General lesson**: when a physics-engine
    limitation (not a bug in this plugin's own code) blocks a joint-based
    approach after multiple genuinely-different joint variants all hit the
    same wall, stop trying joint variants — a kinematic per-tick pose
    override is a legitimate, often more robust substitute for "rigidly
    hold this link in place," and this plugin already had the exact
    primitive needed (`SetLinkWorldPose`) sitting right there, proven, the
    whole time. **Interactively confirmed working**: pin holds correctly
    (stands, small forward/backward PID oscillation, does not fall),
    unpin genuinely releases it -- `pub_atlas_command` (a `3.2 *
    sin(...)`, ~183°, uniform-amplitude command to *every* joint
    including hips/knees/ankles, with no balance controller involved) now
    makes it topple, which is the *correct*, expected physical result of
    that command on a freed biped, not a bug -- the old bug was that nothing
    happened at all because the pelvis stayed permanently welded. Learn
    from the last false "confirmed harmless" mistake in this same section:
    this conclusion is based on the pelvis visibly reacting under load,
    not merely on the absence of log errors.
    One cosmetic-only artifact observed on this same interactive check: a
    brief flash, right around spawn, of what looks like multiple Atlas
    instances in different poses/positions before settling to the single
    correct one. Consistent with a software-rendering (`llvmpipe`, no GPU
    passthrough in this container -- confirmed via the `libEGL warning:
    egl: failed to create dri2 screen` lines already present in every log
    this whole session) GUI-side stale-frame artifact during the
    server-to-GUI scene sync at spawn time, not a physics/duplicate-entity
    bug -- physics itself only ever has one Atlas model, confirmed by it
    correctly and singularly standing afterward. Not investigated further;
    flag here in case it recurs or worsens.
  - **Also cleaned up in this same round** (found via the same interactive
    log, unrelated to the pin bug but real, unfixed leftovers from the
    Tier 3 `atlas_description` pass): `atlas.gazebo`/`atlas_v3`/`atlas_v4`/
    `atlas_v4_no_wry2`/`atlas_v5.gazebo` still had dead Classic ROS-bridge
    sub-plugins (`libgazebo_ros_p3d.so`, `libgazebo_ros_force.so`,
    `libgazebo_ros_camera.so` ×3 per file) with no gz-sim shared library to
    load (`SystemLoader.cc: Could not find shared library` at every
    launch) — removed (the underlying `<sensor>` blocks themselves are
    kept; only the dead ROS-bridge child `<plugin>` tags are gone; no
    ported replacement exists yet, matching the CLI-executable/Tier-3
    drop precedent). Also `multisense_sl{,_v3,_v4,_cpu}.urdf` had the same
    two problems as `AtlasPlugin`/`VRCPlugin` originally did before their
    Tier-2 fix: a dead `libgazebo_ros_gpu_laser.so` sub-plugin (removed)
    and `<plugin filename="libMultiSenseSLPlugin.so" name=
    "multisense_plugin"/>` not matching this port's actual
    `GZ_ADD_PLUGIN_ALIAS(MultiSenseSLPlugin,
    "drcsim_gazebo_ros_plugins::MultiSenseSLPlugin")` name (fixed to
    `filename="MultiSenseSLPlugin"
    name="drcsim_gazebo_ros_plugins::MultiSenseSLPlugin"`).
  - **Regression test added** for the pin/unpin release bug specifically
    (`test_vrc_plugin.cpp`'s new `UnpinActuallyReleasesUtorsoToFallUnder
    Gravity`): shortens `atlas.time_to_unpin` to 1.0s via the
    `DRCSIM_ROS_PARAMS_FILE` mechanism and asserts `utorso` actually falls
    under gravity after auto-unpin, in a world with nothing else to
    support it. The existing test only ever checked that pinning holds —
    never that unpinning releases it, which is exactly the gap that let
    three broken pin-release mechanisms pass automated testing in a row.
  - **`atlas.launch.py` now sets `VRC_CHEATS_ENABLED=1`** (new
    `cheats_enabled` launch arg, default `true`) — it never set this
    before, so `VRCPlugin`'s `atlas/cmd_vel` subscriber (and its other
    cheat-gated extras) were silently unreachable through this launch
    file even though `atlas_v5_gains.yaml`'s gains and `pub_atlas_command`
    worked fine (a separate, non-cheat-gated path via `AtlasPlugin`).
  - **`atlas/cmd_vel` is a kinematic slide/warp, not a walking gait — by
    design, matching the original.** Publishing to it moves Atlas's whole
    body to a new position each tick, standing posture unchanged, no leg
    motion at all. This is not a regression or a bug: **no real dynamic
    walking controller has ever existed in this codebase, including in
    the original Gazebo Classic `drcsim`** — the genuine DRC-era Atlas
    walking behavior ran on Boston Dynamics' proprietary
    `AtlasSimInterface`/BDI software on external hardware, under NDA to
    competing teams, and was never open-sourced or part of this repo.
    `atlas/cmd_vel` was the original's own substitute: a course-navigation
    cheat for quickly repositioning Atlas during VRC scoring tests, not a
    physics-based gait. `atlas.startup_mode=bdi_stand` (separately
    documented above as unreliable) is a *stand-up* choreography, not
    walking, either. A real walking controller would be new functionality
    outside this port's scope, not something to "fix" here.

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

## Tier 4 — `drcsim_tutorials` (✅ scoped subset done)

Five rosbuild (`manifest.xml`, no `package.xml` at all — one step further
back than any tier before this) subpackages. Surveyed via an Explore agent
before touching anything; scope below matches that survey's
recommendation.

**Dropped entirely** (no external references found anywhere in the repo
before removal — confirmed via grep first):
- `atlas_interface` — wraps Boston Dynamics' proprietary, closed-source
  `AtlasRobotInterface` SDK (`$ATLAS_ROBOT_INTERFACE_ROOT`, not present
  anywhere in this workspace) around `AtlasSimInterfaceCommand` — the same
  BDI walk/behavior shim already confirmed inert in Tier 2 (`AtlasPlugin`'s
  `WALK`/`STAND` behaviors are a no-op PID stand-in). Nothing real to port.
- `atlas_interface_bridge` — a bridge for driving the *physical* Atlas
  robot, not simulation; compiles in ~1600 lines of vendor example code
  from that same missing SDK. Its own README admits it was never tested on
  real hardware.
- `control_mode_switch` — one demo script (`demo.py`) whose entire point is
  a scripted BDI walk sequence, which cannot actually walk (same inert
  shim). Its only working part (basic `AtlasCommand` pose sequencing)
  duplicates the already-ported `pub_atlas_command`/`pub_joint_commands`
  CLI tools (Tier 2).
- The stray `drcsim_tutorials/CMakeLists.txt` at this directory's root —
  leftover rosbuild-era top-level workspace glue (`ExternalProject_Add`
  for `atlas_msgs`/`joint_commands_gui`/`atlas_teleop`), meaningless in an
  ament workspace; not a real per-subpackage file.

**Ported: `atlas_teleop`** (ament_cmake, Python scripts installed via
`install(PROGRAMS ...)`, matching this repo's only established Python-heavy
package precedent, `drcsim_gazebo` — no `ament_python` package exists
anywhere else in this repo, so this doesn't introduce a second convention):
- `drc_vehicle_teleop.py` — joystick → `DRCVehicleROSPlugin`/`VRCPlugin`
  cheat topics. **Real bug found and fixed**: the original hardcoded
  `drc_vehicle/...` as the topic prefix, but `DRCVehicleROSPlugin`
  advertises every cmd/state topic under the spawned vehicle model's
  actual gz-sim name (`Model(_entity).Name()`, confirmed by reading
  `DRCVehicleROSPlugin.cpp`), which is `golf_cart` in `atlas.world` — the
  original's hardcoded prefix would never have matched anything in this
  migration. Now a `vehicle_model_name` ROS param, default `golf_cart`.
  `drc_world/robot_enter_car`/`robot_exit_car` (fixed, non-model-scoped
  `VRCPlugin` topics) were already correct as-is.
- `atlas_teleop.py` — joystick-driven whole-body pose blending, publishing
  `AtlasCommand`. **Real bug found and fixed**: the original targeted
  Atlas v3/v4's 28-joint layout (no `wry2` wrist joint) with joint names
  entirely commented out (`AtlasCommand` has no per-joint name field at
  all, unlike `JointCommands` — position/effort/etc. arrays are matched to
  joints purely by index) — mismatched with `atlas_v5`'s real 30-joint
  layout this migration actually targets (confirmed exact order by reading
  `AtlasPlugin.cpp`'s `jointNames.push_back()` calls directly, not
  assumed). Also **simplified a design mistake in the original**: rather
  than hardcoding its own separate `kp`/`kd` gain arrays (the original
  did; this is exactly the class of problem `RosNodeOptionsFromEnv`/the
  Tier 3 gains-file mechanism was built to avoid), this leaves
  `AtlasCommand`'s gain fields empty entirely — `AtlasPlugin::
  SetAtlasCommand` only ever applies a gain field when its array length
  matches its own joint count, so an empty array is silently ignored and
  the real, already-loaded `atlas_v5_gains.yaml` values are left alone.
  The 4 bundled task-preset YAMLs (`drill`/`drive`/`firehose`/`pedal`)
  were mechanically converted from 28- to 30-joint format (a `0.0`
  inserted at both new `wry2` positions; every other value is the
  original's real, hand-tuned data, unchanged) rather than dropped —
  verified correct by diffing the conversion script's output against the
  originals position-by-position before committing.
- `nanokontrol.py` — MIDI-hardware-to-`Joy` driver, ported mechanically
  (rospy→rclpy, Python 2→3 syntax) but **not functionally verified**: it
  needs the `pygame` package and physical Korg nanoKONTROL hardware,
  neither available here. Treat as unverified until someone with the
  actual hardware tries it.
- The pure blending/parsing logic (`blend_command()`, `load_pose_file()`)
  was split out of the ROS callbacks specifically so it's unit-testable
  without rclpy or a live joystick — see `test/test_blend_command.py`.

**Ported, as a real rewrite: `joint_commands_gui`** — a per-joint slider
GUI publishing `JointCommands` to `atlas/joint_commands`. Three real
problems found while reading the original before deciding how to port it:
1. **wxWidgets**, not available in a stock ROS 2 Jazzy desktop-full image.
   Rewritten with Tkinter (stdlib, no new dependency).
2. **Stale joint filtering**: hardcoded a 28-joint, `"atlas::"`-prefixed
   name list (same class of bug as `atlas_teleop.py` above, and the same
   class already fixed for Tier 2's CLI executables) to decide which URDF
   joints get a slider. Updated to the real, unprefixed, 30-joint
   `atlas_v5` list.
3. **Dead code path**: `source_list` subscribed topics as
   `FollowJointTrajectoryActionGoal`, a type the original **never actually
   imported** — would `NameError` if `source_list` were ever set. Dropped
   entirely rather than fixed forward, since nothing in this migration
   ever populates that param anyway.

Two more real bugs surfaced only by actually running it interactively
(not caught by flake8/pep257/lint_cmake or the unit tests, none of which
touch rclpy message construction or Tk's runtime threading model):
4. **`JointCommands` has no `k_effort` field** — confirmed via an actual
   `AttributeError` at runtime. The port had assumed symmetry with
   `AtlasCommand` (which does have one); `osrf_msgs/JointCommands.msg`'s
   real field list has no per-joint PID-vs-effort blend concept at all.
   Fixed by simply not setting it.
5. **`RuntimeError: Calling Tcl from different apartment`** — the first
   version created the `Tk()` root window on the rclpy subscription
   callback thread, then ran its `.mainloop()` on a separate spawned
   thread (mirroring the original wx version's own thread split). Tcl/Tk
   requires the *same* thread that creates a Tk interpreter to also drive
   its event loop; this is a stricter requirement in this environment's
   Tk build than wx apparently enforced. Fixed by removing the extra
   thread entirely: `main()` now blocks on `rclpy.spin_once()` until
   `robot_description` arrives, then builds the Tk window and pumps
   `rclpy.spin_once(timeout_sec=0)` from inside Tk's own event loop via
   `window.after()` — GUI and ROS callbacks all run on one thread.

Also **redesigned how it gets `robot_description`**: the original read it
via `rospy.get_param('robot_description')`, relying on ROS 1's global flat
parameter server (any node can read any other node's params) — no ROS 2
equivalent exists (every node's parameters are private to it). Rather than
invent a new param-passing mechanism, this subscribes to the
`robot_description` topic `robot_state_publisher` already publishes by
default with transient-local QoS in ROS 2 — the standard, idiomatic way
other tools (e.g. RViz) already get it, and needs zero coordination with
`atlas.launch.py`. Same gain-field-left-empty simplification as
`atlas_teleop.py` above, for the same reason (`atlas_controller/gains/...`
was read from the same now-nonexistent global param server in the
original).

**General lesson from this whole tier**: reading the *original* source
carefully before porting caught four real, independent bugs (two stale
28-vs-30-joint layouts, one hardcoded-vs-actual topic prefix, one dead
import) that a purely mechanical rospy→rclpy translation would have
carried forward silently. Cross-checking every topic/param name against
the actual C++ plugin source (`grep`ing `AtlasPlugin.cpp`/
`DRCVehicleROSPlugin.cpp` directly, not trusting the original tutorial's
own comments) is what caught all four. **But two more bugs (the missing
`k_effort` field, the Tk threading crash) got through static review, unit
tests, and every linter clean, and only showed up on the first actual
interactive run** — a reminder that for anything touching real message
types or a GUI event loop, "all lint/tests pass" is necessary but not
sufficient; an actual `ros2 run` is the only thing that catches these.

**Not ported / deferred**: nothing further remains in this tier's original
five subpackages — three were dropped, two fully ported.

## Beyond the migration: `atlas_walking_demo` (new original content, not a port)

First of an open-ended series of *new* tutorials the user is building on
top of the finished migration — deliberately kept out of `drcsim_tutorials`
(which is specifically the ported original content) as its own package.
Statically-stable, keyboard-controlled stepping (`w`=walk, `space`/`s`=stop,
no turning yet) — explicitly not real dynamic/balance-controlled walking,
since no CoM/ZMP controller exists anywhere in this codebase (confirmed via
a repo-wide grep before designing this). `gait_controller.py`'s neutral
standing pose and feedforward efforts are transcribed directly from
`VRCPlugin::AtlasCommandController::SetPIDStand()`'s real, already-tuned
v5 values (`VRCPlugin.cpp` ~line 1634) rather than reinvented; every gait
phase is a target pose composed from that baseline (stance lean + swing-leg
lift/flex deltas) and linearly interpolated to over time, since
`AtlasPlugin` has no velocity/rate limiter of its own — snapping between
poses would yank joints as hard as their effort limit allows. See the
package's own `README.md` for the run command and a milestone-by-milestone
tuning guide (using `AtlasPlugin`'s already-live, cheats-gated
`atlas/debug/{l,r}_foot_contact` topics as a numeric verification signal,
not just visual judgment) — full design rationale lives in the plan this
was built from. All tuning constants are starting estimates from Atlas
v5's real leg geometry (also pulled from the URDF directly, not guessed),
expected to need empirical retuning; being plain Python, that only needs
`ros2 run` again, not a rebuild.

**Real bug #1, first interactive run, caught immediately**: Atlas fell
over *before any key was even pressed*. `walk_keyboard.py`'s publish
timer starts the instant the node comes up, and `GaitController`
originally assumed Atlas was already standing in a hardcoded
`NEUTRAL_STAND` pose (`SetPIDStand()`'s deep crouch) — but
`atlas.launch.py`'s default startup leaves it standing closer to upright
(near-zero joints), not crouched. So the very first published command was
itself an instant, unguarded jump from Atlas's real pose to a deep crouch
— exactly the "no rate limiter, don't snap" trap the module docstring
already warned about, just not actually guarded against in code.
First attempted fix: subscribe to `atlas/joint_states` (`AtlasPlugin`'s
real current positions, same 30-joint order confirmed via `jointStates.
name = this->jointNames` in `AtlasPlugin.cpp`) and smoothly interpolate
*from* that real pose *into* `NEUTRAL_STAND` over a few seconds before
allowing any walk request.

**Real bug #2, second interactive run**: that fix's own settle —
slow, smooth, no instant jump — still made Atlas fall, straight backward,
partway through. The real problem wasn't the transition speed, it was the
*destination*: `NEUTRAL_STAND` (`SetPIDStand()`'s pose) had been assumed
to be a generally safe, "already-tuned" standing pose worth reusing, but
every actual call site of `SetPIDStand()` in `VRCPlugin.cpp` either
re-pins the robot immediately (`"pid_stand"` mode) or runs during vehicle
entry/exit with the base already rigidly held — it was **never once
proven stable for a fully free-standing robot**, because it was never
used that way in the original code either. Commanding a deep crouch with
nothing holding the pelvis in place let the center of mass walk out from
under it. **Fixed** by dropping the idea of a separately-designed
"neutral" pose entirely: `GaitController` now takes Atlas's real starting
pose (from `atlas/joint_states`) as `neutral_pose` directly and never
asks it to move away from that pose before a walk is requested — it's the
only pose actually proven stable free-standing this whole session, so
every gait phase's lean/lift/swing deltas are computed relative to *it*,
not a hardcoded alternative. The mismatched `SetPIDStand()` feedforward
effort terms (tuned for the crouch, not this pose) were dropped for the
same reason — plain `k_effort=255` PID with zero effort feedforward,
matching what's already proven to hold Atlas up on its own.

**Real bug #3, third interactive run**: with `neutral_pose` now sourced
from the real `atlas/joint_states` topic (bug #2's fix), Atlas *still*
fell straight backward — this time with no gait phase ever requested at
all (no `w` pressed), just from the node existing and republishing
whatever it read. Root cause: the fix trusted the very *first*
`atlas/joint_states` message unconditionally as `neutral_pose`, with no
check that it was actually a steady-state reading rather than a snapshot
taken while Atlas was still mid-transient right after unpinning (still
actively moving under `AtlasPlugin`'s PID as it settles) — freezing a
still-moving snapshot as a fixed PID target is not the same thing as
freezing a genuinely at-rest one. **Fixed** by waiting for real stability
before locking in `neutral_pose` at all: `walk_keyboard.py` now buffers
recent `atlas/joint_states` readings and only accepts one once the
max per-joint change across a full second of history drops below 0.01
rad, logging the captured leg-joint values alongside the "ready" message
specifically so the *next* round (if there is one) has something to
inspect immediately rather than needing another back-and-forth just to
get a data point.

**Real bug #4, fourth interactive run**: bug #3's stability-threshold fix
never fired at all — no "ready" log ever appeared, `walk_keyboard.py`
never published anything (so no fall either, trivially), and `w` did
nothing (`self.gait` stayed `None` forever). Root cause: Atlas's standing
pose has a small persistent oscillation by design — already observed and
described as normal/expected multiple times earlier this same session
("oscillating forward and backward a little... not fallen") — so a
strict "wait until per-joint change drops below 0.01 rad" condition could
be waiting for a stillness this system may never actually produce.
**Fixed** by replacing the stability-threshold wait with a fixed delay
instead (`SETTLE_DELAY_SEC = 5.0`, measured from the first
`atlas/joint_states` message): simpler, and structurally can't loop
forever the way a never-satisfied threshold can. Whatever small
oscillation remains 5 seconds after the first reading is the same kind
already long confirmed harmless, not the one-time, larger post-unpin
settling transient bugs #2/#3 were actually guarding against.

**Real bug #5, fifth interactive run**: bug #4's fixed-delay fix produced
a fully sane, symmetric captured pose (logged in full this time: every
joint near-zero, legs and arms mirrored) — and Atlas *still* fell
immediately the instant the "ready" message printed, before any key was
pressed. With the pose data itself finally cleared of suspicion, the
remaining candidate was the *act* of publishing itself. Checked
`AtlasPlugin.cpp` directly rather than guessing again: `ZeroAtlasCommand()`
(called once at `Configure()`, before any `AtlasCommand` message is ever
received) sets `k_effort=0` for every joint, and `UpdatePIDControl()`'s
integral term (`errorTerms[i].kIqI`) accumulates *every tick regardless
of `k_effort`*, even while contributing nothing to the output — with
`atlasCommand.position` stuck at its `0.0` default the entire time while
Atlas's real position drifted slightly, that integral term looked likely
to have been quietly winding up (clamped, but substantial) all session.
**Tried**: route the first command through the `atlas/reset_controls`
*service* (`reset_pid_controller=true`, already ported in Tier 2, unused
by anything until now) instead of the plain topic — zeroes `errorTerms`
and atomically applies the new `AtlasCommand` in one call.

**This theory was also wrong** — the service call reported success
(logged explicitly: "the PID handoff succeeded"), and Atlas fell anyway,
ruling out stale integral state as the (sole) cause.

**Real bug #6, sixth interactive run — the actual root cause**: with
state-based explanations exhausted, checked what `k_effort=0`'s *own*
output path actually computes, rather than assuming "some other
reasonable control keeps it up." `UpdateAtlasSimInterface()`'s
`asiState.f_out[i]` is computed entirely from `asiCommand`'s gains
(`kp_position`/`ki_position`/`kp_velocity`/`effort`) — and those fields
are *only* ever populated by an incoming `AtlasSimInterfaceCommand`
message (`SetASICommand`), which nothing in `atlas.launch.py`'s default
flow ever sends. They sit at their post-`resize()` default of `0.0`
forever. So `f_out[i]` has been computing to exactly zero this entire
session. With `k_effort=0`, the blended output is `1.0 * f_out[i] = 0`.
**Atlas has been standing this whole session under zero active control
torque** — held up by passive joint dynamics near its resting pose, not
by the real, loaded `atlas_v5_gains.yaml` PID gains at all, despite
multiple earlier entries in this very doc (before this was discovered)
describing it as standing "under `AtlasPlugin`'s real gains." Switching
`k_effort` to `255` for the first time — even with the position matched
exactly and the integral freshly zeroed — is the first moment *any*
nonzero gain has ever actually been applied to a joint: an inherent shock
to a system that had been running torque-free the whole time, not a
side-effect of stale state. **Fixed** by ramping `k_effort` from `0` to
`255` over `KEFFORT_RAMP_SEC` (3s) instead of switching it instantly —
the `reset_controls` call from bug #5 is kept (still needed, sets
`k_effort=0` explicitly and the matching position target atomically,
genuinely bumpless at that instant) but now only as the ramp's starting
point, not the whole fix.

**Real bug #7, seventh interactive run**: with bug #6's ramp in place and
diagnostic logging added (measured-vs-target for the leg pitch joints,
~3Hz, throughout the ramp), Atlas fell again — "on its knees, then on its
back." The log data itself is what finally pointed away from control
theory entirely: at `t=0.1s` into the ramp (`k_effort=9`, ~3.5% strength),
`l_leg_kny` was *already* measured at `2.167` rad (out of a `2.356` rad
max — essentially fully collapsed). A gain that weak cannot move a knee
joint over 2 radians in 0.1 simulated second; the earlier torque-spike
and insufficient-gain theories both require far more applied effort than
was available yet. Root cause, on reflection: `walk_keyboard.py`'s main
loop called `rclpy.spin_once(node, timeout_sec=0)` exactly once per
iteration, interleaved with a **blocking** `select()`-based keyboard read
of up to ~50ms — meaning ROS callback processing (the 30Hz publish timer,
the `joint_states` subscription, the `reset_controls` response) could be
starved or batched behind that blocking read, corrupting the very timing
(the settle-delay check, the `k_effort` ramp) every fix since bug #4
depended on: the "0.1s" label was `self.get_clock().now()` at *processing*
time, not necessarily when the underlying event actually happened, so a
backlog of queued messages could make a much longer real delay look like
"0.1s" once finally drained. **Fixed** by moving ROS spinning onto its
own dedicated background thread (a `SingleThreadedExecutor` on a
`threading.Thread`), fully decoupled from the keyboard-reading loop, so
timer/subscription/service callbacks are serviced promptly regardless of
keyboard activity.

**Real bug #8 — the actual root cause of bugs #2–#7 (2026-09-28, found
with direct Docker access, measured rather than inferred)**: bug #6's
premise — "Atlas stands under zero active torque" — was wrong. Measured
at rest via `atlas/atlas_state` / `atlas/atlas_sim_interface_state`:
`k_effort=255` on every joint (`LoadPIDGainsFromParameter()` sets it right
after `ZeroAtlasCommand()` zeroes it), `f_out=0` (irrelevant at
`k_effort=255`), PID setpoint all zeros, and real holding torques (knee
≈ −59 N·m, hip pitch ≈ −81 N·m) generated by gravity sagging each joint
~0.03 rad below that setpoint — pure P+D gains, no integral term, so the
sag *is* the error that produces the holding torque. Every version since
bug #2 commanded the *measured* (sagged) pose as the new setpoint, which
zeroes that error and therefore the holding torque: Atlas collapses. Bug
#5/#6's `reset_controls(k_effort=0)` made it worse by switching the
already-working controller off outright. Timeline confirmed with a
monitor running alongside the node: pitch went −3° → −34° → −81° (on its
back) within 2 s of the handoff, at only ~11% ramp gain. Also: the
"converged" ramp diagnostics from bug #7's run were Atlas lying on the
ground with its joints at target — misread at the time. **Fixed** by
rewriting `walk_keyboard.py` to take over the controller's *setpoint*,
reconstructed from `atlas/atlas_state` as `position + effort / kp`
(exact at rest; came out 0.000 as predicted), keeping `k_effort=255`
throughout — no reset, no ramp, no settle delay. Verified: the node runs
25 s with Atlas standing exactly as in the no-node baseline (itself
verified stable for 90 s). Bug #7's executor-thread change is kept (good
practice regardless), but it was not the cause of the falls. **Remaining,
genuinely new work**: pressing `w` makes Atlas fall ~2 s into the first
stride — the gait keyframes themselves (lean direction/magnitude, the
baseline's existing ~−4° backward lean) now need real tuning.

**Lesson from bug #8 specifically**: five rounds of fixes were built on a
conclusion inferred from reading source (`k_effort=0` from
`ZeroAtlasCommand()`) without measuring the live value, which was one
`ros2 topic echo` away. Measure the running system before theorizing
about it.

**General lesson (all seven bugs, same root shape, getting deeper each
time)**: any new controller publishing a fixed "assumed" pose, reusing an
existing "known-good" pose, trusting a single live reading as if it
represented steady state, waiting on a condition that may never actually
be satisfied, resetting a control path's internal state without checking
whether *that path was ever actually active in the first place*,
switching a system straight to full-strength active control without
verifying it was under any active control at all before, or trusting its
*own* timing/message-processing loop without checking whether that loop
can actually keep up, risks exactly this — check *where the robot really
is* (bug #1), *under what conditions a reused pose was actually
validated* (bug #2), *whether a live reading is actually settled, not
mid-transient* (bug #3), *whether "settled" is even a condition this
system will ever satisfy* (bug #4), *what invisible state a
newly-activated control path may already be carrying* (bug #5), *whether
that control path — or any active control at all — was ever actually the
thing holding the system up in the first place* (bug #6), and *whether
the diagnosing node's own execution model can actually deliver the timing
it's being trusted for* (bug #7) before trusting any of them. Same
root-cause shape as the `atlas.world` pin-link mismatch from Tier 3, just
further up the stack each time — and bugs #5–#7 specifically are the same
*class* of lesson as the VRCPlugin world-pin saga earlier in this
session: a system's real behavior only becomes visible the moment
something finally exercises the path that reveals it, and "confirmed
stable" observations made before that moment don't actually validate what
they were assumed to.

### Harness walking (✅ verified 2026-09-29, in the `drcsim_jazzy` container)

Free-standing stepping still falls ~2 s after `w` (open-loop, sway ±3.5°
at ~2 s period; P-only ankle feedback `aky += 0.5*pitch` halves sway, any D
term or Kp ≥ 1 falls). Per the user's choice, walking now runs **in a
harness first**; balance control (CoM/ZMP) is the next project.

- `walk_keyboard.py` (default `harness:=true`) publishes
  `pinned_with_gravity` on `atlas/mode` at takeover.
  - It runs `gait_controller.HARNESS_CYCLE`: LIFT/SWING/PLANT per leg.
  - Leg poses are (hpy, kny, aky) from `HARNESS_LEG_POSES`, with the sole
    flat when grounded.
  - It publishes `atlas/cmd_vel` (`forward_speed`, 0.35 m/s) only while
    not IDLE.
  - `harness:=false` keeps the old free-standing `WALK_CYCLE`.
- **VRCPlugin warp drift fix**: the cmd_vel warp integrated from the
  *measured* pin-link pose.
  - Physics nudges the held pelvis within a tick, and that nudge was
    adopted into `pinHoldPose` every tick.
  - Before the fix: 3 s of pure `linear.x=0.3` went −0.36 m x, +0.31 m y,
    yaw +0.39 rad.
  - The warp now integrates from `pinHoldPose` while pinned.
  - After the fix: 25 s walk → x +9.08 m, y −0.02 m, yaw −0.002 rad.
- "Drunk" gait root cause, measured with `docker_ws/walk_stats.py`:
  - The pelvis was held throughout (roll/pitch/yaw ±1°, z ±1 mm).
  - The legs were not: hip yaw hit its ±45° limits and hip roll swung
    ±24°, both commanded 0.
  - `atlas_v5_gains.yaml` has `hpz` p=5, `hpx` p=900, `akx` p=300, so
    foot friction twists the legs freely.
  - Fix: `walk_keyboard.py` sends full `kp_position`/`kd_position` arrays
    in AtlasCommand; AtlasPlugin adopts full-length gain arrays. In harness
    mode it overrides hpz (1000, 10), hpx (2500, 10), akx (1000, 3).
  - Result: hpz/hpx within ±6°.
- Arms: at joint zero Atlas is in a T-pose. Harness mode lowers the arms
  (`HARNESS_ARM_REST`) over `IDLE_DURATION` and swings `shz` against the
  opposite leg's hpy.
  - Signs were measured with TF pelvis→hand: negative `l_arm_shz` and
    positive `r_arm_shz` move the hand forward.
- Foot contact: with the pelvis held at standing height, fore/aft leg
  poses lift the foot.
  - Heel-strike FRONT and toe-down BACK/PUSH_OFF raised per-foot loaded
    time from ~37% to ~46%.
- `GaitController` phase transitions now start from the last *sampled*
  pose, not the previous target, so pressing `w` mid-interpolation (e.g.
  while the arms are lowering) never snaps.
- RViz was blank because `docker_ws/start_gui.sh` ran `rviz2` without
  `-d /root/atlas.rviz` and without `use_sim_time`.
- `walk_keyboard.py` quit abort ("terminate called without an active
  exception") came from destroying the node under a still-spinning
  executor thread. Fixed by joining the spin thread first.
- Tests: atlas_walking_demo 22/22, drcsim_gazebo_ros_plugins 160 (0 fail,
  33 skipped).

### Harness walking v2: IK foot trajectories + spring harness (2026-09-29)

The keyframe harness gait walked, but looked bad, so it was replaced.
Everything below was measured in the container with
`docker_ws/{foot_track,walk_stats,track_err,glide_test}.py`. Foot world
position comes from `/world/default/dynamic_pose/info`: model pose plus
foot pose relative to the model.

**New gait: `atlas_walking_demo/scripts/harness_gait.py`** (pure,
unit-tested). It plans ankle positions in the hip-pitch frame and solves
2-link sagittal IK. Details:
- **Leg geometry** from `atlas_v5_raw.urdf`: thigh (-0.05, -0.374), shin
  (0, -0.422). At zero angles the ankle sits under the pelvis origin.
- **IK branch gotcha:** the 5 cm thigh offset means `kny=0` and
  `kny≈0.27` give the same leg length. Joint zero is on the *other* branch
  from a forward-bent knee, so the takeover crouch blends in joint space.
- **Crouch depth is very sensitive:** a 5 cm drop needs 49° of knee bend;
  2.5 cm gives about 30° mid-stance (used).
- **Swing:** Hermite curve with endpoint slopes equal to ground speed, so
  the foot leaves and lands at zero *world* velocity. This alone did not
  fix slip; the harness did.
- **Timing:** stance moves the ankle back at the pelvis speed. The node
  runs on sim time (`/clock` is bridged), so its velocity integration
  matches VRCPlugin's (measured 0.409 against 0.417 m/s commanded).

**VRCPlugin changes:**
- The cmd_vel warp honours `linear.z` while pinned (lowers the harness).
  `SetRobotCmdVel` used to treat a z-only command as a stop.
- While pinned, the warp only advances `pinHoldPose`; it no longer
  teleports.
- `ApplyHarness()` is a spring-damper wrench on the pin link:
  - linear 20000 N/m and 4000 N·s/m;
  - rotational 10000 N·m/rad and 300 N·m·s/rad;
  - teleport only if the error exceeds 5 cm or 0.25 rad.

Why the harness changed, in order of discovery (each version measured):
1. **Pose teleport every tick:** teleporting a model moves *all* its
   links. The loaded stance foot was dragged with the pelvis, then yanked
   back by the hip: stick-slip, 6–12 cm per stance.
2. **Adding a pin-link velocity hold:** `Link::SetAngularVelocity`
   (link-frame) splayed hip roll ±15–30°, cause unknown. Linear velocity
   alone did not stop slip.
3. **Velocity servo with no teleport:** still 6–10 cm slip, and foot
   loads of about 1000 N each (2000 N total, more than body weight).
   Diagnosis: a rigid pelvis, stiff joint servos and a rigid floor
   over-constrain the robot. Millimetre mismatches become hundreds of
   newtons, and the soles tip ±10°.
4. **Spring-damper (current):** slip about 2.5 cm per stance, a steady
   0.50 m stride, pelvis and hip yaw/roll within 1.5°, about 880 N per
   foot. Plugin tests 160/0 fail; the normal startup pin/unpin/stand is
   unchanged.

Also: the gz GUI `/gui/screenshot` service returns true but saves nothing.
`ImageGrab` of `:1` captures the user's whole desktop, so don't use it.

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
  interface. The only real consumer, `DRCVehicleROSPlugin` (Tier 2, ✅
  ported — see its entry above), **subclasses** `DRCVehiclePlugin`
  (`class DRCVehicleROSPlugin: public DRCVehiclePlugin`) and calls these
  methods directly via normal C++ inheritance — confirmed via repo-wide
  grep before deciding this (an earlier assumption that a *separate*
  plugin held a raw pointer to this one was wrong; don't reintroduce a
  transport-based design for this). That subclassing is cross-package
  (`DRCVehicleROSPlugin` lives in `drcsim_gazebo_ros_plugins`), which
  required adding a real CMake target export to this package — see
  `DRCVehicleROSPlugin`'s entry for details — and a small
  `IsValidConfig()` accessor.
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
