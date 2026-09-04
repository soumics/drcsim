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
     but a **new, expensive lesson** did: see "The `ament_uncrustify`
     `Component<T>()` saga" below before writing any test that calls
     `EntityComponentManager::Component<T>()`.

**Remaining Tier 2 plugins** (each its own `port/<name>` branch): SandiaHandPlugin,
IRobotHandPlugin (needs the `handle_msgs` camelCase→snake_case field fix noted
in Tier 0), RobotiqHandPlugin, MultiSenseSLPlugin, VRCPlugin, VRCScoringPlugin,
AtlasPlugin/V3/V4/V5 (biggest/riskiest — ties into the AtlasSimInterface shim
libs from `drcsim_model_resources`), DRCVehicleROSPlugin (**subclasses**
`DRCVehiclePlugin` — see below), ContactModelPlugin ✅, then the 8 CLI
executables + `actionlib_server` + `gz_model_teleport` + `test_ros_plugin`.

### The `ament_uncrustify` `Component<T>()` saga — read before touching this again

`EntityComponentManager::Component<T>()` (and no other templated call found so
far) triggers a genuine, unexplained `ament_uncrustify` bug in
`test_contact_model_plugin.cc`: uncrustify misparses `Component<T>(...)` as a
comparison expression and wants spaces around `<`/`>`, which `cpplint` then
rejects — a real disagreement between the two tools, not a mistake in the code.
**Nine restructuring attempts, all failed, each isolating one variable:**
return vs. assign-then-use, file-scope free function vs. lambda vs. an
out-of-line `ClassName::Method()` definition (matching
`ContactModelPlugin::PostUpdate()`'s own passing shape exactly), a `for` loop
copied statement-for-statement from the passing plugin code, a long vs. short
type-alias name, one-line vs. two-line call — **every single one still
failed**, including a plain `std::vector<gz::sim::Entity>` **parameter
declaration** with no relation to `Component<T>()` at all. Meanwhile the
identical `_ecm.Component<gz::sim::components::ContactSensorData>(...)` call
passes with zero divergence inside `ContactModelPlugin.cpp` itself. **The
actual cause was never found.**
**Resolution: stop fighting it.** Don't call `EntityComponentManager::
Component<T>()` in a test file at all if a non-template alternative proves the
same thing. `EntityHasComponentType(entity, ComponentT::typeId)` takes no
template argument and was enough to prove `ContactModelPlugin` correctly
identified and tagged the configured collision — the test now uses that
instead of also reading back the resulting contact list's content. If a future
test genuinely needs typed component *data* (not just presence), expect this
same wall and don't burn more than one or two attempts on it — go straight to
a non-template route or accept the weaker assertion.

## `drcsim_gazebo_plugins` — design decisions and lessons (done, keep as reference)

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
  flag any downstream C++ consumers that used the old field names (e.g.
  `IRobotHandPlugin.cpp` in `drcsim_gazebo_ros_plugins`, Tier 2, still uses old
  `handle_msgs` camelCase field names — needs updating when that package's ported).
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
