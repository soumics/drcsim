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
8. 🟡 `drcsim_gazebo_plugins` — **in progress, see "Current state" below.** Full
   architectural rewrite (Gazebo Classic `ModelPlugin` → gz-sim `System`
   `ISystemConfigure`/`ISystemPreUpdate`), not a mechanical port. Builds; tests added
   but not yet passing (debugging in progress — see below).
9. ⬜ `drcsim_model_resources` — **not started.** Worlds/models/SDF version upgrade,
   plus proprietary AtlasSimInterface binaries (v1.1.1/2.10.2/3.0.2) — will very likely
   not link against Jazzy's toolchain; need a decision on rebuild vs. drop vs. stub
   before this package can be finished.

**Tier 1+ (not started):** `atlas_msgs`, `atlas_description`, then Tier 2
`drcsim_gazebo_ros_plugins` (huge — VRCPlugin, AtlasPlugin family tied to the
proprietary AtlasSimInterface binaries, SandiaHandPlugin/IRobotHandPlugin/
RobotiqHandPlugin/MultiSenseSLPlugin, DRCVehicleROSPlugin which **subclasses**
DRCVehiclePlugin — see below), then Tier 3 `drcsim_gazebo` (launch/config/tests,
150 old rostest files). `drcsim_tutorials/*` (rosbuild, proprietary SDK) — scope
undecided, leaning toward dropping.

## `drcsim_gazebo_plugins` — current state and open issue

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

**Open issue right now (mid-fix when this file was written):** ran
`ament_uncrustify --reformat` to fix a large style-diff (brace placement,
`Type & name` spacing) — it introduced a **syntax error**
(`return {0.0, 0.0}` missing trailing `;` around `DRCVehiclePlugin.cpp:695`,
inside `JointLimits()`) and **reintroduced blank lines after `public:`/`private:`
labels** in both headers (which `cpplint` explicitly rejects) — uncrustify's and
cpplint's default configs aren't fully mutually consistent for this repo's
inline-label style. Next steps: read the current (reformatted) file contents fresh
from disk, fix the missing semicolon, strip the reintroduced blank lines after
visibility labels (without undoing uncrustify's other formatting), rebuild, retest.
The two gtests (`test_drc_building_plugin`, `test_drc_vehicle_plugin`) had a real
segfault (now fixed, see `FirstOrZero` above) — need to confirm they pass after
these remaining fixes land.

## Established per-package conversion pattern (message packages)

- `package.xml`: format 2 → 3, correct element order matters for `xmllint`:
  `name, version, description, maintainer, license, url, author, depends...`
  (license/url *before* author — got this wrong once, xmllint caught it).
- `catkin` → `ament_cmake`; `add_message_files`/`generate_messages` →
  `rosidl_generate_interfaces`; `message_generation`/`message_runtime` →
  `rosidl_default_generators`/`rosidl_default_runtime`.
- ROS 1's bare `Header header` → must be fully qualified `std_msgs/Header header`
  (ROS 2 dropped the shorthand) — add `std_msgs` as an explicit dependency.
- ROS 2 message/field names must be lower snake_case (no camelCase) — rename and
  flag any downstream C++ consumers that used the old field names (e.g.
  `IRobotHandPlugin.cpp` in `drcsim_gazebo_ros_plugins`, Tier 2, still uses old
  `handle_msgs` camelCase field names — needs updating when that package's ported).
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
