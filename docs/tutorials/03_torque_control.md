# 3. Whole-body torque control

`torque_stand.py` balances Atlas with **joint torques** computed by a whole-body QP every
`AtlasState` (1 kHz of sim time), instead of joint PD targets.

```bash
docker/sim.sh sync_max_per_window:=5.0 sync_max_per_step:=0.05
docker/shell.sh
ros2 run atlas_walking_demo torque_stand.py --ros-args -p sync_period_ms:=2
# another shell: a precise push on the torso, 525 N for 0.2 s of sim time (105 N s)
drcsim_push 0 525 0.2
```

It crouches in position mode first (about 5 s), then logs `switched to torque control`.

## The pieces

| File | Role |
|---|---|
| `wbc.py` | `WholeBodyController`: Pinocchio model from `/robot_description` (free flyer, fingers and lidar locked, 179.7 kg), the QP, ProxQP |
| `torque_balance.py` | `TorqueBalance`: state estimate, contacts, DCM balance law, (experimental) stepping, hand targets |
| `torque_stand.py` | the ROS node: takeover, crouch, torque mode, lockstep |

## The QP

Decision variables: joint accelerations (36 including the floating base) plus a 6D wrench
per foot.

- **Equalities:**
  - floating-base dynamics;
  - for each foot on the ground, no slip;
  - for a lifted foot, zero wrench.
- **Inequalities:**
  - friction and centre-of-pressure cones per sole;
  - torque limits;
  - joint position limits, as acceleration bounds that let each joint stop within
    0.15 s.
- **Tasks**, as weighted least squares:
  - CoM acceleration;
  - pelvis orientation;
  - joint posture;
  - a lifted foot back down level;
  - 6D palm tasks.

The torques come from the joint rows of the dynamics: `τ = M q̈ + h − Jᵀ f`.

## Walking

`TorqueBalance.start_walk()` hands the CoM to a walking plan (`qp_walk.QpWalk`, built on
the position-mode walker's ZMP preview planner):

- **CoM:** tracked with DCM feedback around the planned ZMP,
  `CMP = p_ref + (1 + k)(ξ − ξ_ref)`.
- **Swing foot:** leaves the contact set and follows its planned sole path and heading
  (a 6D task).
- **Landing:** the foot rejoins the contacts when its load cell sees it, or 60 ms after
  the planned touchdown.

```bash
ros2 run atlas_walking_demo torque_stand.py --ros-args -p sync_period_ms:=2 -p walk_steps:=8 -p walk_vx:=0.1
```

`qp_walk.Route` scripts a walk (segments of velocity and step count), and
`qp_walk.predict()` says where it will end. Both are used by pick-and-place
([tutorial 5](05_pick_and_place.md)).

The QP solver is capped at 200 iterations. Once, one solve on a hard problem ran for
about 2 minutes while the simulation went on without commands. When a solve doesn't
converge, the last good torques are reused for that tick.

## Balance law

Divergent component of motion (capture point) ξ = c + ċ/ω:

```
CMP  = ξ + k (ξ − ξ_target)
c̈    = ω² (c − CMP)
```

Foot contact comes from the foot load cells, with 60/20 N hysteresis. A lifted foot leaves
the contact set, so the QP can't lean on it.

## Torque mode in AtlasPlugin

```
force = k_effort · (kp·(q_d − q) + kd·d(q_d − q)/dt + effort)
```

Torque mode sends `kp_position = 0`, `kd_position = 1` (a little damping), a **constant**
position target and `effort = τ`. Keep the target constant: the plugin's kd acts on the
derivative of the error, so a moving target produces torque spikes.

## Lockstep

A Python controller can fall behind the simulation under CPU load. With lockstep:

1. The node sets `desired_controller_period_ms` and stamps each command with its state's
   stamp.
2. `AtlasPlugin` holds each physics step until that command arrives, within a real-time
   budget:
   - `sync_max_per_step`: seconds per step;
   - `sync_max_per_window`: seconds per window.

Start the sim with `sync_max_per_window:=5.0` for a generous budget. Runs then become
repeatable.

## Results (precise 0.2 s pushes on the torso)

| Push | torque control | position control |
|---|---|---|
| sideways 106 N·s | ok | ok |
| forward 75 N·s | ok | ok |
| forward 90 N·s | fell | fell |
| backward 75 N·s | ok | fell |

Capture-point stepping (`stepping`, on) catches roughly a third of 90 N·s forward/backward
pushes. The learned policy ([tutorial 4](04_learned_push_recovery.md)) does better.
