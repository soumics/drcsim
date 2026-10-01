# 5. Pick-and-place: carry a box to another table

Atlas picks a **10 kg box** off table A with both hands, steps back, turns about 80° and
walks to table B, sets the box down and lowers its arms. All of it runs under whole-body
torque control, and Atlas balances on its own the whole time (no harness).

```bash
docker/sim.sh sync_max_per_window:=5.0 sync_max_per_step:=0.05    # SVH hands (default)
docker/shell.sh
ros2 run atlas_manipulation pick_place.py              # -p box_mass:=10.0 -p debug:=true
```

The node does everything itself:

1. It spawns table A with the box in front of Atlas, and table B where the walking route
   will end (`ros_gz_sim create`).
2. Atlas crouches in position mode, then switches to torque control.
3. It runs the stages below, printing `stage: …` for each.
4. It ends with `Pick-and-place finished.`

It takes about 2 minutes of simulation time.

| Stage | What happens |
|---|---|
| raise | the hands come up and out, above the table top (they start low at the sides) |
| reach, pre-grasp, approach | palms above, beside, then 2 cm from the box's sides, hands open |
| grasp | the palms close in and squeeze (200 N each), hands flat on the box |
| lift, bring in | 15 cm up, then in close to the chest |
| walk to table B | 4 steps back, a right turn (24 short steps), 8 steps forward |
| present, reach out, lower | out over table B, then down onto it |
| release, retreat, clear, home | open, hands up and away, arms back to where they started |

## How it works

| Piece | Where |
|---|---|
| Palm tasks: 6D, smoothly interpolated, error saturated at 8 cm | `wbc.py` (`hand_targets`) |
| Squeeze: the box pushes back on each palm, as a known force in the QP's dynamics | `wbc.py` (`squeeze`) |
| Payload: half the box's mass in front of each palm in the model | `wbc.set_payload` |
| Joint limits as acceleration bounds | `wbc.py` |
| Walking under torque control: the ZMP planner's footsteps and CoM path, tracked by the QP | `qp_walk.py`, `TorqueBalance.start_walk` |
| A scripted route, and where it will end (for placing table B) | `qp_walk.Route`, `qp_walk.predict` |
| Task sequence, body frames, tables, hands | `atlas_manipulation/scripts/pick_place.py` |

**Frames.** Task targets are written in a *body frame*: on the ground between the soles,
x forward. While walking, that frame rides on the measured pelvis, so the box stays still
relative to the chest, the way a person carries it.

## What it took (each step measured in the simulator)

| Problem | Fix |
|---|---|
| The box and tables spawned inside Atlas | `ros_gz_sim create` ignores the SDF `<pose>`; pass `-x -y -z -Y` |
| The QP reached by bending the elbows past straight; the arms hit their stops and pulled Atlas over | joint limits in the QP |
| The hands ran into the side of the table | a `raise` stage: up and over first |
| Atlas fell on "finished" | `home` stage: the arms go back down before the hand task is dropped |
| The CoM sat 8 cm off its target, standing with the box | the balance task outweighs the hand task (`w_com` 10000) |
| The walk's first step ran off backward | the plan starts at rest at the measured CoM |
| The box slid out of the hands | an explicit squeeze; flat hands (curled fingers took the load on weak joints); rubber-like hand friction (μ 2) |
| A squeeze added after the solve tilted Atlas 20° forward | the squeeze is modelled in the QP's dynamics instead |
| Turning in place toppled Atlas | the box was brushing table A: step back 4 steps first; slower steps with the box; small turns per step |
| Atlas's toes hit table B on the last step | tables are a top on four legs, not solid blocks |
| Sudden "falls" after 2 minutes of nothing | a ProxQP solve ran for about 115 s on a hard problem: iterations are capped, and the last good torques are reused for that tick |

Result: 3 of 3 runs succeeded. The box ends on table B at the same spot to within 2 mm.

## Things to try

- `-p box_mass:=15.0`: a heavier box. Raise `squeeze` to match (it must exceed
  m·g/2/μ per palm).
- Change `ROUTE` in `pick_place.py`. Table B moves to wherever `qp_walk.predict` says
  the route ends; the unit tests check that the box would land on it.
- `debug:=true` logs tilt, foot loads, palm error, squeeze, QP failures and CoM tracking
  error twice a second.
- Walk under torque control without a box:
  `ros2 run atlas_walking_demo torque_stand.py --ros-args -p sync_period_ms:=2 -p walk_steps:=8 -p walk_vx:=0.1`.
