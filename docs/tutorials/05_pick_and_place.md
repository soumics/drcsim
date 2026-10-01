# 5. Pick-and-place

Atlas picks up a **10 kg box** from a table with both hands, carries it 25 cm to the side
and sets it down, under whole-body torque control.

```bash
docker/sim.sh sync_max_per_window:=5.0 sync_max_per_step:=0.05    # SVH hands (default)
docker/shell.sh
ros2 run atlas_walking_demo pick_place.py                       # -p box_mass:=10.0
```

The node does everything itself:

1. It spawns a table and the box in front of Atlas (`ros_gz_sim create`).
2. Atlas crouches, switches to torque control and holds still for 2 s.
3. It runs through the waypoints below. Each prints `stage: …`.
4. It ends with `Pick-and-place finished.`

| Stage | Palms |
|---|---|
| raise | up and out, above the table top (the hands start low at the sides) |
| reach, pre-grasp | above, then beside the box, 15 cm out |
| approach | 2 cm from the sides |
| squeeze | 2 cm *inside* the sides: the grip is a two-handed squeeze |
| lift | 15 cm up |
| carry | 25 cm to the right |
| lower, release, retreat | set down, open, back up |
| clear, home | back to where the hands started |

## How it works

- **Hand tasks.** `wbc.py` adds a 6D task per SVH palm frame. Targets are interpolated
  smoothly (position linear, rotation along the geodesic). The position error is
  saturated at 8 cm, so a hand that falls behind can't demand a huge acceleration.
- **Squeeze force.** Pushing a palm target into the box makes the hand press on it,
  roughly kp × depth × the arm's effective mass. Friction (μ = 1) holds the box.
- **Payload.** From lift-off to release, `wbc.set_payload(mass, reach)` puts half the
  box's mass in front of each palm in the Pinocchio model. The CoM, gravity and dynamics
  then include it, so the balance task leans Atlas back to carry it.
- **Joint limits.** The QP keeps every joint within its limits, as acceleration bounds.
  Without them it reached for the box by hyperextending the elbows, which can't straighten
  past 0, instead of leaning the back. The arms then hit their stops and dragged Atlas
  over.
- **Frames.** Waypoints are relative to the point on the ground between the soles. The
  QP's world frame starts at the pelvis and Gazebo's at the spawn point, so neither can be
  used directly.

## Things to try

- `-p box_mass:=15.0`: heavier boxes. The torque limits of the arms are the limit.
- Change `CARRY_Y`, `BOX_CENTER` or the waypoints in `pick_place.py`. The unit tests in
  `test/test_pick_place.py` check that the geometry stays consistent: the box sits on the
  table, the table holds both spots, and the palms face each other.
- Use `debug:=true` to log tilt, CoM, foot loads, palm errors and joints at their limits
  every 0.5 s.
