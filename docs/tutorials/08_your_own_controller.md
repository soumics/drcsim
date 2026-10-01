# 8. Writing your own controller

Every controller in this repo talks to `AtlasPlugin` through two topics:

| Topic | Type | Direction |
|---|---|---|
| `atlas/atlas_state` | `atlas_msgs/AtlasState` | joint position/velocity/effort, IMU orientation and rates, foot and wrist force/torque, every physics step (1 kHz) |
| `atlas/atlas_command` | `atlas_msgs/AtlasCommand` | per joint: `position`, `velocity`, `effort`, `kp_position`, `ki_position`, `kd_position`, `k_effort` |

The arrays are matched to joints **by index**, in `AtlasPlugin`'s order. The list is
`atlas_walking_demo/scripts/harness_gait.py`'s `ATLAS_JOINT_NAMES` (30 joints: back, neck,
legs, arms).

## The control law

```
force = clamp( k_effort/255 · (kp·(q_d − q) + ki·∫ + kd·d(q_d − q)/dt + effort), ±effort limit )
```

- `k_effort` (0–255) blends between this command and the (inert) BDI interface. Use 255.
- **Gains:** a gain array is applied only when its length is 30. Send empty arrays to keep
  the gains loaded from `atlas_v5_gains.yaml`.
- **kd** acts on the derivative of the *error*, so a jumping target gives a torque spike.

## Rule 1: take over from the setpoint, not the measured pose

When Atlas is standing, the default controller's setpoint is all zeros. Gravity sags each
joint a little below it, and that error is what produces the holding torque. If you
command the measured pose, the error goes to zero and Atlas collapses. Reconstruct the
setpoint instead:

```python
setpoint = [p + e / kp for p, e, kp in zip(state.position, state.effort, gains_kp)]
```

`balance_controller.FreeWalkController` does this, then blends to its own stance over a
few seconds. The simplest robust start is to reuse it:

```python
import balance_controller as bc   # from atlas_walking_demo's lib dir

ctrl = bc.FreeWalkController(first_state)
out = ctrl.update(state, t)       # at 200 Hz of sim time
cmd.position, cmd.effort = out.position, out.effort
cmd.kp_position, cmd.kd_position = out.kp, out.kd
if ctrl.status == 'ready':
    ...                           # crouched and balanced: switch to your controller
```

## Rule 2: use simulation time

Atlas with all its sensors runs below real time. Create nodes with
`Parameter('use_sim_time', value=True)` and time everything by `get_clock().now()`.

## Torque mode

For torque control, send:
- `kp_position = 0`;
- `kd_position = 1` (a little damping);
- a **constant** `position`;
- `effort = τ`.

See `torque_stand.py` and [tutorial 3](03_torque_control.md).

## Lockstep

Set `desired_controller_period_ms` (e.g. 2) and copy each state's `header.stamp` into the
command you send for it. `AtlasPlugin` then waits for your command before stepping
physics, within the `sync_max_per_step` / `sync_max_per_window` budgets set at launch.

## Other useful interfaces

- `atlas/mode` (`std_msgs/String`):
  - `pinned`, `pinned_with_gravity`: the harness;
  - `nominal`: free;
  - `recover`: harness reset.
- `atlas/cmd_vel`: moves the harness (pelvis) while pinned.
- `svh_hands/<left|right>/command`: SVH finger joints.
- `/world/default/wrench/persistent`: push links (`drcsim_push` wraps it).

## A minimal skeleton

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from atlas_msgs.msg import AtlasCommand, AtlasState


class MyController(Node):
    def __init__(self):
        super().__init__('my_controller',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(AtlasCommand, 'atlas/atlas_command', 10)
        self.create_subscription(AtlasState, 'atlas/atlas_state', self.on_state, 1)

    def on_state(self, s):
        cmd = AtlasCommand()
        cmd.header.stamp = s.header.stamp
        cmd.k_effort = [255] * 30
        cmd.position = list(s.position)   # replace with your targets (see Rule 1!)
        self.pub.publish(cmd)


rclpy.init()
rclpy.spin(MyController())
```
