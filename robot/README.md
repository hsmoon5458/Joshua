# Robot

Hardware-facing code: everything between a `.pbtxt` config and a physical
motor, sensor, or bus. ROS 2 nodes live in [ros2/](../ros2/README.md) and
consume these interfaces; nothing here depends on ROS 2 message types.

> This is the code that moves physical motors and opens real buses. Before
> running anything from here, see the hardware-safety section of
> [AGENTS.md](../AGENTS.md).

## Layers

```
action/       what moves      motor semantics — degrees, limits, calibration
board/        what runs the   control-loop owner: bring-up, codec, channel mux
              loop
comm/         how bytes move  serial, EtherCAT — transport only
perception/   what senses     cameras, encoders, lidar
```

`board/` is **mid-migration — read this before touching the actuator path.**
Each layer talks to the one below through an interface, never a concrete type:
a motor driver holds a `BoardChannel`, not a `Serial`, so a motor, a controller
board, and a transport can be chosen independently in config rather than in
code.

The actuator path is there: every `MotorType` `ActionFactory` supports resolves
`board_name` → `BoardFactory` → `OpenChannel` → driver. Perception has not
migrated — it is still driver-direct (Phase 6). Check
[docs/BOARD_LAYER_RFC.md](../docs/BOARD_LAYER_RFC.md) §10 for which phase has
landed before assuming either way.

**There is no Python in this directory, and none should be added.** The Python
robot layer (factories, interfaces, mock drivers) was deleted in RFC §10
Phase 9, and the Pybricks bench driver moved to
[tools/pybricks/](../tools/README.md) as off-runtime-path tooling.
Hardware-facing ROS 2 nodes are C++ only, and `node_generator` no longer
selects between backends.

## Responsibilities

- `action/` — motor drivers (`motors/drivers/`), the actuator interfaces, and
  `factory/`, which resolves a config actuator to a driver.
- `board/` — `interfaces/` (`BoardChannel`, `BoardInterface`), `factory/` with
  its per-board instance cache and motor/channel compatibility validation,
  `proto/`, and `mock/`. `mock/` is C++ test infrastructure: it lets the
  factory and board tests exercise real drivers with no hardware attached.
- `comm/` — `serial/` and `ethercat/` transports plus `factory/`. Transports
  move bytes and know nothing about motors.
- `perception/` — camera, encoder, and lidar drivers behind
  `perception/interfaces/`.

## Non-Goals

- ROS 2 node lifecycle, topics, or message types — see [ros2/](../ros2/README.md).
- Robot-specific values. Joint names, limits, ports, and calibration come from
  the protobuf config ([config/](../config/README.md)), never from constants
  in a driver.
- Firmware. Board firmware and flashing live in
  [firmware/](../firmware/README.md); runtime code never flashes a board.

## Before you change this

Adding a motor type, board, or transport should mean **one new file in one
layer**, not a new enum value threaded through several. Communication mechanism
and capability boundaries are described in [comm/README.md](comm/README.md).
EtherCAT specifics are in [comm/ethercat/README.md](comm/ethercat/README.md).
