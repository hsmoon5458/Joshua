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

Each layer talks to the one below through an interface, never a concrete type:
a motor driver holds a `BoardChannel`, not a `Serial`, so a motor, a controller
board, and a transport can be chosen independently in config rather than in
code.

The actuator path is there: every `MotorType` `ActionFactory` supports resolves
`board_name` → `BoardFactory` → `OpenChannel` → driver, over the shared lookup
in `board/factory/board_resolver.h`.

Perception resolves the same way, and a sensor reaches hardware through one of
two configured paths:

- **Board leg.** The device multiplexes several channels over one link (a
  Feetech servo bus, an MCU, an EtherCAT slave), so it needs channel
  addressing and bus arbitration: it *is* a board. The sensor names one and
  opens a channel on it, so a sensor and an actuator on one bus share a board
  instance and its bus mutex.
- **Device leg.** The device multiplexes nothing (a camera, a scanning lidar).
  There is no channel to address and no bus to share, so it owns its own
  handle — a `robot::comm::ByteStream` for anything that consumes ordered bytes,
  which keeps the comm axis a config choice.

Sensors are named for what they measure, independently of how they are wired.

Hardware-facing runtime code in this directory is C++.

## Responsibilities

- `action/` — motor drivers (`motors/drivers/`), the actuator interfaces, and
  `factory/`, which resolves a config actuator to a driver.
- `board/` — `interfaces/` (`BoardChannel`, `BoardInterface`), and `factory/`
  with its per-board instance cache, the shared channel resolver, and the
  motor/drive and sensor/signal compatibility tables,
  `proto/`, and `mock/`. `mock/` is C++ test infrastructure: it lets the
  factory and board tests exercise real drivers with no hardware attached.
- `comm/` — transport capability interfaces, concrete communication
  mechanisms, and their factories. See [comm/README.md](comm/README.md).
- `perception/` — sensor drivers behind `PerceptionInterface`: `position/`
  for board-attached position sensors, `camera/` and `lidar/` for device
  sensors, and `factory/`, which resolves the configured acquisition path.

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
