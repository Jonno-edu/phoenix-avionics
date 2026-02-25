# Phoenix Avionics — OBC Firmware

Flight software for the Phoenix mission's On-Board Computer (OBC), running on a
Raspberry Pi Pico 2 (RP2350) with FreeRTOS. The OBC is the RS485 bus master and
central controller for all subsystems on the rocket.

## Hardware

| Component | Detail |
|---|---|
| MCU | RP2350 (Raspberry Pi Pico 2) |
| RTOS | FreeRTOS SMP (dual-core) |
| Bus | RS485 @ 115200 baud |
| Debug | USB CDC (stdio) |

## Repository Structure

```text
phoenix-avionics/
│
├── common/protocol/          # Shared wire contract (pending move to lib/phoenix-icd)
│   ├── telemetry_defs.h      # ICD: all node addresses, message IDs, common structs
│   ├── eps_payloads.h        # EPS-specific wire structs (power status, measurements)
│   ├── radio_payloads.h      # Tracking radio-specific wire structs
│   └── phoenix_protocol_version.h
│
├── hal/                      # Hardware Abstraction Layer
│   ├── rs485_hal.c/h         # UART1 + DE/RE GPIO, raw debug flag
│   └── platform_hal.c/h
│
├── bsp/                      # Board Support Package
│   └── bsp_init.c/h
│
├── src/
│   ├── main.c                # Startup, task creation
│   ├── FreeRTOS-openocd.c    # Debug symbol preservation
│   │
│   ├── common/               # OBC-internal utilities (→ will become src/norb/)
│   │   ├── pubsub.h/c        # NORB message bus (→ norb.h/c)
│   │   ├── topics.h          # Topic enum (TOPIC_SENSOR_IMU, TOPIC_EPS_*, ...)
│   │   ├── logging.h         # ESP_LOGI/W/D macros over printf
│   │   └── topic_defs/
│   │       └── sensor_imu.h  # IMU topic payload struct
│   │
│   └── modules/
│       ├── nodes/            # RS485 node drivers (pure function libs, no tasks)
│       │   ├── eps/
│       │   │   ├── eps.h     # eps_poll(), eps_request_*(), eps_set_line()
│       │   │   └── eps.c
│       │   └── tracking_radio/
│       │       ├── tracking_radio.h  # tracking_radio_poll(), request_ident()
│       │       └── tracking_radio.c
│       │
│       ├── datalink/         # RS485 request/response abstraction over HAL
│       │   ├── datalink.h
│       │   ├── datalink.c
│       │   ├── receiver.c     # Dedicated RX tasks (RS485 + USB)
│       │   ├── tctlm.c        # Telecommand/Telemetry dispatcher
│       │   └── streams/       # Periodic telemetry streams
│       │       └── stream.h
│       │
│       ├── housekeeping/     # Health monitor — polls nodes, publishes topics
│       │   ├── housekeeping.h
│       │   └── housekeeping.c
│       │
│       ├── sensors/          # OBC onboard sensors (IMU)
│       │   ├── sensors.h/c
│       │   └── imu.h/c
│       │
│       ├── estimator/        # Extended Kalman Filter
│       │   └── ekf.h/c
│       │
│       └── commander/        # Flight state machine (placeholder)
│
├── lib/
│   ├── esl-comms/            # RS485 framing engine (git submodule)
│   ├── esl-math/             # Math utilities — DCM, quaternion (git submodule)
│   ├── esl-sensors/          # Sensor drivers (git submodule)
│   └── FreeRTOS-Kernel/      # FreeRTOS source (fetched by CMake if absent)
│
├── docs/
├── FreeRTOSConfig.h          # RTOS configuration (stack sizes, tick rate, SMP)
├── CMakeLists.txt
└── cmake-variants.yaml       # Pico / Host build variants
```

## RS485 Bus

The OBC is bus master. All communication is request/response — the OBC
initiates every transaction and nodes reply.

### Node Addresses

| Address | Node |
|---|---|
| `0x01` | OBC |
| `0x02` | EPS |
| `0x03` | Tracking Radio |
| `0xF0` | GSE (ground support) |

### Message Descriptor Byte

Every packet contains a `MSG_DESC` byte that encodes both the message type
and ID:

```
MSG_DESC = (msg_id & 0x1F) | ((msg_type & 0x07) << 5)
           └── bits [4:0]: ID    └── bits [7:5]: Type
```

Examples:

| Request | MSG_DESC | Hex |
|---|---|---|
| Identify any node | TLM_REQ + TLM_COMMON_IDENT (0x00) | `0x80` |
| EPS power status | TLM_REQ + TLM_EPS_POWER (0x01) | `0x81` |
| EPS measurements | TLM_REQ + TLM_EPS_MEASURE (0x02) | `0x82` |
| EPS power command | TC + TC_EPS_POWER (0x01) | `0x41` |

All IDs and addresses are defined in `common/protocol/telemetry_defs.h` —
the single source of truth shared with EPS and tracking radio firmware.

## Software Architecture

### Layer Model

```
┌─────────────────────────────────────────────┐
│  Tasks: housekeeping, sensors, estimator,   │  ← FreeRTOS tasks
│          tctlm, commander                   │
├─────────────────────────────────────────────┤
│  NORB (pubsub)  —  inter-module message bus │  ← src/common/pubsub (→ src/norb)
├──────────────────────┬──────────────────────┤
│  Node drivers        │  Sensor drivers      │  ← src/modules/nodes/
│  eps, tracking_radio │  imu, baro, gps      │     src/modules/sensors/
├──────────────────────┴──────────────────────┤
│  Datalink  —  request/response over RS485   │  ← src/modules/datalink/
├─────────────────────────────────────────────┤
│  HAL  —  UART, GPIO, direction control      │  ← hal/rs485_hal
└─────────────────────────────────────────────┘
```

### nORB — nano Object Request Broker

Modules communicate via topics, never by calling each other directly.
Inspired by PX4's uORB.

```c
// Publisher (housekeeping after polling EPS):
norb_publish(TOPIC_EPS_MEASUREMENTS, &meas);

// Non-blocking subscriber (check latest value, don't stall):
EpsMeasurements_t meas;
if (norb_subscribe_poll(TOPIC_EPS_MEASUREMENTS, &meas)) { ... }

// Blocking subscriber (sleep until data arrives):
sensor_imu_t imu;
if (norb_subscribe(TOPIC_SENSOR_IMU, &imu, 10)) { ... }
```

Each topic holds exactly one value (depth-1 queue). Publishing always
overwrites. Subscribing never consumes — multiple tasks can read the same topic.

### Node Drivers

Node modules are pure function libraries with no tasks of their own.
Housekeeping calls `eps_poll()` and `tracking_radio_poll()` on a 1-second
schedule; each poll function handles the full request/response conversation
and publishes results to NORB.

```c
// housekeeping_task — the full poll loop:
while (1) {
    eps_poll(NODE_TIMEOUT_MS);
    tracking_radio_poll(NODE_TIMEOUT_MS);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
}
```

## Building

### Prerequisites

- [Raspberry Pi Pico SDK 2.2.0](https://github.com/raspberrypi/pico-sdk)
- CMake >= 3.13
- GCC ARM toolchain 14.2
- VS Code + Raspberry Pi Pico extension (optional)

### Pico Build

```bash
mkdir build && cd build
cmake .. -DPICO_BOARD=pico2
make -j$(nproc)
```

### Host Build (Mac/Linux — runs FreeRTOS on POSIX)

```bash
mkdir build-host && cd build-host
cmake .. -DPICO_PLATFORM=host
make -j$(nproc)
./phoenix_avionics_monorepo
```

### Flash

```bash
make flash   # requires picotool; put Pico in BOOTSEL mode first
```

Or drag-and-drop the `.uf2` from `build/` onto the Pico mass-storage device.

## Debug Output

Connect via USB CDC (115200 baud). Raw RS485 byte logging is disabled by
default and can be enabled at runtime:

```c
rs485_hal_set_raw_debug(true);   // enable [RS485 RX RAW] lines
rs485_hal_set_raw_debug(false);  // disable (buffer still drained silently)
```

## Planned: `lib/phoenix-icd`

`common/protocol/telemetry_defs.h` is the Interface Control Document (ICD)
shared between all Phoenix node firmware repos. It will be extracted into a
standalone `lib/phoenix-icd` git submodule so that OBC, EPS, tracking radio,
and GSE decoder all pull from the same versioned source. A mismatch between
`PHOENIX_ICD_VERSION` values will be detectable at compile time.

## License

MIT — see [LICENSE](LICENSE).
