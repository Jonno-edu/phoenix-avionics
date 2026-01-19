# Phoenix Avionics (OBC Firmware)

Avionics software for the Phoenix mission's On-Board Computer (OBC), based on the Raspberry Pi Pico 2 (RP2350). The system runs on FreeRTOS and acts as the central command and control unit, communicating with subsystems via RS485.

## System Capabilities

- **Role**: RS485 Bus Master (Address `0x01`).
- **OS**: FreeRTOS Kernel (SMP enabled).
- **Telemetry**: USB Console (CDC) for debugging + RS485 for subsystem comms.

### Architecture
The firmware is structured into modular FreeRTOS tasks:
- **`rs485_task`**: Manages the RS485 bus protocol, handles packet reception/transmission, and routes commands.
- **`eps_polling_task`**: Periodically (1Hz) polls the Electrical Power System (EPS) for status and health.
- **`heartbeat_task`**: Provides visual system health indication via LED.

### Communication Protocol
Uses a custom **HDLC-inspired RS485 protocol** provided by `lib/esl-comms`:
- **Physical**: UART 115200 8N1.
- **Framing**: Start/End markers with byte stuffing (COBS-like escaping).
- **Integrity**: 16-bit CRC (CCITT).
- **Mode**: Half-Duplex (Master/Slave architecture).
- **Updates**: Recently optimized for block-based DMA-friendly transmission to improve bus efficiency.

## Hardware Configuration (RP2350)

| Interface     | Pin   | Function          |
| ------------- | ----- | ----------------- |
| **RS485 TX**  | GP4   | UART1 TX          |
| **RS485 RX**  | GP5   | UART1 RX          |
| **RS485 DE/RE**| GP3   | Direction Control |
| **Console**   | USB   | CDC Serial        |

## Software-in-the-Loop (SIL)

The project includes a functional SIL target for macOS/Linux. This allows running the full flight software on your host machine without hardware.

- **Variant**: Select `mac_host` (or linux) in CMake.
- **IO**: Stubs the hardware UART and redirects RS485 traffic to a **Virtual Serial Port (PTY)**.
- **Testing**: Python tools in `tools/` can connect to this virtual port to simulate the EPS or other subsystems.

## Getting Started

1. **Clone**:
   ```bash
   git clone --recurse-submodules https://github.com/Jonno-edu/phoenix-avionics.git
   ```

2. **Build (Hardware)**:
   - Select `Debug-rp2350` variant.
   - Run task: **"Flash and Monitor via SSH Rockchip"** (if using remote lab) or regular Build.

3. **Build (Simulation)**:
   - Select `Debug-mac` variant.
   - Build and Run.
   - The terminal will display: `SIL Virtual Serial Port Created: /dev/pts/X`.
   - Run `python tools/test_ping.py --port /dev/pts/X` to talk to the simulated OBC.

## Project Structure

```
├── lib/
│   ├── esl-comms/       # Protocol definitions
│   └── FreeRTOS-Kernel/ # RTOS Source
├── src/
│   ├── apps/           # High-level logic (OBC state machine)
│   ├── tasks/          # FreeRTOS tasks
│   ├── hal/            # Hardware Abstraction (RP2350 + SIL)
│   └── core/           # System logging types
└── tools/              # Python test scripts for SIL/HIL
```
