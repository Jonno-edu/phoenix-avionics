# Phoenix Avionics (OBC Firmware)

Avionics software for the Phoenix mission's On-Board Computer (OBC), based on the Raspberry Pi Pico 2 (RP2350). The system runs on FreeRTOS and acts as the central command and control unit.

## Applications

The workspace contains two distinct applications located in `apps/`:

### 1. Main OBC (`apps/main-obc`)
The primary flight software.
- **Role**: RS485 Bus Master, System Controller.
- **OS**: FreeRTOS Kernel (SMP enabled).
- **Features**:
  - **TCTLM Dispatcher**: Handle Telecommands (Reset, Log Level, Sim Mode) and Telemetry.
  - **Twin-Pipe Communication**: Dedicated channels for command/response (Queue A) and high-speed telemetry streaming (Queue B).
  - **Subsystem Management**: Polls EPS, Radio, and HIL node via RS485.

### 2. HIL Node (`apps/hil-node`)
A Hardware-in-the-Loop simulation bridge.
- **Role**: Responds to OBC queries by returning simulated sensor data.
- **Features**:
  - **I2C Slave**: Emulates sensor hardware (IMU, GPS, Barometer).
  - **Simulation Bridge**: Injects synthetic data from a host PC into the I2C bus.

## Project Structure

```text
 apps/
   ├── main-obc/       # Flight Software
   └── hil-node/       # HIL Simulation Bridge
 lib/
   ├── esl-comms/      # Shared Protocol Library (RS485/TCTLM)
   └── FreeRTOS-Kernel/# RTOS Source
 common/             # Shared definitions (sensor configs)
 tools/              # Python scripts for deployment & testing
 rockchip/           # Deployment scripts for the remote lab host
```

## Development Workflow

This project uses **CMake Variants** and **VS Code Tasks** for a streamlined workflow.

### 1. Select Variant
Use the CMake Variant selector in the status bar (or `CMake: Select Variant`) to choose:
- **Main OBC**: Builds the flight software.
- **HIL Node**: Builds the simulation node.

### 2. Build & Deploy
We use a remote development setup (Rockchip Host). The following VS Code tasks are available:

- **Flash OBC**: Builds `main-obc` and flashes it to the OBC Pico.
- **Flash HIL Node**: Builds `hil-node` and flashes it to the HIL Pico.
- **Monitor OBC**: Opens a remote `minicom` session for the OBC UART.
- **Flash All**: Flashes both devices in sequence.

### 3. Telemetry & Streaming
- **Command Mode**: Standard command/response via RS485.
- **Stream Mode**: High-speed binary dump of sensor data.
- **Console**: Debug logs available via USB CDC (when enabled) or UART.

## Communication Architecture

### Protocol
Uses the **ESL-Comms** library:
- **RS485**: Custom HDLC-like framing with COBS escaping.
- **TCTLM**: Generic dispatcher for Event, Telecommand, and Telemetry messages.

### Generic Component IDs
The TCTLM layer uses generic `CommsInterfaceId_t` types, allowing the same library to be used by both the OBC (RS485 Master) and HIL Node (RS485 Slave).

## Software-in-the-Loop (SIL)

A generic `mac_host` (or Linux) variant is available to run logic on a PC.
- **IO**: Redirects RS485 traffic to a Virtual Serial Port (PTY).
- **Usage**: Select `Mac Host` variant -> Build -> Run.

