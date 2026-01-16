# Software-in-the-Loop (SIL) Usage Guide

This guide explains how to build, run, and test the Phoenix Avionics firmware in SIL mode on macOS.

## Overview

SIL (Software-in-the-Loop) mode allows you to run the complete firmware stack on your Mac, simulating the RP2350 hardware. This is useful for:

- Testing RS485 protocol implementation
- Developing and debugging without physical hardware
- Running automated tests with Python scripts
- Validating command/telemetry interfaces

## Architecture

### How SIL Works

**Hardware (RP2350)**:
```
RS485 Data → UART IRQ → ISR → Circular Buffer → RS485 Task
```

**SIL (Mac)**:
```
Python Script → PTY (Virtual Serial) → High-Priority Task → Circular Buffer → RS485 Task
```

### Key Components

1. **Virtual Serial Port (PTY)**: The firmware creates a pseudo-terminal pair that acts like a real serial port
2. **Polling Task (`vSerialRxTask`)**: High-priority FreeRTOS task that polls the PTY every 5ms, simulating UART interrupts
3. **Circular Buffer**: Same buffer used on hardware, filled by the polling task instead of an ISR
4. **RS485 Protocol**: Runs identically to hardware - no changes needed

### Differences from Hardware

| Aspect | Hardware | SIL |
|--------|----------|-----|
| Serial Trigger | UART interrupt (μs latency) | Task polling (5ms latency) |
| Context | ISR context | FreeRTOS task context |
| Timing | Real-time | Best-effort |
| Cores | Dual-core SMP | Single-core |

**Note**: 5ms polling is sufficient for RS485 at 115200 baud (~87μs per byte). The 256-byte circular buffer can handle burst reception.

## Building and Running

### 1. Select Mac Host Variant

In VS Code:
1. Open Command Palette (`Cmd+Shift+P`)
2. Type "CMake: Select Variant"
3. Choose `mac_host`

### 2. Build the Firmware

**Option A: VS Code Task**
- Press `Cmd+Shift+P` → "Tasks: Run Task" → "Build Active Variant"

**Option B: Command Line**
```bash
cd /Users/jonno/GitProjects/Phoenix/phoenix-avionics
cmake -B build -G Ninja -DPICO_BOARD=host
cmake --build build
```

### 3. Run the Firmware

```bash
./build/phoenix-avionics
```

You should see output like:
```
========================================
SIL Virtual Serial Port Created
========================================
Port: /dev/ttys002

Connect your Python scripts to this port.
Example: python3 test.py --port /dev/ttys002
Or use: screen /dev/ttys002 115200
========================================

[SIL] Serial RX task started - polling virtual serial port
```

**Important**: Note the virtual port path (e.g., `/dev/ttys002`) - this changes each time you run the firmware.

## Testing with Python

### Using the Provided Test Script

In a **separate terminal** (keep the firmware running):

```bash
# Install pyserial if needed
pip3 install pyserial

# Run the test script (use the port from firmware output)
python3 tools/test_sil_rs485.py --port /dev/ttys002
```

### Expected Output

```
==================================================
Phoenix Avionics - SIL RS485 Test
==================================================
Port: /dev/ttys002
Baud: 115200
✓ Connected to virtual serial port

→ Requesting system identification...
  TX: 7e 01 00 28 00 29 7f
  RX: 7e 00 01 38 08 01 01 00 01 00 00 00 0a 3a 7f
  ✓ Received telemetry response

  System Information:
    Node Type: 1
    Interface Version: 1
    Firmware: v0.1
    Runtime: 0.010 seconds

→ Sending RESET command...
  TX: 7e 01 00 00 00 01 7f
  RX: 7e 00 01 10 00 11 7f
  ✓ Received ACK

→ Requesting system identification...
  TX: 7e 01 00 28 00 29 7f
  RX: 7e 00 01 38 08 01 01 00 01 00 01 00 14 3f 7f
  ✓ Received telemetry response

  System Information:
    Node Type: 1
    Interface Version: 1
    Firmware: v0.1
    Runtime: 1.020 seconds

==================================================
Test completed!
==================================================
```

### Manual Testing with Screen

You can also connect manually to send raw bytes:

```bash
screen /dev/ttys002 115200
```

Type hex values (with proper framing) to send RS485 packets.

## Integrating with Your Python Scripts

Your existing RS485 Python tools should work with minimal changes:

```python
import serial

# Instead of:
# ser = serial.Serial('/dev/ttyUSB0', 115200)  # Hardware

# Use:
ser = serial.Serial('/dev/ttys002', 115200)  # SIL (adjust path)

# Everything else stays the same!
packet = build_rs485_packet(...)
ser.write(packet)
response = ser.read(100)
```

## Troubleshooting

### "Failed to open PTY: ...".

PTY creation failed. This is rare on macOS but could indicate:
- System limit on pseudo-terminals reached
- Permission issue

**Fix**: Restart your terminal or check system limits.

### "No such file or directory" when connecting

The virtual port path has changed or the firmware isn't running.

**Fix**: 
1. Make sure `./build/phoenix-avionics` is running
2. Copy the exact port path from its output
3. Use that path in your Python script

### Python script hangs or times out

The polling task might not be running or there's a communication issue.

**Fix**:
1. Check firmware output for `[SIL] Serial RX task started`
2. Try sending data with `screen` first to verify basic connectivity
3. Increase timeout in Python: `serial.Serial(..., timeout=5)`

### Firmware crashes on Mac

FreeRTOS POSIX port issue or stack overflow.

**Fix**:
1. Check for stack overflow messages
2. Increase task stack sizes in `main.c`
3. Verify FreeRTOS configuration in `FreeRTOSConfig.h`

## Limitations

1. **Timing**: 5ms polling means ~5ms worst-case latency vs μs on hardware
2. **Single Core**: Mac build runs on one core (no SMP)
3. **No Real Sensors**: I2C devices won't work (need mock implementations)
4. **No GPIO**: Hardware pins can't be toggled (need simulation layer)

For high-fidelity timing tests, use real hardware. SIL is for functional testing.

## Next Steps

1. **Add More Tests**: Create Python scripts for all your telecommands
2. **Automated CI/CD**: Run SIL tests in GitHub Actions
3. **Mock Sensors**: Implement simulated sensor data for SIL
4. **Logging**: Add flight data recording in SIL mode

## Technical Details

### PTY (Pseudo-Terminal) Implementation

The firmware uses POSIX pseudo-terminals:

```c
// Create master/slave PTY pair
pty_master_fd = posix_openpt(O_RDWR | O_NOCTTY);
grantpt(pty_master_fd);    // Grant permissions
unlockpt(pty_master_fd);   // Unlock for access
ptsname_r(...);            // Get slave device path
```

The firmware uses the **master** side (reads/writes).
Your Python script uses the **slave** side (appears as `/dev/ttysXXX`).

### Task Priorities

```c
vSerialRxTask:  tskIDLE_PRIORITY + 3  // Highest - simulates interrupt
vRS485Task:     tskIDLE_PRIORITY + 2  // Lower - gets preempted
```

This ensures `vSerialRxTask` preempts `vRS485Task` when data arrives, mimicking hardware interrupt behavior.

### Circular Buffer Thread Safety

- **Hardware**: ISR writes, task reads → needs critical sections
- **SIL**: High-priority task writes, lower-priority task reads → same protection needed

Both use `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` which:
- Hardware: Disables interrupts
- SIL: Uses pthread mutex

Code is identical, FreeRTOS handles the difference.

## References

- [FreeRTOS POSIX Port](https://www.freertos.org/FreeRTOS-simulator-for-Linux.html)
- [POSIX Pseudo-Terminals](https://man7.org/linux/man-pages/man7/pty.7.html)
- Project README: `../README.md`
