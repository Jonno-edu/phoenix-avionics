# Phoenix Avionics

Avionics software for RP2350 based flight controller, featuring FreeRTOS and multi-platform support (Pico 2 & macOS host simulation). This project demonstrates how to maintain a single codebase with dual-target support using CMake variants and VS Code integration.

## Features

- **Multi-platform builds**: Run the same FreeRTOS code on your Mac (simulation) and Pico (hardware).
- **CMake Variant Switching**: Seamlessly switch between `pico` and `mac_host` targets with full IntelliSense support.
- **FreeRTOS Support**: Pre-configured FreeRTOS Kernel (via submodule) working on both platforms.
- **Modular Architecture**:
  - **Command Handler**: Processes incoming commands.
  - **System Data**: Manages shared system state.
  - **Serial**: Handles communication interfaces.
- **VS Code Integration**: Custom tasks for building, flashing, and monitoring.
- **Modern Tooling**: Uses Ninja, picotool, and CMake.

## Prerequisites

1. **VS Code** with the **CMake Tools** extension
2. **CMake** (3.13+) and **Ninja** build tools
3. **GCC ARM toolchain** or system GCC (for mac_host builds)
4. **picotool** (for flashing)

**Note**: No need to install Pico SDK separately - it's included as a submodule!

## Getting Started

1. **Clone with submodules**:
   ```
   git clone --recurse-submodules https://github.com/Jonno-edu/phoenix-avionics.git
   cd phoenix-avionics
   ```

2. **Open in VS Code**:
   ```
   code .
   ```

3. **Select build variant** in CMake Tools status bar
4. **Build and run!**

## Build Variants

This project uses `cmake-variants.yaml` to define build targets. You can switch variants in the VS Code status bar (click the CMake variant selector, usually showing "Debug" or "Release").

- **Debug-pico2_w**: Builds for the Raspberry Pi Pico 2 W (RP2350) hardware.
- **Debug-mac_host**: Builds a native executable for macOS.

## VS Code Tasks

Press `Cmd+Shift+P` and type `Run Task` to access the pre-configured tasks:

- **Build Active Variant**: Compiles the code for the currently selected CMake variant.
- **Build & Flash Active Variant**:
  - *Host*: Builds and runs the simulation.
  - *Pico*: Builds and flashes the `.uf2` file to a connected Pico in BOOTSEL mode.
- **Build, Flash & Monitor Active Variant**:
  - *Host*: Builds and runs.
  - *Pico*: Builds, flashes, and opens a serial monitor (requires `screen`).
- **Clean Workspace**: Removes the `build/` directory.

## FreeRTOS Configuration

The project handles platform differences using the `PICO_BUILD` macro (defined in `CMakeLists.txt`):

- **Pico Build**: Uses `pico/stdlib.h`, `hardware/gpio.h`, and hardware timers.
- **Host Build**: Uses standard C headers (`stdio.h`, `unistd.h`) and simulates GPIO/Timing.

See `main.c` for examples of how to wrap platform-specific code:

```c
#if PICO_BUILD
    stdio_init_all();
    serial_init();
#else
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("System initialized (simulated)\n");
#endif
```

## Troubleshooting

### Picotool Issues

- **"No accessible RP2040 devices in BOOTSEL mode"**: Ensure you hold the BOOTSEL button while plugging in the Pico.
- **Permission denied**: You might need `sudo` or udev rules (on Linux) to access the USB device.

### CMake Configuration

- If CMake fails to configure, try running the **Clean Workspace** task and re-configuring.
- Ensure `PICO_SDK_PATH` is correctly set in your environment or VS Code settings.

## License

MIT License. See [LICENSE](LICENSE) for details.
