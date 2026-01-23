# Phoenix Avionics - Docker Development Environment

This document explains how to set up and use the Docker-based development environment for the Phoenix Avionics project. This setup provides a consistent build environment across Windows, Linux, and macOS.

## Prerequisites

### All Platforms
1. **Docker Desktop** (Windows/macOS) or **Docker Engine** (Linux)
   - Windows: [Download Docker Desktop](https://www.docker.com/products/docker-desktop/)
   - macOS: [Download Docker Desktop](https://www.docker.com/products/docker-desktop/)
   - Linux: `sudo apt install docker.io docker-compose`

2. **Git** with SSH key configured for GitHub

3. **VS Code** (recommended) with extensions:
   - Dev Containers (ms-vscode-remote.remote-containers)
   - C/C++ (ms-vscode.cpptools)
   - CMake Tools (ms-vscode.cmake-tools)

### SSH Key Setup for Remote Flashing
The flash script uses SSH to connect to the Rock Pi. Ensure you have:
1. SSH key generated: `ssh-keygen -t ed25519`
2. Public key added to Rock Pi: `ssh-copy-id rock@100.115.224.114`

## Quick Start

### Option 1: VS Code Dev Containers (Recommended)

1. **Open project in VS Code**
2. **Press F1** and select "Dev Containers: Reopen in Container"
3. Wait for container to build (first time takes ~5-10 minutes)
4. **Build**: Press F7 or use CMake Tools
5. **Flash**: Run task "Flash via SSH Rockchip" (Ctrl+Shift+P → Tasks: Run Task)

### Option 2: Command Line (All Platforms)

```bash
# Initial setup (first time only)
make setup

# Build firmware
make build

# Build and flash to Pico
make flash

# Open serial monitor
make monitor

# Open interactive shell in container
make docker-shell
```

### Option 3: Windows Command Prompt

```batch
:: Build firmware
build.bat

:: Build and flash
flash.bat
```

## Configuration

### Rock Pi Connection Settings

Copy the example environment file and customize:

```bash
cp .env.example .env
```

Edit `.env` with your Rock Pi settings:
```
ROCK_PI_HOST=100.115.224.114
ROCK_PI_USER=rock
ROCK_PI_PORT=22
```

### Build Variants

The project supports different build targets:

| Target | Board | Description |
|--------|-------|-------------|
| RP2350 | pico2_w | Raspberry Pi Pico 2 W (default) |
| Host | host | macOS/Linux for unit testing |

## Project Structure

```
phoenix-avionics/
├── .devcontainer/       # VS Code Dev Container config
│   └── devcontainer.json
├── .vscode/             # VS Code settings
├── scripts/             # Build and flash scripts
│   ├── build.sh         # Main build script
│   ├── flash.sh         # Remote flash script
│   ├── monitor.sh       # Serial monitor script
│   ├── clean.sh         # Clean build artifacts
│   └── setup-submodules.sh
├── src/                 # Source code
├── include/             # Header files
├── lib/                 # Git submodules (pico-sdk, FreeRTOS, etc.)
├── Dockerfile           # Docker image definition
├── docker-compose.yml   # Docker Compose config
├── Makefile             # Convenience targets
├── build.bat            # Windows build script
└── flash.bat            # Windows flash script
```

## Common Tasks

### Building

```bash
# Standard build
make build

# Clean and rebuild
make clean
make build

# Build inside container shell
make docker-shell
./scripts/build.sh
```

### Flashing

```bash
# Build and flash
make flash

# Flash without rebuilding
make flash-only

# Flash and open monitor
make flash-monitor
```

### Monitoring Serial Output

```bash
# Open serial monitor
make monitor
```

Press `Ctrl+A` then `X` to exit minicom.

### Interactive Development

```bash
# Open shell in container
make docker-shell

# Now you can run any commands
./scripts/build.sh
cmake --build build/Release-rp2350 --target flash
```

## Troubleshooting

### Docker Issues

**Container won't start:**
```bash
# Remove old containers and rebuild
docker-compose down
docker-compose build --no-cache
```

**Permission denied on Linux:**
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Log out and back in
```

### SSH/Flashing Issues

**Can't connect to Rock Pi:**
```bash
# Test SSH connection
ssh rock@100.115.224.114

# If prompted for password, copy your SSH key
ssh-copy-id rock@100.115.224.114
```

**Pico not detected:**
- Check USB connection to Rock Pi
- Verify Pico is powered
- Check if picotool is installed on Rock Pi: `ssh rock@100.115.224.114 'which picotool'`

### Build Issues

**Submodules not initialized:**
```bash
# Inside container
./scripts/setup-submodules.sh

# Or manually
git submodule update --init --recursive
```

**CMake configuration fails:**
```bash
# Clean and reconfigure
make clean
make build
```

## VS Code Tasks

The following tasks are available (Ctrl+Shift+P → Tasks: Run Task):

| Task | Description |
|------|-------------|
| Build Active Variant | Build using CMake Tools |
| Flash via SSH Rockchip | Build and flash to Pico |
| Monitor via SSH Rockchip | Open serial monitor |
| Flash and Monitor | Flash then monitor |
| Clean and Reconfigure | Delete build dir and reconfigure |

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Computer                            │
│  ┌───────────────────────────────────────────────────────┐  │
│  │              Docker Container                         │  │
│  │  ┌─────────────┐  ┌──────────────┐  ┌─────────────┐  │  │
│  │  │  pico-sdk   │  │  ARM GCC     │  │  picotool   │  │  │
│  │  │  FreeRTOS   │  │  CMake/Ninja │  │  SSH client │  │  │
│  │  └─────────────┘  └──────────────┘  └──────────────┘  │  │
│  │                         │                             │  │
│  │                    Build .uf2                         │  │
│  └─────────────────────────┼─────────────────────────────┘  │
│                            │ SCP                            │
└────────────────────────────┼────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────┐
│                      Rock Pi (SBC)                         │
│  ┌──────────────┐                     ┌─────────────────┐  │
│  │   picotool   │────USB Cable────────│  Pico 2 W       │  │
│  │              │                     │  (RP2350)       │  │
│  └──────────────┘                     └─────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

## Contributing

1. Clone with submodules: `git clone --recursive <repo-url>`
2. Set up Docker environment: `make setup`
3. Create a feature branch
4. Make changes and test: `make build && make flash`
5. Submit pull request

## License

See [LICENSE](LICENSE) file.
