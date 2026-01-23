# ============================================================================
# Phoenix Avionics - Raspberry Pi Pico 2 Development Environment
# Cross-platform Docker build environment for Windows, Linux, and macOS
# ============================================================================
FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Build arguments for customization
ARG PICO_SDK_VERSION=2.1.0
ARG FREERTOS_VERSION=V11.2.0

# Environment variables
ENV PICO_SDK_PATH=/opt/pico-sdk
ENV PICO_BOARD=pico2_w

# ============================================================================
# Install all required packages
# ============================================================================
RUN apt-get update && apt-get install -y \
    # Build essentials
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    # ARM toolchain for Pico
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    # Version control
    git \
    # Python for various tools
    python3 \
    python3-pip \
    python3-serial \
    # USB/picotool dependencies
    libusb-1.0-0-dev \
    # SSH tools for remote flashing
    openssh-client \
    sshpass \
    # Serial monitor tools
    minicom \
    screen \
    # Utilities
    curl \
    wget \
    unzip \
    file \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Install Pico SDK (used as fallback if submodule not initialized)
# ============================================================================
RUN git clone --branch ${PICO_SDK_VERSION} --depth 1 \
    https://github.com/raspberrypi/pico-sdk.git ${PICO_SDK_PATH} \
    && cd ${PICO_SDK_PATH} \
    && git submodule update --init

# ============================================================================
# Build and install picotool from source
# ============================================================================
RUN git clone --depth 1 https://github.com/raspberrypi/picotool.git /tmp/picotool \
    && cd /tmp/picotool \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/picotool

# ============================================================================
# Create workspace and set permissions
# ============================================================================
WORKDIR /workspace

# Set default shell to bash
SHELL ["/bin/bash", "-c"]

# Default command
CMD ["/bin/bash"]
