# ============================================================================
# Phoenix Avionics - Makefile
# Convenience targets for building and flashing
# ============================================================================

.PHONY: all build flash monitor clean docker-build docker-shell help setup

# Default target
all: build

# Build using Docker
build:
	@docker-compose run --rm pico-dev ./scripts/build.sh

# Build and flash to Pico via Rock Pi
flash:
	@docker-compose run --rm pico-dev ./scripts/flash.sh

# Flash without rebuilding
flash-only:
	@docker-compose run --rm pico-dev ./scripts/flash.sh --skip-build

# Flash and open serial monitor
flash-monitor:
	@docker-compose run --rm pico-dev ./scripts/flash.sh --monitor

# Open serial monitor
monitor:
	@docker-compose run --rm pico-dev ./scripts/monitor.sh

# Clean build artifacts
clean:
	@docker-compose run --rm pico-dev ./scripts/clean.sh

# Build the Docker image
docker-build:
	@docker-compose build

# Open interactive shell in Docker container
docker-shell:
	@docker-compose run --rm pico-dev bash

# Setup: build Docker image and initialize submodules
setup: docker-build
	@docker-compose run --rm pico-dev ./scripts/setup-submodules.sh

# Stop and remove containers
docker-down:
	@docker-compose down

# Show help
help:
	@echo "Phoenix Avionics - Available Targets"
	@echo "====================================="
	@echo ""
	@echo "  make build         - Build the firmware"
	@echo "  make flash         - Build and flash to Pico via Rock Pi"
	@echo "  make flash-only    - Flash without rebuilding"
	@echo "  make flash-monitor - Flash and open serial monitor"
	@echo "  make monitor       - Open serial monitor"
	@echo "  make clean         - Clean build artifacts"
	@echo ""
	@echo "Docker Commands:"
	@echo "  make setup         - Initial setup (build image, init submodules)"
	@echo "  make docker-build  - Build the Docker image"
	@echo "  make docker-shell  - Open interactive shell in container"
	@echo "  make docker-down   - Stop and remove containers"
	@echo ""
	@echo "Configuration:"
	@echo "  Copy .env.example to .env and edit Rock Pi settings"
