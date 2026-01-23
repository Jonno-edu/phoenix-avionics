@echo off
REM ============================================================================
REM Phoenix Avionics - Windows Flash Script
REM Builds and flashes via Docker container
REM ============================================================================

echo ==============================================
echo Phoenix Avionics - Docker Flash (Windows)
echo ==============================================
echo.

REM Check if Docker is running
docker info >nul 2>&1
if errorlevel 1 (
    echo Error: Docker is not running!
    echo Please start Docker Desktop and try again.
    exit /b 1
)

REM Run the flash script inside container
docker-compose run --rm pico-dev ./scripts/flash.sh %*
