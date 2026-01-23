@echo off
REM ============================================================================
REM Phoenix Avionics - Windows Build Script
REM Runs the build inside Docker container
REM ============================================================================

echo ==============================================
echo Phoenix Avionics - Docker Build (Windows)
echo ==============================================
echo.

REM Check if Docker is running
docker info >nul 2>&1
if errorlevel 1 (
    echo Error: Docker is not running!
    echo Please start Docker Desktop and try again.
    exit /b 1
)

REM Build the Docker image if needed
echo Building Docker image...
docker-compose build

REM Run the build inside container
echo.
echo Running build...
docker-compose run --rm pico-dev ./scripts/build.sh

echo.
echo Build complete!
