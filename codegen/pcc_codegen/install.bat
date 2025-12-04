@echo off
REM Run CMake and install built files to lib/

REM Remove existing 'build' and 'lib' directories if they exist
if exist build rmdir /s /q build
if exist lib rmdir /s /q lib

REM Settings -- Options: Debug, RelWithDebInfo, Release
SET BUILD_TYPE=Release

REM Create build directory
mkdir build

REM Run CMake commands
cmake -B build -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
cmake --build build --config %BUILD_TYPE%
cmake --install build --config %BUILD_TYPE%

REM Run tests
call test.bat

REM Clean up build files
echo Cleaning up...
rmdir /s /q build