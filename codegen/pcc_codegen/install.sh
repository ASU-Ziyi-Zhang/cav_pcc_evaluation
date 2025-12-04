#!/bin/sh

## Run cmake and install the built files to lib/
if [ -d build ]; then
    rm -r build
fi 

if [ -d lib ]; then
    rm -r lib
fi

# Settings
BUILD_TYPE=Release # Debug, RelWithDebInfo, Release

# cmake
mkdir build

cmake -B build -DCMAKE_BUILD_TYPE=$BUILD_TYPE
cmake --build build --config $BUILD_TYPE
cmake --install build --config $BUILD_TYPE

## Run tests
sh ./test.sh

## Clean up build files
rm -r build
