#!/bin/bash

set -e

if [ ! -z "$TRAVIS_BUILD_DIR" ]; then
	export GITHUB_WORKSPACE="$TRAVIS_BUILD_DIR"
	export GITHUB_REPOSITORY="$TRAVIS_REPO_SLUG"
elif [ -z "$GITHUB_WORKSPACE" ]; then
	export GITHUB_WORKSPACE="$PWD"
	export GITHUB_REPOSITORY="esphome/async-mqtt-client"
fi

# PlatformIO Test
source ./.github/scripts/install-platformio.sh

echo "Installing ESPAsyncTCP ..."
python -m platformio lib --storage-dir "$GITHUB_WORKSPACE" install

build_pio_sketches "esp12e" "$GITHUB_WORKSPACE/examples/FullyFeatured-ESP8266"
build_pio_sketches "esp32dev" "$GITHUB_WORKSPACE/examples/FullyFeatured-ESP32"
