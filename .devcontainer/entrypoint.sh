#!/bin/bash

# Function to add device mapping only if the device exists
add_device_if_exists() {
    local DEVICE_PATH=$1
    local DEVICE_NAME=$2
    if [ -e "$DEVICE_PATH" ]; then
        echo "--device=$DEVICE_PATH:$DEVICE_NAME"
    fi
}

# Construct the Docker run arguments dynamically
RUN_ARGS=""
RUN_ARGS+=" $(add_device_if_exists /dev/ttyUSB0 /dev/witImu)"
RUN_ARGS+=" $(add_device_if_exists /dev/ttyACM0 /dev/ubloxGPS)"

# Start the container with dynamic args
exec docker run $RUN_ARGS my-devcontainer
