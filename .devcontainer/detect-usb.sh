#!/bin/bash

# List connected USB devices
USB_DEVICES=$(ls /dev | grep ttyUSB)

echo "Detected USB devices: $USB_DEVICES"

# Check if a specific device is connected and set permissions accordingly
if [[ "$USB_DEVICES" == *"ttyUSB0"* ]]; then
    echo "Applying permissions for ttyUSB0..."
    sudo chmod 666 /dev/witImu
else
    echo "ttyUSB0 device not found."
fi

if [[ "$USB_DEVICES" == *"ttyACM0"* ]]; then
    echo "Applying permissions for ttyACM0..."
    sudo chmod 666 /dev/ubloxGPS
else
    echo "ttyACM0 device not found."
fi
