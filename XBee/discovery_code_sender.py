from digi.xbee.devices import XBeeDevice
from digi.xbee.devices import XBeeException
from digi.xbee.devices import RemoteXBeeDevice
from digi.xbee.devices import XBee64BitAddress
from digi.xbee.devices import NeighborDiscoveryMode
from digi.xbee.devices import NetworkDiscoveryStatus
import sys
import serial
import time
from datetime import timedelta

PORT_BAUD = 115200
PORT = "COM16"
device = XBeeDevice(PORT, PORT_BAUD)
remote = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20012345678"))


device.open()

# Read the operating PAN ID of the device.
dest_addr = device.get_dest_address()

# Read the operating PAN ID of the device.
pan_id = device.get_pan_id()

# Read the output power level.
p_level = device.get_power_level()

print(pan_id)

try:

    # Access the network object
    xnet = device.get_network()

    # Set discovery timeout (in seconds)
    xnet.set_discovery_timeout(15)

    # Callback function for each discovered device
    def device_found_callback(remote):
        print(f"Discovered: {remote.get_64bit_addr()}, NI: {remote.get_node_id()}")

    # Add callback
    xnet.add_device_discovered_callback(device_found_callback)



    # Start discovery
    print("Starting network discovery...")
    xnet.start_discovery_process(deep=True)

    # Wait until it's finished
    while xnet.is_discovery_running():
        time.sleep(0.5)

    print("Discovery complete.")

except Exception as e:
    print(f"Error: {e}")



discovered_devices = xnet.get_devices()

print(f"\nFound {len(discovered_devices)} device(s):")
for remote in discovered_devices:
    addr = remote.get_64bit_addr()
    ni = remote.get_node_id()
    print(f"Device Address: {addr}, Node Identifier: {ni}")


message = "Hello Xbee!"
counter = 1  #will increment up as messages are sent

print("\nStarting message loop. Press Ctrl+C to stop.")
try:
    while True:
        try:
            device.send_data(remote, message)
            print(f"[{time.strftime('%H:%M:%S')}] Sent #{counter}: {message}") #spam message in terminal
            counter += 1
        except Exception as e:
            print(f"Error sending message: {e}")
        time.sleep(0.25)
except KeyboardInterrupt:
    print("\nStopped by user.") #just to stop showing the large error message
finally:
    device.close()
    print("Device closed.")