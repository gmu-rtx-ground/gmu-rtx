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

READ_BAUD = 115200
PORT_READ = "COM3"
PORT_WRITE = "COM4"
WRITE_BAUD = 115200
device = XBeeDevice(PORT_READ, READ_BAUD)
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


ser4 = serial.serial(PORT_WRITE, WRITE_BAUD)

while(True):
    message = device.read_data()
    if message:
        # Get timestamp and payload
        timestamp = message.timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')  # Convert to string
        payload = message.data.decode('utf-8', errors='replace')  # Cleanly decode bytes

        # Format the outgoing message
        full_message = f"[{timestamp}] {payload}\n"

        # Encode and send to COM port
        ser4.write(full_message.encode('utf-8'))

        print(f"Forwarded: {full_message.strip()}")
    time.sleep(0.25)



   