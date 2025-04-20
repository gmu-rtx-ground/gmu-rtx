from digi.xbee.devices import XBeeDevice

# Replace with your serial port and baud rate
PORT = "COM16"  # e.g. "/dev/ttyUSB0" on Linux/Mac
BAUD_RATE = 9600

# Create XBee device
device = XBeeDevice(PORT, BAUD_RATE)

try:
    device.open()

    # Set important AT parameters
    device.set_parameter("CH", b"\x0C")       # Channel
    device.set_parameter("ID", b"\x7F\xFF")   # PAN ID
    device.set_parameter("CE", b"\x01")       # Coordinator Enable
    device.set_parameter("NI", b"0x20SENDER") # Node Identifier
    device.set_parameter("MR", b"\x01")       # Mesh Routing
    device.set_parameter("NT", b"\x82")       # Node discovery timeout
    device.set_parameter("NO", b"\x00")       # Node discovery options
    device.set_parameter("NH", b"\x07")       # Max hops
    device.set_parameter("TO", b"\xC0")       # Transmit options
    device.set_parameter("BH", b"\x00")       # Broadcast hops

    # Apply changes and write to flash
    device.write_changes()

    print("Configuration applied successfully!")

finally:
    if device.is_open():
        device.close()
