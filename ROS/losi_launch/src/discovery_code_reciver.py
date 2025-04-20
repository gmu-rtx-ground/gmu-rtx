from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.devices import NetworkDiscoveryStatus
import serial
import time
import rospy
from sensor_msgs.msg import NavSatFix

# === XBee setup ===
READ_BAUD = 115200
PORT_READ = "xBee"
PORT_WRITE = "COM4"
WRITE_BAUD = 115200

device = XBeeDevice(PORT_READ, READ_BAUD)
device.open()

remote = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20012345678"))

# === ROS setup ===
rospy.init_node("xbee_gps_receiver")
pub = rospy.Publisher("/xbee_gps", NavSatFix, queue_size=10)

# === Optional: Network discovery ===
xnet = device.get_network()
xnet.set_discovery_timeout(15)

def device_found_callback(remote):
    print(f"Discovered: {remote.get_64bit_addr()}, NI: {remote.get_node_id()}")

xnet.add_device_discovered_callback(device_found_callback)

print("Starting network discovery...")
xnet.start_discovery_process(deep=True)
while xnet.is_discovery_running():
    time.sleep(0.5)
print("Discovery complete.")

# === Serial for forwarding (optional) ===
ser4 = serial.Serial(PORT_WRITE, WRITE_BAUD)

# === Main loop ===
while not rospy.is_shutdown():
    message = device.read_data()
    if message:
        timestamp = message.timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')
        payload = message.data.decode('utf-8', errors='replace')

        # Debug: print raw payload
        print(f"Received: {payload.strip()}")

        # Optional: Forward raw message to COM port
        full_message = f"[{timestamp}] {payload}\n"
        ser4.write(full_message.encode('utf-8'))

        # Parse GPS data
        try:
            lat, long = map(float, payload.strip().split(','))
            gps_msg = NavSatFix()
            gps_msg.header.stamp = rospy.Time.now()
            gps_msg.header.frame_id = "target_gps"
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            # gps_msg.altitude = alt
            # gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            pub.publish(gps_msg)
            rospy.loginfo(f"Published NavSatFix: {lat}, {lon}")
            rospy.spinOnce()

        except Exception as e:
            print(f"Invalid GPS data or parse error: {e}")


    time.sleep(0.25)
