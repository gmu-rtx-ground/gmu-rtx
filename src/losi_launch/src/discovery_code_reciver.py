#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.devices import NetworkDiscoveryStatus
import serial
import time
import rospy
from sensor_msgs.msg import NavSatFix
import utm
from geometry_msgs.msg import PoseStamped

# import serial.tools.list_ports
# ports = serial.tools.list_ports.comports()
# for port in ports:
#     print(port.device)
def shutdown_hook():
    if device is not None and device.is_open():
        device.close()  
        rospy.loginfo("Shutting down: Closing XBee device...")
        
rospy.on_shutdown(shutdown_hook)
        

# === XBee setup ===
READ_BAUD = 9600
PORT_READ = "/dev/xBee"
WRITE_BAUD = 9600

device = XBeeDevice(PORT_READ, READ_BAUD)
device.open()
print("Device Opened")

# remote = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A200FFFF"))


# === Optional: Network discovery ===
# xnet = device.get_network()
# xnet.set_discovery_timeout(15)

# def device_found_callback(remote):
#   print(f"Discovered: {remote.get_64bit_addr()}, NI: {remote.get_node_id()}")

# xnet.add_device_discovered_callback(device_found_callback)

# print("Starting network discovery...")
# xnet.start_discovery_process(deep=True)
# while xnet.is_discovery_running():
#     time.sleep(0.5)
# print("Discovery complete.")

rospy.init_node("xbee_gps_to_goal")
pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)


# rospy.spin()


# === Main loop ===
while not rospy.is_shutdown():
    message = device.read_data()
    if message:
        # timestamp = message.timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')
        # timestamp = message.timestamp.strftime('%H:%M:%S')
        payload = message.data.decode('utf-8', errors='replace')

        # Debug: print raw payload
        print(f"Received: {payload.strip()}")

        # Optional: Forward raw message to COM port
        # full_message = f"[{timestamp}] {payload}\n"
        # ser4.write(full_message.encode('utf-8'))

        # Parse GPS data
        try:
            lat, long = map(float, payload.strip().split(','))
            # gps_msg = NavSatFix()
            # gps_msg.header.stamp = rospy.Time.now()
            # gps_msg.header.frame_id = "target_gps"
            # gps_msg.latitude = lat
            # gps_msg.longitude = long
            # gps_msg.altitude = alt
            # gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            # Convert to UTM
            x, y, zone_num, zone_letter = utm.from_latlon(lat, long)

            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "utm"  # or "utm" if you're working in UTM frame

            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0  # facing forward

            pub.publish(goal)
            rospy.loginfo(f"Published goal: lat={lat}, lon={long} → x={x}, y={y}")
            
            # pub.publish(gps_msg)
            # rospy.loginfo(f"Published NavSatFix: {lat}, {long}")
            rospy.spin()

        except Exception as e:
            print(f"Invalid GPS data or parse error: {e}")

        finally:
            shutdown_hook()


    time.sleep(0.25)
