import utm as UTM

from rospy import Subscriber, init_node, loginfo, spin
from sensor_msgs.msg import NavSatFix


LAT: float = 0.0
LON: float = 0.0
FRAME = None

def callback(data):
    global LAT, LON, FRAME
    loginfo(f"{data.status}")

    LAT = data.latitude
    LON = data.longitude

    print(f"LAT: {LAT}\nLON: {LON}")

    FRAME = UTM.from_latlon(LAT, LON)

    print(FRAME)

def listener():
    init_node("test_subscribe")
    Subscriber('/losi/f9p_rover/fix', NavSatFix, callback)
    spin()

if __name__=='__main__':
    listener()

    frame = utm.conversion.from_latlon(LAT, LON)
    print(frame)

