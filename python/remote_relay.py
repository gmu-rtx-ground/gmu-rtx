import rospy
import os
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

rospy.init_node("Joy_relay")

remote_control = {
    'Thr_Cmd': 0.0,
    'Str_Cmd': 0.0,
    'ROS_ctrl': False,
    'Aux1': 0.0,
    'Aux2': 0.0,
    'Aux3': 0.0
}

def joy_callback(msg):
    remote_control['Thr_Cmd'] = msg.axes[1]
    remote_control['Str_Cmd'] = msg.axes[0]
    remote_control['ROS_ctrl'] = True if msg.axes[2] > 0.0 else False
    remote_control['Aux1'] = msg.axes[3]
    remote_control['Aux2'] = msg.axes[4]
    remote_control['Aux3'] = msg.axes[5]

rospy.Subscriber("/joy", Joy, joy_callback, queue_size=200)
stats_pub = rospy.Publisher("/cmd_vel1", Float32MultiArray, queue_size=10)

while not rospy.is_shutdown():
    if remote_control['ROS_ctrl']:
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"Throttle: {remote_control['Thr_Cmd']}, Steering: {remote_control['Str_Cmd']}")
        msg = Float32MultiArray(data = [remote_control['Str_Cmd'], remote_control['Thr_Cmd'], -0.8])
        stats_pub.publish(msg)
    else:
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"Remote control disabled")
        pass
    rospy.sleep(0.02)