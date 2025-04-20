#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

def cmd_vel_callback(msg):
    out = Float32MultiArray(data=[0.0, 0.0, 0.0])
    # Example conversion: linear.x -> throttle, angular.z ->steering
    out.data[0] = -np.clip(msg.angular.z*6.25, -1.0, 1.0)  # Convert to steering
    out.data[1] = np.clip(msg.linear.x*2, -1.0, 1.0)  # Convert to throttle
    
    pub.publish(out)

rospy.init_node('cmd_vel_bridge')
pub = rospy.Publisher('/cmd_vel1', Float32MultiArray, queue_size=10)
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
rospy.spin()
