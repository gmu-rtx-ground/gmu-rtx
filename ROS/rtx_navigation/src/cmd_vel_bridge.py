#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

def cmd_vel_callback(msg):
    out = Float32MultiArray()
    # Example conversion: linear.x -> throttle, angular.z ->steering
    out.data = [msg.angular.z, msg.linear.x, 0.0]
    pub.publish(out)

rospy.init_node('cmd_vel_bridge')
pub = rospy.Publisher('/cmd_vel1', Float32MultiArray, queue_size=10)
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
rospy.spin()
