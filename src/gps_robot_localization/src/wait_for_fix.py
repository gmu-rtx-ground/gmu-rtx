#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import time

class WaitForFix:
    def __init__(self):
        rospy.init_node('wait_for_fix')
        
        # Parameters
        self.gps_topic = rospy.get_param('~gps_topic', '/losi/f9p_rover/fix')
        self.min_satellites = rospy.get_param('~min_satellites', 5)
        self.timeout = rospy.get_param('~timeout', 60.0)
        
        # Publishers and subscribers
        self.fix_ready_pub = rospy.Publisher('/gps_fix_ready', Bool, queue_size=1)
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)
        
        # State variables
        self.fix_received = False
        self.start_time = time.time()
        
        rospy.loginfo("Waiting for good GPS fix on topic %s...", self.gps_topic)
        
    def gps_callback(self, msg):
        # Check GPS status
        # NavSatFix doesn't directly provide satellite count, so we'll use status and covariance
        # Status: -1 (No fix), 0 (Fix), 1 (SBAS fix), 2 (GBAS fix)
        if msg.status.status >= 0:  # Has a fix
            # Check position covariance (lower is better)
            # Using the trace of the covariance matrix as a quality indicator
            position_variance = msg.position_covariance[0] + msg.position_covariance[4] + msg.position_covariance[8]
            
            # Check if variance is reasonable (adjust threshold as needed)
            if position_variance < 10.0:  # Threshold in m^2
                rospy.loginfo("Good GPS fix acquired!")
                self.fix_received = True
                self.fix_ready_pub.publish(Bool(True))
                
                # Log the initial GPS coordinates
                rospy.loginfo("Initial GPS coordinates: Lat: %.8f, Lon: %.8f, Alt: %.2f", 
                             msg.latitude, msg.longitude, msg.altitude)
                
                # Store to parameter server for potential later use
                rospy.set_param('/initial_gps_latitude', msg.latitude)
                rospy.set_param('/initial_gps_longitude', msg.longitude)
                rospy.set_param('/initial_gps_altitude', msg.altitude)
                
                # Unsubscribe after getting a good fix
                self.gps_sub.unregister()
        
    def spin(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown() and not self.fix_received:
            # Check timeout
            if time.time() - self.start_time > self.timeout:
                rospy.logwarn("Timeout waiting for GPS fix. Proceeding without good fix.")
                self.fix_ready_pub.publish(Bool(False))
                break
                
            rate.sleep()
            
if __name__ == '__main__':
    try:
        wait_for_fix = WaitForFix()
        wait_for_fix.spin()
    except rospy.ROSInterruptException:
        pass