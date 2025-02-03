#ifndef RC_H_
#define RC_H_

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Joy.h>
#include "def.h"
#include "robot_controller.h"


#define USB_USBCON
extern unsigned long lastMsgTime;
extern ros::NodeHandle nodeHandle;
void control_callback(const std_msgs::Float32MultiArray&  control_msg);
extern sensor_msgs::Joy joyMsg;
extern std_msgs::Float32MultiArray robotStatus;
extern ros::Subscriber<std_msgs::Float32MultiArray> controlSub;
extern ros::Publisher joyPub;
extern ros::Publisher rStatPub;
void init_ros_node();

#endif
