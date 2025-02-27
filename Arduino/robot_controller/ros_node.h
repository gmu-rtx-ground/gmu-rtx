#ifndef ROS_NODE_H_
#define ROS_NODE_H_

#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include "def.h"
#include "robot_controller.h"

// #define USB_USBCON // unused

namespace ros_node {
    extern unsigned long lastMsgTime;
    extern float rosAux;
    extern float rosThr;
    extern float rosSteering;
    extern uint8_t rosMsgFlag;
    extern ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1024>  nodeHandle;
    extern sensor_msgs::Joy joyMsg;
    extern std_msgs::Float32MultiArray robotStatus;
    extern ros::Publisher joyPub;
    extern ros::Publisher rStatPub;
    extern ros::Subscriber<std_msgs::Float32MultiArray> controlSub;
    
}   // namespace ros_node

void init_ros_node();
void control_callback(const std_msgs::Float32MultiArray&  control_msg);

#endif  // ROS_NODE_H_
