#include "ros_node.h"
#include "util.h"

namespace ros_node {
  unsigned long lastMsgTime = 0;
  float rosAux = 0.0;
  float rosThr = 0.0;
  float rosSteering = 0.0;
  uint8_t rosMsgFlag = 0;
  ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1024> nodeHandle;
  sensor_msgs::Joy joyMsg;
  std_msgs::Float32MultiArray robotStatus;
  ros::Publisher joyPub("joy", &joyMsg);
  ros::Publisher rStatPub("losi_stats", &robotStatus);
  ros::Subscriber<std_msgs::Float32MultiArray> controlSub("/cmd_vel1", &control_callback);
}

void init_ros_node(){
  ros_node::nodeHandle.getHardware()->setBaud(baudRate);  
  ros_node::nodeHandle.initNode();
  ros_node::nodeHandle.subscribe(ros_node::controlSub);
  ros_node::nodeHandle.advertise(ros_node::joyPub);
  ros_node::nodeHandle.advertise(ros_node::rStatPub);
}

void control_callback(const std_msgs::Float32MultiArray &control_msg){
  // timestamp the  last ros message
  ros_node::lastMsgTime = micros();
  ros_node::rosMsgFlag = 0;

  // Handle for steering command
  ros_node::rosSteering = clip(control_msg.data[0], -1.0, 1.0);
  ros_node::rosThr =  clip(control_msg.data[1], -1.0, 1.0);
  ros_node::rosAux =  clip(control_msg.data[2], -1.0, 1.0);
}

