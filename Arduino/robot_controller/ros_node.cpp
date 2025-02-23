#include "ros_node.h"
#include "util.h"

unsigned long lastMsgTime = 0;
ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1024> nodeHandle;
sensor_msgs::Joy joyMsg;
std_msgs::Float32MultiArray robotStatus;
ros::Subscriber<std_msgs::Float32MultiArray> controlSub("/cmd_vel1", &control_callback);
ros::Publisher joyPub("joy", &joyMsg);
ros::Publisher rStatPub("losi_stats", &robotStatus);
float rosThr = 0.0;
float rosSteering = 0.0;
float rosAux = 0.0;
uint8_t rosMsgFlag = 0;

void init_ros_node(){
  nodeHandle.getHardware()->setBaud(115200);
  
  nodeHandle.initNode();
  nodeHandle.subscribe(controlSub);
  nodeHandle.advertise(joyPub);
  nodeHandle.advertise(rStatPub);
}

void control_callback(const std_msgs::Float32MultiArray &control_msg){
  //timestamp the  last ros message
  lastMsgTime = micros();
  rosMsgFlag = 0;
  //Handle for steering command
  rosSteering = clip(control_msg.data[0], -1.0, 1.0);
  rosThr =  clip(control_msg.data[1], -1.0, 1.0);
  rosAux =  clip(control_msg.data[2], -1.0, 1.0);
  
}

