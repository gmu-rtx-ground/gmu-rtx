#include "ros_node.h"
#include "util.h"

unsigned long lastMsgTime = 0;
ros::NodeHandle nodeHandle;
sensor_msgs::Joy joyMsg;
std_msgs::Float32MultiArray robotStatus;
ros::Subscriber<std_msgs::Float32MultiArray> controlSub("/cmd_vel1", &control_callback);
ros::Publisher joyPub("joy", &joyMsg);
ros::Publisher rStatPub("losi_stats", &robotStatus);

float rosThr = 0.0;
float rosSteering = 0.0;
float rosAcc = 0.0;

void init_ros_node(){
  nodeHandle.initNode();
  nodeHandle.subscribe(controlSub);
  nodeHandle.advertise(joyPub);
  nodeHandle.advertise(rStatPub);
}

void control_callback(const std_msgs::Float32MultiArray &control_msg){
  //timestamp the  last ros message
  lastMsgTime = millis();
  //Handle for steering command
  rosSteering = clip(control_msg.data[0], -1.0, 1.0);
  rosThr =  clip(control_msg.data[1], -1.0, 1.0);
  rosAcc =  clip(control_msg.data[2], 0.0, 1.0);
  rosAcc = fmap(rosAcc, 0.0, 1.0, 0.0, maxAcc);
}

