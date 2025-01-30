#include "utils.h"
#include <ros/ros.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "hello_world");

  ros::NodeHandle nh;
  ROS_INFO("Hello, world!");
  // nh.setParam("foo", 42); // Use the ROS parameter server to store a value
  // nh.getParam("foo", foo); // Retrieve the value from the parameter server

  std::uint8_t foo = 42;

  ROS_INFO("foo = %d", foo);
  return 0;
}
