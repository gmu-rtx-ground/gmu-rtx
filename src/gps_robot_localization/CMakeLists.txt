cmake_minimum_required(VERSION 3.0.2)
project(gps_robot_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  robot_localization
  tf
)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)