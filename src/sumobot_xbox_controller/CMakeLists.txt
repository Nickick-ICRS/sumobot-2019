cmake_minimum_required(VERSION 2.8.3)
project(sumobot_xbox_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sumobot_xbox_controller
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

install(PROGRAMS
  scripts/node
  scripts/xbox.py
  scripts/config.py
  scripts/xbox_listener.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
