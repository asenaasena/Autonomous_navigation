cmake_minimum_required(VERSION 2.8.3)
project(error_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   scripts/simon_script1
   nodes/simon_node1
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
