cmake_minimum_required(VERSION 3.10.2)
project(eu_cluster3)

add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
std_msgs
sensor_msgs
jsk_recognition_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 # CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs jsk_recognition_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(eu_cluster3 src/eu_cluster3.cpp)


target_link_libraries(eu_cluster3
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
