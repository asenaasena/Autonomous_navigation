cmake_minimum_required(VERSION 3.2.3)
project(n03)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs

)
  ##PCL 1.2 REQUIRED
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
add_executable(n03  src/n03.cpp)
target_link_libraries(n03 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set (CMAKE_CXX_FLAGS "-lstdc++fs -std=c++17")

