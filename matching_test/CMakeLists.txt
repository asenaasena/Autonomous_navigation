cmake_minimum_required(VERSION 3.16.3)
project(matching_test)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
 
)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "-std=c++14")

##add_dependencies(matching_test pointcloud_vector_gencpp)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
add_executable(matching_test src/matching_test.cpp)
target_link_libraries(matching_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})



