cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS} -g")
project(icp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp src/icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})
