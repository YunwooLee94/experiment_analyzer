cmake_minimum_required(VERSION 3.0.2)
project(experiment_analyzer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf_conversions
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
        LIBRARIES
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        std_msgs
)

file(GLOB FLIGHT_DATA_SRC src/Wrapper.cpp)

add_executable(interpreter exe/main.cpp ${FLIGHT_DATA_SRC})
target_link_libraries(interpreter ${catkin_LIBRARIES})
