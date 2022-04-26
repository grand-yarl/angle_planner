cmake_minimum_required(VERSION 3.0.2)
project(orientation_pub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(orientation_pub src/orientation_pub.cpp)
target_link_libraries(orientation_pub ${catkin_LIBRARIES})

