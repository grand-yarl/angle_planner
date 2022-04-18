cmake_minimum_required(VERSION 3.0.2)
project(angle_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  tf
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  roscpp
  std_msgs
  genmsg
)

generate_dynamic_reconfigure_options(cfg/AnglePlanner.cfg)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)


add_library(angle_planner_lib src/angle_planner.cpp)

add_library(${PROJECT_NAME} src/angle_planner.cpp)
add_dependencies(${PROJECT_NAME} ${angle_planner_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(angle_planner ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
