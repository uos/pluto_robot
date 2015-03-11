cmake_minimum_required(VERSION 2.4.6)

project(laser_ps3_control)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs actionlib actionlib_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs actionlib actionlib_msgs
)

add_executable(laser_ps3_control src/laser_ps3_control.cpp)
target_link_libraries(laser_ps3_control ${catkin_LIBRARIES})
add_dependencies(laser_ps3_control ${catkin_EXPORTED_TARGETS})

install(TARGETS
  laser_ps3_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
