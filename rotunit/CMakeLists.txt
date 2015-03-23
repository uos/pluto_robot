cmake_minimum_required(VERSION 2.4.6)

project(rotunit)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

add_executable(rotunit src/rotunit.cpp src/can.cpp)
target_link_libraries(rotunit ${catkin_LIBRARIES})
add_dependencies(rotunit ${catkin_EXPORTED_TARGETS})

install(TARGETS
  rotunit
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
