cmake_minimum_required(VERSION 2.8.3)
project(pluto_levers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  pluto_bringup
  message_filters
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    sensor_msgs
    tf
    pluto_bringup
    message_filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/pluto_levers_node.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

