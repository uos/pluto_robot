cmake_minimum_required(VERSION 2.4.6)

project(box_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
  INCLUDE_DIRS include
  
  LIBRARIES pointcloud_filters laser_scan_filters
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
)

add_library(box_filter src/box_filter.cpp)
target_link_libraries(box_filter ${catkin_LIBRARIES})

link_directories(
  ${catkin_LIBRARY_DIRS}
)

install(TARGETS box_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES box_filter_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
