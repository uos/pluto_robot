cmake_minimum_required(VERSION 2.4.6)

project(map_odom_icp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_service_files(
  FILES
  IcpSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
  INCLUDE_DIRS include
)

add_executable(map_odom_icp src/map_odom_icp.cpp src/ros_pcl_icp.cpp)

target_link_libraries(map_odom_icp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
)

add_dependencies(map_odom_icp ${catkin_EXPORTED_TARGETS})

install(TARGETS
  map_odom_icp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
