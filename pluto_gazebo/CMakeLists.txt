cmake_minimum_required(VERSION 3.0.2)
project(pluto_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY models/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)
install(DIRECTORY worlds/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
