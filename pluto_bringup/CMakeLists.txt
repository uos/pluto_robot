cmake_minimum_required(VERSION 2.8.3)
project(pluto_bringup)

find_package(catkin REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#     pluto_description
#     robot_state_publisher
#     joint_state_publisher
#     # velodyne_pointcloud
#     # volksbot_driver
#     # uos_rotunit_driver
#     # kurt_bringup
#     # uos_rotunit_snapshotter
#     # uos_rotunit_teleop
#     laser_filters
#     sound_play
#     battery_guard
# )

catkin_package()

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
