cmake_minimum_required(VERSION 2.4.6)

project(lvr_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  dynamic_reconfigure
  driver_base
  pcl_conversions
  pcl_ros
  nav_msgs
  message_generation
)

find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

### add OpenMP flags
find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

add_definitions("-frounding-math")
add_definitions("-std=c++0x")
add_definitions(${PCL_DEFINITIONS})
add_definitions(-D_USE_PCL_)

FIND_PATH(OPENGL_INC gl.h /usr/include/GL)
include_directories(${OPENGL_INC})

#find_package(PCL 1.0 REQUIRED)
#include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

include_directories(
  include/${PROJECT_NAME}
  ${PCL_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /include/lvr
  /home/robot/ros/muffin-dry/mapping/lvr/ext/psimpl
  /home/robot/ros/muffin-dry/mapping/lvr/ext/rply
  /home/robot/ros/muffin-dry/mapping/lvr/ext/stann
  /home/robot/ros/muffin-dry/mapping/lvr/ext/
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${CGAL_LIBRARY_DIRS}
  ${OPENGL_LIBRARY_DIRS}
  ${GLUT_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  /home/robot/ros/muffin-dry/mapping/lvr/lib/
)


add_message_files(
  FILES
  BoundingBox.msg
  Face.msg
  LabeledFaces.msg
  MaterialGroup.msg
  Mesh.msg
  PointNormals.msg
  PolygonMesh.msg
  Polygon.msg
  PolygonRegion.msg
  Textures.msg
  TriangleMeshGeometry.msg
)

add_service_files(
  FILES
  AddToFusionMesh.srv
  CreateMesh.srv
  CreatePolygonMesh.srv
  DetectRooms.srv
  GetFusionMesh.srv
  GetMultiSliceGrid.srv
  GetMultiSliceSegments.srv
  GetSingleSliceGrid.srv
  GetSingleSliceSegments.srv
  PolygonFusion.srv
  ResetFusionMesh.srv
  SaveFusionMesh.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  shape_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/lvr.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs  visualization_msgs dynamic_reconfigure driver_base pcl_conversions pcl_ros nav_msgs message_runtime 
  INCLUDE_DIRS include/${PROJECT_NAME}
  DEPENDS PCL OPENGL GLUT CGAL Boost
  LIBRARIES lvr_ros_converter
) 

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${CGAL_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  lvr
  freenect
)


### lvr_ros_converter
add_library(lvr_ros_converter src/lvr_ros_converter.cpp)
target_link_libraries(lvr_ros_converter ${LINK_LIBS})

### lvr_reconstruction_node
add_executable(lvr_reconstruction_node src/lvr_reconstruction_node.cpp)
set_target_properties(lvr_reconstruction_node PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS})
target_link_libraries(lvr_reconstruction_node lvr_ros_converter ${LINK_LIBS})

### lvr_fusion_node
add_executable(lvr_fusion_node src/lvr_fusion_node.cpp)
target_link_libraries(lvr_fusion_node lvr_ros_converter ${LINK_LIBS} 'gmp')

### lvr_slicer_node
add_executable(lvr_slicer_node src/lvr_slicer_node.cpp)
target_link_libraries(lvr_slicer_node lvr_ros_converter ${LINK_LIBS})

### lvr_classifier_node
add_executable(lvr_classifier_node src/lvr_classifier_node.cpp)
target_link_libraries(lvr_classifier_node lvr_ros_converter ${LINK_LIBS})

### mesh_publisher
add_executable(mesh_publisher src/mesh_publisher_node.cpp)
target_link_libraries(mesh_publisher lvr_ros_converter ${LINK_LIBS})

add_dependencies(lvr_ros_converter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(lvr_ros_converter ${catkin_EXPORT_TARGETS} ${PROJECT_NAME}_gencfg)

install(
  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS lvr_reconstruction_node lvr_fusion_node lvr_classifier_node lvr_slicer_node mesh_publisher 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
