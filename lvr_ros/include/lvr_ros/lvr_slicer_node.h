#ifndef LVR_SLICER_NODE_H_
#define LVR_SLICER_NODE_H_

#include <vector>
#include <iostream>

#include <io/PLYIO.hpp>
#include <io/Timestamp.hpp>
#include <io/Progress.hpp>
#include <io/DataStruct.hpp>
#include <io/Model.hpp>
#include <io/MeshBuffer.hpp>
#include <io/ModelFactory.hpp>
#include <slicer/MeshSlicer.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <lvr_ros/Face.h>
#include <lvr_ros/TriangleMeshGeometry.h>
#include <lvr_ros/GetSingleSliceGrid.h>
#include <lvr_ros/GetMultiSliceGrid.h>
#include <lvr_ros/GetSingleSliceSegments.h>
#include <lvr_ros/GetMultiSliceSegments.h>
#include <lvr_ros_converter.h>

//#include <grid_creation/grid_creator.h>

using namespace std;
using namespace lvr;
using namespace nav_msgs;
using namespace visualization_msgs;

namespace lvr_slicer_node
{

/**
 * @brief  The LVR Slicer Node provides a ROS wrapper to the LVR functionality to slice triangle meshes and create occupancy grid maps from them.
 *          The node provides services to create grids from single slices or multiple slices projected into one grid, which can be used 
 *          in robot navigation. Further a callback mode can be enabled that takes in meshes from a topic and outputs grid maps and the respective markers.
 */

class LvrSlicerNode
{

private:
/// node handle
ros::NodeHandle nh;

/// topic to output the grid of a mesh that will be sliced once, only used in callback mode
string single_slice_grid_topic;
/// topic to output the markers of a the mesh that will be sliced once, only used in debug mode
string single_slice_marker_topic;
/// topic to output the grid of a mesh that will be sliced multiple times, only used in callback mode
string multi_slice_grid_topic;
/// topic to output the markers of a the mesh that will be sliced multiple times, only used in debug mode
string multi_slice_marker_topic;

/// subscriber to input the mesh that will be sliced, only used in callback mode
ros::Subscriber mesh_sub;
/// publisher to output the grid of a mesh that will be sliced once, only used in callback mode
ros::Publisher single_slice_grid_pub;
/// publisher to output the markers of a the mesh that will be sliced multiple times, only used in debug mode
ros::Publisher single_slice_marker_pub;
/// publisher to output the grid of a mesh that will be sliced multiple times, only used in callback mode
ros::Publisher multi_slice_grid_pub;
/// publisher to output the markers of a the mesh that will be sliced multiple times, only used in debug mode
ros::Publisher multi_slice_marker_pub;

/// server for the single slice grid service
ros::ServiceServer get_single_slice_grid_srv;
/// server for the multi slice grid service
ros::ServiceServer get_multi_slice_grid_srv;
/// server for the single slice segments service
ros::ServiceServer get_single_slice_segments_srv;
/// server for the multi slice segments service
ros::ServiceServer get_multi_slice_segments_srv;

/// converter object to transform between LVR and ROS types
lvr_ros_converter::LvrRosConverter converter;

/// LVR object that is used to do the acutal slicing process 
MeshSlicer slicer;

/// grid creator
//grid_creation::GridCreator grid_creator_;

/// parameter to set verbosity of the slicing process within the lvr class 
bool verbose_lvr;
/// parameter to enable the callback functionality
bool activate_callback;
/// dimension that orients the plane
string dimension;
///  value set the intersecting plane on the given dimension
double single_slice_value;
///  min value for the intersecting planes on the given dimension
double multi_slice_min_value;
///  max value for the intersecting planes on the given dimension
double multi_slice_max_value;
///  resolution value for the intersecting planes on the given dimension
double multi_slice_resolution;
///  resolution value for the resulting grid map
double grid_resolution;
///  default value for unoccupied values on the resulting grid map
double grid_unoccupied_default;


/**
 * @brief converts the derived segment points in 3D into ROS geometry_msgs::Points (later used for Marker display)
 */

std::vector<geometry_msgs::Point> convertToGeometryMsgsPoint(std::vector<double> input);

/**
 * @brief converts the derived segment points in 3D into a 2D proejction
 *
 * @param input 3d segment points
 * @param dimension on which to project
 * @return 2d proejction points
 */

bool projectTo2D(std::vector<double> input, std::string dimension, std::vector<double>& output);

/**
 * @brief service method to create a grid map from a triangle mesh by slicing it with one specific intersection plane
 *
 * @param dimension on which to project
 * @param response of the lvr_ros::GetSingleSliceGrid ROS Service, see definition for details
 */

vector<double> createSingleSliceSegments(lvr_ros::Mesh mesh, string dimension, double value);

/**
 * @brief service method to create a grid map from a triangle mesh by slicing it multiple times and project them onto one plane
 *
 * @param request of the lvr_ros::GetMultiSliceGrid ROS Service, see definition for details 
 * @return response of the lvr_ros::GetMultiSliceGrid ROS Service, see definition for details
 */

vector<double> createMultiSliceSegments(lvr_ros::Mesh mesh, string dimension, double min_value, double max_value, double resolution);

public:

/**
  * @brief Constructor
  */
LvrSlicerNode(ros::NodeHandle n);

/**
  * @brief Destructor
  */
~LvrSlicerNode() {};

/**
 * @brief calls the creation of single and multi slice grids based on mesh data coming from a topic, mainly for debug and visualization purposes
 *
 * @param mesh a ROS lvr_ros::TriangleMeshGeometry Message
 */
void mesh_callback(const lvr_ros::TriangleMeshGeometry::ConstPtr& mesh);

/**
 * @brief service method to create a grid map from a triangle mesh by slicing it with one specific intersection plane
 *
 * @param request of the lvr_ros::GetSingleSliceGrid ROS Service, see definition for details 
 * @param response of the lvr_ros::GetSingleSliceGrid ROS Service, see definition for details
 */
 
bool srvGetSingleSliceGrid(lvr_ros::GetSingleSliceGrid::Request& request, lvr_ros::GetSingleSliceGrid::Response& response);

/**
 * @brief service method to create a grid map from a triangle mesh by slicing it with multiple intersection planes, along one axis
 *
 * @param request of the lvr_ros::GetSingleSliceGrid ROS Service, see definition for details 
 * @param response of the lvr_ros::GetSingleSliceGrid ROS Service, see definition for details
 */
bool srvGetMultiSliceGrid(lvr_ros::GetMultiSliceGrid::Request& request, lvr_ros::GetMultiSliceGrid::Response& response);


/**
 * @brief service method to create a list of line segments from a triangle mesh by slicing it with one specific intersection plane
 *
 * @param request of the lvr_ros::GetSingleSliceSegments ROS Service, see definition for details 
 * @param response of the lvr_ros::GetSingleSliceSegments ROS Service, see definition for details
 */
 
bool srvGetSingleSliceSegments(lvr_ros::GetSingleSliceSegments::Request& request,  lvr_ros::GetSingleSliceSegments::Response& response);

/**
 * @brief service method to create a list of line segments from a triangle mesh by slicing it with multiple intersection planes, along one axis
 *
 * @param request of the lvr_ros::GetMultiSliceSegments ROS Service, see definition for details 
 * @param response of the lvr_ros::GetMultiSliceSegments ROS Service, see definition for details
 */
bool srvGetMultiSliceSegments(lvr_ros::GetMultiSliceSegments::Request& request,  lvr_ros::GetMultiSliceSegments::Response& response);

};

}

#endif /* LVR_SLICER_NODE_H_ */
