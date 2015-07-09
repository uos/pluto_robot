/*
 * Studygroup MUFFIN Packages - Robot Operating System
 *
 * Copyright (C) 2013 University of Osnabrück, Germany
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * lvr_slicer_node.cpp
 *
 *  Created on: 22.08.2013
 *      Author: Henning Deeken    <hdeeken@uos.de>
 *              Ann-Katrin Häuser <ahaeuser@uos.de>
 */

#include <lvr_slicer_node.h> 

namespace lvr_slicer_node{

LvrSlicerNode::LvrSlicerNode(ros::NodeHandle n)
{
	nh = n;
	ros::NodeHandle nh_ns("~");
	nh_ns.param("verbose_lvr", verbose_lvr, false);
	nh_ns.param("activate_callback", activate_callback, false);

	nh_ns.param("dimension", dimension, string("z"));
	nh_ns.param("single_slice_value", single_slice_value, 1.0);
	nh_ns.param("multi_slice_min_value", multi_slice_min_value, 0.0);
	nh_ns.param("multi_slice_max_value", multi_slice_max_value, 1.0);
	nh_ns.param("multi_slice_resolution", multi_slice_resolution, 0.05);
	nh_ns.param("grid_resolution", grid_resolution, 0.05);
	nh_ns.param("grid_unoccupied_default", grid_unoccupied_default, 1.0);

	// advertise the services for slicing
	get_single_slice_grid_srv = nh.advertiseService("get_single_slice_grid", &LvrSlicerNode::srvGetSingleSliceGrid, this);
	get_multi_slice_grid_srv = nh.advertiseService("get_multi_slice_grid", &LvrSlicerNode::srvGetMultiSliceGrid, this);
	
	get_single_slice_segments_srv = nh.advertiseService("get_single_slice_segments", &LvrSlicerNode::srvGetSingleSliceSegments, this);
	get_multi_slice_segments_srv = nh.advertiseService("get_multi_slice_segments", &LvrSlicerNode::srvGetMultiSliceSegments, this);

	ROS_INFO("LVR Slicer Node");
	ROS_INFO("Advertising GetSingleSliceGrid Service on 'get_single_slice_grid'.");
	ROS_INFO("Advertising GetMultiSliceGrid Service  on 'get_multi_slice_grid'. \n");
	ROS_INFO("Advertising GetSingleSliceSegments Service on 'get_single_slice_segments'.");
	ROS_INFO("Advertising GetMultiSliceSegments Service  on 'get_multi_slice_segments'. \n");

	// if the node is in callback mode, connect the respective topics
	if(activate_callback)
	{
		mesh_sub  = nh.subscribe("/mesh", 1, &LvrSlicerNode::mesh_callback, this);
		single_slice_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/single_grid", 1);
		single_slice_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/single_grid_marker", 1);
		multi_slice_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/multi_grid", 1);
		multi_slice_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/multi_grid_marker", 1);
		ROS_INFO("Publising SingleSliceGrids and Markers on %s and %s.", single_slice_grid_topic.c_str(), single_slice_marker_topic.c_str());
		ROS_INFO("Publising MultiSliceGrids an Markers on %s and %s.", multi_slice_grid_topic.c_str(), multi_slice_marker_topic.c_str());
	}
}

std::vector<geometry_msgs::Point> LvrSlicerNode::convertToGeometryMsgsPoint(std::vector<double> input)
{
    std::vector<geometry_msgs::Point> output;

    for(unsigned int i = 0; i < input.size(); i+=3)
    {
        geometry_msgs::Point point;
        point.x = input.at(i);
        point.y = input.at(i+1);
        point.z = input.at(i+2);
        output.push_back(point);
    }

    return output;
}

bool LvrSlicerNode::projectTo2D(std::vector<double> input, std::string dimension, std::vector<double>& output)
{
    for(unsigned int i = 0; i < input.size(); i+=3)
    {
        if(dimension == "x")
        {
           output.push_back(input.at(i+1));     
           output.push_back(input.at(i+2));
        }
        else if(dimension == "y")
        {
           output.push_back(input.at(i+0));     
           output.push_back(input.at(i+2));
        }
        else if(dimension == "z")
        {
           output.push_back(input.at(i+0));     
           output.push_back(input.at(i+1));
        }
        else 
          return false;
    }

    return true;
}

vector<double> LvrSlicerNode::createSingleSliceSegments(lvr_ros::Mesh mesh, string dimension, double value)
{
	// convert the incoming ros message into a lvr mesh buffer
	MeshBufferPtr input_mesh = boost::make_shared<MeshBuffer>(converter.convertMeshMessageToMeshBuffer(mesh.mesh));

	// setup the lvr class object with the requested parameters
	slicer.clear();
	slicer.setVerbosity(verbose_lvr);
	slicer.setDimension(dimension);
	slicer.setValue(value);

	// slice the mesh once
	std::vector<float> slice = slicer.addMeshAndCompute2dSlice(input_mesh);
	std::vector<double> segments(slice.begin(), slice.end());
	return segments;
}

vector<double> LvrSlicerNode::createMultiSliceSegments(lvr_ros::Mesh mesh, string dimension, double min_value, double max_value, double resolution)
{
	// convert the incoming ros message into a lvr mesh buffer
	MeshBufferPtr input_mesh = boost::make_shared<MeshBuffer>(converter.convertMeshMessageToMeshBuffer(mesh.mesh));

	// setup the lvr class object with the requested parameters
	slicer.clear();
	slicer.setVerbosity(verbose_lvr);
	slicer.setDimension(dimension);
	slicer.setMinValue(min_value);
	slicer.setMaxValue(max_value);
	slicer.setResolution(resolution);

	// slice the mesh multiple times
	std::vector<float> projection =  slicer.addMeshAndCompute2dProjection(input_mesh);
	std::vector<double> segments(projection.begin(), projection.end());
	return segments;
}

void LvrSlicerNode::mesh_callback(const lvr_ros::TriangleMeshGeometry::ConstPtr& mesh)
{
	/*
	// create a single slice grid
	lvr_ros::GetSingleSliceGrid::Request single_slice_grid_request;
	lvr_ros::GetSingleSliceGrid::Response single_slice_response;

	single_slice_grid_request.dimension = dimension;
	single_slice_grid_request.value = single_slice_value;
	single_slice_grid_request.mesh = *mesh;
	single_slice_grid_request.grid_resolution = grid_resolution;
	single_slice_grid_request.grid_unoccupied_default = grid_unoccupied_default;

	single_slice_response = createSingleSliceGrid(single_slice_grid_request);

	single_slice_grid_pub.publish(single_slice_response.grid);
	single_slice_marker_pub.publish(single_slice_response.marker);

	// create a multi slice grid
	lvr_ros::GetMultiSliceGrid::Request multi_slice_grid_request;
	lvr_ros::GetMultiSliceGrid::Response multi_slice_response;

	multi_slice_grid_request.dimension = dimension;
	multi_slice_grid_request.min_value = multi_slice_min_value;
	multi_slice_grid_request.max_value = multi_slice_max_value;
	multi_slice_grid_request.mesh = *mesh;
	multi_slice_grid_request.slicing_resolution = multi_slice_resolution;
	multi_slice_grid_request.grid_resolution = grid_resolution;
	multi_slice_grid_request.grid_unoccupied_default = grid_unoccupied_default;

	multi_slice_response = createMultiSliceGrid(multi_slice_grid_request);

	multi_slice_grid_pub.publish(multi_slice_response.grid);
	multi_slice_marker_pub.publish(multi_slice_response.marker);
	* */
}

bool LvrSlicerNode::srvGetSingleSliceGrid(lvr_ros::GetSingleSliceGrid::Request& request, lvr_ros::GetSingleSliceGrid::Response& response)
{
	vector<double> slice = createSingleSliceSegments(request.mesh, request.dimension, request.value);

	/// TODO exchange with grid_creation lib
	//vector<geometry_msgs::Point> slice_points = convertToGeometryMsgsPoints(slice);
	//response.grid = createOccupancyGrid(request.mesh.header.frame_id, request.mesh.header.stamp, request.mesh.pose, slice_points, request.grid_resolution, request.grid_unoccupied_default);
	//response.marker = createMarkerArray(request.mesh.header.frame_id, request.mesh.pose, slice_points, request.color);

	return true;
}

bool LvrSlicerNode::srvGetMultiSliceGrid(lvr_ros::GetMultiSliceGrid::Request& request, lvr_ros::GetMultiSliceGrid::Response& response)
{
	vector<double> projection = createMultiSliceSegments(request.mesh, request.dimension, request.min_value, request.max_value, request.slicing_resolution);

	/// TODO exchange with grid_creation lib
	//vector<geometry_msgs::Point> projection_points = convertToGeometryMsgsPoints(projection);
	//response.grid = createOccupancyGrid(request.mesh.header.frame_id, request.mesh.header.stamp, request.mesh.pose, projection_points, request.grid_resolution, request.grid_unoccupied_default);
	//response.marker = createMarkerArray(request.mesh.header.frame_id, request.mesh.pose, projection_points, request.color);

	return true;
}

bool LvrSlicerNode::srvGetSingleSliceSegments(lvr_ros::GetSingleSliceSegments::Request& request, lvr_ros::GetSingleSliceSegments::Response& response)
{
	vector<double> segments = createSingleSliceSegments(request.mesh, request.dimension, request.value);
	projectTo2D(segments, request.dimension, response.segments);
	response.points = convertToGeometryMsgsPoint(segments);

	return true;
}

bool LvrSlicerNode::srvGetMultiSliceSegments(lvr_ros::GetMultiSliceSegments::Request& request, lvr_ros::GetMultiSliceSegments::Response& response)
{
	
	vector<double> segments = createMultiSliceSegments(request.mesh, request.dimension, request.min_value, request.max_value, request.slicing_resolution);
	projectTo2D(segments, request.dimension, response.segments);
	response.points = convertToGeometryMsgsPoint(segments);
	return true;
}

} 

// Let the magic happen...

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lvr_slicer_node");
	ros::NodeHandle n;
	lvr_slicer_node::LvrSlicerNode node(n);

	// using a loop rate frees cpu capacity, however not necessary
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
