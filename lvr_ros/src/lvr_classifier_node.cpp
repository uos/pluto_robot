/**
 * Copyright (C) 2013 University of Osnabrück
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
 */


/**
 * @file lvr_classifier_node.cpp
 * @author Simon Herkenhoff <sherkenh@gmail.com>
 */

#include <lvr_classifier_node.h>

namespace lvr_classifier_node
{

LvrClassifierNode::LvrClassifierNode(ros::NodeHandle n)
{
	ros::NodeHandle nh_ns("~");

	detect_rooms_srv = n.advertiseService("detect_rooms", &LvrClassifierNode::detectRoomsSrvCallback, this);

	ROS_INFO("LVR classifier node started!");
	ROS_INFO("Advertising service \"classify\". Happy classifying.");
}

bool LvrClassifierNode::detectRoomsSrvCallback(lvr_ros::DetectRooms::Request& request, lvr_ros::DetectRooms::Response& response)
{
	// some variables needed here
	int ii;

	// set the initial response
	response.mesh = request.mesh;

	// loop over all prelabeled faces
	//for(ii=0; ii<request.mesh.labeledfaces.size(); ii++)
	//{
		//ROS_INFO(" -> label number %d is %s", (ii+1), request.mesh.labeledfaces[ii].label.c_str());
	//}
	ROS_INFO("there are %d labels in this message.", request.mesh.labeledfaces.size());
	ROS_INFO("length of faces array is %d", request.mesh.mesh.faces.size());
	ROS_INFO("length of vertex array is %d", request.mesh.mesh.vertices.size());
	return true;
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lvr_classifier_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");

	lvr_classifier_node::LvrClassifierNode node(nh);

	ros::Rate r(10);
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

