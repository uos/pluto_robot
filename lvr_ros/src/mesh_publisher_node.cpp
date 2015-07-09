/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 *
 * mesh_publisher_node.cpp
 *
 * Created on: 31.08.2013
 *     Author: Henning Deeken <hdeeken@uos.de>
 *             Ann-Katrin Häuser <ahaeuser@uos.de>
 */

#include <mesh_publisher_node.h> 

namespace mesh_publisher_node{

MeshPublisherNode::MeshPublisherNode(ros::NodeHandle h)
{
	nh = h; 
	ros::NodeHandle nh_ns("~");
	nh_ns.param("from_file_path", from_file_path, string("/home/user/mymesh.ply"));
	nh_ns.param("from_file_topic", from_file_topic, string("from_file"));
	nh_ns.param("frame", frame, string("base_link"));
	nh_ns.param("rate", rate, .25);
	nh_ns.param("to_file_path", to_file_path, string("/home/user/"));
	nh_ns.param("to_file_topic", to_file_topic, string("to_file"));

	mesh_pub = nh.advertise<lvr_ros::Mesh>(from_file_topic, 1);
	mesh_sub = nh.subscribe(to_file_topic, 1, &MeshPublisherNode::mesh_callback, this);
	ROS_INFO("Subscribed to %s", to_file_topic.c_str(), frame.c_str());
	ROS_INFO("Writing to %s", to_file_path.c_str());
	ROS_INFO("Reading from %s", from_file_path.c_str());
	ROS_INFO("Publising on %s with frame %s", from_file_topic.c_str(), frame.c_str());
	n=0;
}

double MeshPublisherNode::getRate()
{
	return rate;
}

void MeshPublisherNode::readAndPublish()
{
	lvr_ros::TriangleMeshGeometry triangle_mesh = converter.readTriangleMeshGeometry(from_file_path, frame);
	
	lvr_ros::Mesh mesh;
	mesh.header = triangle_mesh.header;
	mesh.header.frame_id = frame;
	mesh.header.stamp = ros::Time::now();
	
	mesh.mesh = triangle_mesh;
	mesh_pub.publish(mesh);
}

void MeshPublisherNode::mesh_callback(const lvr_ros::TriangleMeshGeometry::ConstPtr& mesh)
{
		converter.writeTriangleMeshGeometry(*mesh, to_file_path + static_cast<ostringstream*>( &(ostringstream() << n) )->str() + "mesh.ply");
		n++;
}

}

// Let the magic happen...

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mesh_publisher");
	ros::NodeHandle nh;
	mesh_publisher_node::MeshPublisherNode node(nh);
	ros::Rate loop_rate(node.getRate());

	while(ros::ok())
	{
		node.readAndPublish();
		ros::spinOnce();
		loop_rate.sleep();
	}
 
	return 0;
}
