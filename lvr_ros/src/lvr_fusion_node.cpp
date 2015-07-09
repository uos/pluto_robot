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
 * lvr_fusion_node.cpp
 *
 *  Created on: 31.08.2013
 *      Author: Henning Deeken <hdeeken@uos.de>
 *              Ann-Katrin Häuser <ahaueser@uos.de>
 */

#include <lvr_fusion_node.h> 

namespace lvr_fusion_node
{

LvrFusionNode::LvrFusionNode(ros::NodeHandle h)
{
    nh = h;
    ros::NodeHandle nh_ns("~");

    nh_ns.param("verbose_lvr", verbose_lvr, false);
    nh_ns.param("use_callback", use_callback, false);
    nh_ns.param("distance_threshold", distance_threshold, 0.05);
    nh_ns.param("type", type, string("integrate"));

    if ( use_callback )
    {
        input_mesh_sub = nh.subscribe("reconstruction/mesh", 1,
                &LvrFusionNode::mesh_callback, this);
        output_mesh_pub = nh.advertise<lvr_ros::Mesh>("fusion/mesh", 1);
    }

    add_to_fusion_mesh_srv = nh.advertiseService("add_to_fusion_mesh",
            &LvrFusionNode::srvAddToFusionMesh, this);
    get_fusion_mesh_srv = nh.advertiseService("get_fusion_mesh",
            &LvrFusionNode::srvGetFusionMesh, this);
    save_fusion_mesh_srv = nh.advertiseService("save_fusion_mesh",
            &LvrFusionNode::srvSaveFusionMesh, this);
    reset_fusion_mesh_srv = nh.advertiseService("reset_fusion_mesh",
            &LvrFusionNode::srvResetFusionMesh, this);

    fusion = new Fusion<fVertex, fNormal>;
    fusion->setVerbosity(verbose_lvr);
    fusion->setDistanceThreshold(distance_threshold);
    init = false;

    ROS_INFO("LVR Fusion Node is online.");
}

// Functionality

bool LvrFusionNode::addToFusionMesh(lvr_ros::TriangleMeshGeometry mesh)
{
    if ( !init )
    {
        frame = mesh.header.frame_id;
        init = true;
    }

    if ( frame == mesh.header.frame_id )
    {
        try
        {
            MeshBuffer input_mesh = converter.convertMeshMessageToMeshBuffer(mesh);
            MeshBufferPtr input_mesh_ptr = boost::make_shared<MeshBuffer>(input_mesh);
            
            if(type.compare("integrate") == 0)
            {
                fusion->addMeshAndIntegrate(input_mesh_ptr);
            }
            else if(type.compare("remote") == 0)
            {
                fusion->addMeshAndRemoteIntegrateOnly(input_mesh_ptr);
            }
            else if(type.compare("lazy") == 0)
            {
                fusion->addMeshAndLazyIntegrate(input_mesh_ptr);
            }
            else
            {
                ROS_ERROR("Could not identify type: '%s' \n Abort Fusion!", type.c_str());
                return false;
            }
            return true;
        }
        catch(...)
        {
            ROS_ERROR("Fusion failed!");
            return false;
        }

        return true;
    }
    else
    {
        ROS_ERROR(
                "FusionMeshNode recieved Mesh with different frame_id then the orginal Mesh. This should not happen. Current Mesh will not be fused.");
        ROS_INFO(
                "Set Frame_Id is: %s and should be %s", mesh.header.frame_id.c_str(), frame.c_str());
        return false;
    }

    return false;
}

bool LvrFusionNode::finalizeFusionMesh(lvr_ros::TriangleMeshGeometry *mesh)
{
    fusion->finalize();
    *mesh = converter.convertMeshBufferToMeshMessage(fusion->meshBuffer(), 
        ros::Time::now(), frame);

    return true;
}

// Services

bool LvrFusionNode::srvResetFusionMesh(lvr_ros::ResetFusionMesh::Request& request,
        lvr_ros::ResetFusionMesh::Response& response)
{
    ROS_INFO("ResetFusionMesh Service was called...");

    delete fusion;
    fusion = new Fusion<fVertex, fNormal>;
    fusion->setVerbosity(verbose_lvr);
    fusion->setDistanceThreshold(distance_threshold);
    init = false;

    ROS_INFO("Cleaned the Fusion Mesh Buffers...");

    return true;
}

bool LvrFusionNode::srvSaveFusionMesh(lvr_ros::SaveFusionMesh::Request& request,
        lvr_ros::SaveFusionMesh::Response& response)
{
    ROS_INFO("SaveFusionMesh Service was called...");

    fusion->finalize();
    converter.writeMeshBufferPtr(fusion->meshBuffer(), request.path);

    ROS_INFO("Wrote mesh to %s.", request.path.c_str());
    return true;
}

bool LvrFusionNode::srvGetFusionMesh(lvr_ros::GetFusionMesh::Request& request,
        lvr_ros::GetFusionMesh::Response& response)
{
    ROS_INFO("GetFusionMesh Service was called...");

    if ( init )
    {
        finalizeFusionMesh(&response.mesh.mesh);

        response.mesh.header = response.mesh.mesh.header;

        ROS_INFO("Current Mesh:");
        ROS_INFO("#Vertices %d", (int) response.mesh.mesh.vertices.size());
        ROS_INFO("#Faces    %d", (int) response.mesh.mesh.faces.size());
        ROS_INFO("Result with %s", response.mesh.header.frame_id.c_str());
        return true;
    }
    else
    {
        ROS_WARN("No FusionMesh generated yet. There's no return.");
        return false;
    }

    return false;
}

bool LvrFusionNode::srvAddToFusionMesh(lvr_ros::AddToFusionMesh::Request& request,
        lvr_ros::AddToFusionMesh::Response& response)
{
    ROS_INFO("AddToFusionMesh Service was called...");
    return addToFusionMesh(request.mesh.mesh);
}

// Callback
void LvrFusionNode::mesh_callback(lvr_ros::Mesh mesh)
{
    ROS_INFO("FusionMeshNode - Callback... \n");
    addToFusionMesh(mesh.mesh);
    lvr_ros::Mesh output_mesh;
    finalizeFusionMesh(&output_mesh.mesh);
    output_mesh_pub.publish(output_mesh);
}
}

// Let the magic happen...

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lvr_fusion_node");
    ros::NodeHandle nh;

    lvr_fusion_node::LvrFusionNode node(nh);
    ros::Rate loop_rate(10);

    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
