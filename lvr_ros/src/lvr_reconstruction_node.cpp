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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * lvr_reconstruction_node.cpp
 *
 * Author: Tristan Igelbrink <tigelbri@uos.de>, 
 *         Johannes Heitmann <joheitma@uos.de>,
 *         Simon Herkenhoff <sherkenh@uos.de>, 
 *         Henning Deeken <hdeeken@uos.de>,
 *         Marcel Mrozinski <mmrozins@uos.de>,
 *         Dominik Feldschnieders <dofeldsc@uos.de>
 */

#include "lvr_reconstruction_node.h"

using namespace std;

namespace lvr_reconstruction_node
{

LvrReconstructionNode::LvrReconstructionNode(ros::NodeHandle nh)
{
    m_nodeHandler = nh; 
    bool useCallback;
    ros::NodeHandle nh_ns("~");
    nh_ns.param("use_callback", useCallback, false);

    m_dynRecCallback = boost::bind(&LvrReconstructionNode::cfgcallback, this, _1, _2);
    m_dynRecServer.setCallback(m_dynRecCallback);

    if(useCallback)
    {
        ROS_INFO("Using callback...");
        m_pointCloudSubscriber = m_nodeHandler.subscribe("/mapping_cloud", 1, &LvrReconstructionNode::callback, this);
        m_meshPublisher = m_nodeHandler.advertise<lvr_ros::Mesh>("/reconstruction/mesh", 1);
        m_pointNormalsPublisher = m_nodeHandler.advertise<lvr_ros::PointNormals>("/reconstruction/point_normals", 1);
    }
    else
    {
        ROS_INFO("Not using callback...");
    }

    m_createMeshService = m_nodeHandler.advertiseService("create_mesh", &LvrReconstructionNode::srvCreateMesh, this);
    m_createPolygonMeshService = m_nodeHandler.advertiseService("create_polygonmesh", &LvrReconstructionNode::srvCreatePolygonMesh, this);
    m_doPolygonFusionService = m_nodeHandler.advertiseService("polygon_fusion", &LvrReconstructionNode::srvDoFusion, this);

// debug
    polygon_mesh_pub_  = nh.advertise<lvr_ros::PolygonMesh>("Fusion_Mesh_YEAH", 1);
    polygon_mesh_pub2_ = nh.advertise<lvr_ros::PolygonMesh>("Fusion_Mesh_final", 1);
//debug ende

    ROS_INFO("LVR Reconstruction Node");
}

/*
    @brief processes an incoming PointCloud2 and publishes a mesh/triangle
*/

void LvrReconstructionNode::callback(sensor_msgs::PointCloud2 cloud)
{
    ROS_INFO("LVR Reconstruction received a pointcloud!");

    lvr_ros::Mesh mesh;
    lvr_ros::PointNormals normals;

    createMesh(cloud, &mesh, &normals);

    m_meshPublisher.publish(mesh);
    m_pointNormalsPublisher.publish(normals);
}

bool LvrReconstructionNode::createMesh(sensor_msgs::PointCloud2& inputCloud,
                                       lvr_ros::Mesh *outputMesh,
                                       lvr_ros::PointNormals *outputNormals)
{
    outputMesh->header.stamp = inputCloud.header.stamp;
    outputMesh->header.frame_id = inputCloud.header.frame_id;

    ROS_INFO("Set mesh header to: %s", inputCloud.header.frame_id.c_str());

    setupReconstructionParameters();

    // Create a model
    int numPoints;
    ModelPtr model = m_lvrRosConverter.convertPointCloud2PtrToModelPtr(boost::make_shared<sensor_msgs::PointCloud2>(inputCloud),
                                                                       numPoints);
    PointBufferPtr pLoader = model->m_pointCloud;
    psSurface::Ptr surface;

    if (m_dynRecConfig.pcm == "STANN" ||
        m_dynRecConfig.pcm == "FLANN" ||
        m_dynRecConfig.pcm == "NABO" ||
        m_dynRecConfig.pcm == "NANOFLANN" ||
        m_dynRecConfig.pcm == "PCL")
    {
        // Create point set surface object (AdaptiveKSearchSurface)
        if (m_dynRecConfig.pcm == "PCL")
        {
            // Created later as there is no setter for the point normals
        }

        else
        {
            surface.reset(new akSurface(pLoader,
                                        m_dynRecConfig.pcm,
                                        m_dynRecConfig.kn,
                                        m_dynRecConfig.ki,
                                        m_dynRecConfig.kd));

            // Set RANSAC flag
            if (m_dynRecConfig.ransac)
            {
                static_cast<akSurface*>(&(*surface))->useRansac(true);
            }
        }
    }

    else
    {
        ROS_WARN_STREAM(timestamp << "Unable to create PointCloudManager");
        ROS_WARN_STREAM(timestamp << "Unknown option '" << m_dynRecConfig.pcm << "'.");
    }

    surface->setKd(m_dynRecConfig.kd);
    surface->setKi(m_dynRecConfig.ki);
    surface->setKn(m_dynRecConfig.kn);

    calculateNormals(boost::make_shared<sensor_msgs::PointCloud2>(inputCloud), pLoader, surface, numPoints);

    // create an empty mesh
    HalfEdgeMesh<cVertex, cNormal> mesh(surface);

    // set recursion depth for region growing
    if (m_dynRecConfig.depth)
    {
        mesh.setDepth(m_dynRecConfig.depth);
    }

    float resolution;
    bool useVoxelsize;

    if (m_dynRecConfig.intersections > 0)
    {
        resolution = m_dynRecConfig.intersections;
        useVoxelsize = false;
    }

    else
    {
        resolution = m_dynRecConfig.v;
        useVoxelsize = true;
    }

    // Create a new reconstruction object
    FastReconstruction<cVertex, cNormal> reconstruction(surface,
                                                        resolution,
                                                        useVoxelsize,
                                                        m_dynRecConfig.decomposition,
                                                        !m_dynRecConfig.noExtrusion);

    // Create mesh
    reconstruction.getMesh(mesh);
    optimizeAndFinalize(mesh);

    // get MeshBufferPtr from mesh
    MeshBufferPtr meshBfr;
    meshBfr = mesh.meshBuffer();

    // creating message-types
    *outputNormals = m_lvrRosConverter.convertPointBufferPtrToPointNormalsMessage(pLoader,
                                                                                  inputCloud.header.stamp,
                                                                                  inputCloud.header.frame_id);
    outputNormals->header.frame_id = inputCloud.header.frame_id;
    outputNormals->header.stamp = inputCloud.header.stamp;

    outputMesh->mesh = m_lvrRosConverter.convertMeshBufferToMeshMessage(meshBfr,
                                                                        inputCloud.header.stamp,
                                                                        inputCloud.header.frame_id);
    outputMesh->mesh.header.frame_id = inputCloud.header.frame_id;
    outputMesh->mesh.header.stamp = inputCloud.header.stamp;

    if (m_dynRecConfig.generateTextures)
    {
        outputMesh->textures = m_lvrRosConverter.convertMeshBufferToTextureMessage(meshBfr,
                                                                                   inputCloud.header.stamp,
                                                                                   inputCloud.header.frame_id);
        outputMesh->textures.header.frame_id = inputCloud.header.frame_id;
        outputMesh->textures.header.stamp = inputCloud.header.stamp;
    }

    ROS_ERROR("Publish Mesh");
    m_meshPublisher.publish((*outputMesh));
    return true;
}



bool LvrReconstructionNode::createPolygonMesh(sensor_msgs::PointCloud2& inputCloud,
	        lvr_ros::PolygonMesh *polymesh)
{
/*
	// TODO remove debug zeug und stelle alte Funktionalität wieder her (auskommentiert)
	PolyMesh debug_mesh1;
	PolyMesh debug_mesh2;
	PolyRegion debug_region1;
	PolyRegion debug_region2;
	PolyRegion debug_region3;
	PolyRegion debug_region4;
	PolyRegion debug_region5;
	PolyRegion debug_region6;
	PolyRegion debug_region7;
	Poly debug_poly1;
	Poly debug_poly2;
	Poly debug_poly3;
	Poly debug_poly4;
	Poly debug_poly5;
	Poly debug_poly6;
	Poly debug_poly7;

	cVertex p11(1.0, 1.0, 0.9);
	//cVertex p12(1.0, 2.0, 1.0);
	cVertex p13(1.0, 3.0, 0.9);
	//cVertex p14(2.0, 3.0, 1.0);
	cVertex p15(3.0, 3.0, 1.1);
	//cVertex p16(3.0, 2.0, 1.0);
	cVertex p17(3.0, 1.0, 1.1);

	std::vector<Poly> debug_vector3;
	cVertex p31(6.0, 6.0, 1.1);
	cVertex p32(6.0, 8.0, 1.1);
	cVertex p33(8.0, 8.0, 1.1);
	cVertex p34(8.0, 6.0, 1.1);
	std::vector<cVertex> debug_vector_un;
	debug_vector_un.push_back(p31);
	debug_vector_un.push_back(p32);
	debug_vector_un.push_back(p33);
	debug_vector_un.push_back(p34);
	debug_poly3.setVertices(debug_vector_un);
	debug_vector3.push_back(debug_poly3);
	debug_region3.setPolygons(debug_vector3, "unknown", cNormal(0.0, 0.0, 1.0));

	std::vector<Poly> debug_vector6;
	cVertex p61(6.0, 6.0, 1.00);
	cVertex p62(6.0, 8.0, 1.00);
	cVertex p63(8.0, 8.0, 1.00);
	cVertex p64(8.0, 6.0, 1.00);
	std::vector<cVertex> debug_vector_6;
	debug_vector_6.push_back(p61);
	debug_vector_6.push_back(p62);
	debug_vector_6.push_back(p63);
	debug_vector_6.push_back(p64);
	debug_poly6.setVertices(debug_vector_6);
	debug_vector6.push_back(debug_poly6);
	debug_region6.setPolygons(debug_vector6, "unknown", cNormal(0.0, 0.0, 1.0));


	std::vector<Poly> debug_vector4;
	cVertex p41(-6.0, -6.0, 6.0);
	cVertex p42(-6.0, -6.0, 8.0);
	cVertex p43(-8.0, -6.0, 8.0);
	cVertex p44(-8.0, -6.0, 6.0);
	std::vector<cVertex> debug_vector_4;
	debug_vector_4.push_back(p41);
	debug_vector_4.push_back(p42);
	debug_vector_4.push_back(p43);
	debug_vector_4.push_back(p44);
	debug_poly4.setVertices(debug_vector_4);
	debug_vector4.push_back(debug_poly4);
	debug_region4.setPolygons(debug_vector4, "Nils", cNormal(0.0, 1.0, 0.0));

	std::vector<Poly> debug_vector5;
	cVertex p51(-7.0, -6.0, 7.0);
	cVertex p52(-7.0, -6.0, 9.0);
	cVertex p53(-9.0, -6.0, 9.0);
	cVertex p54(-10.0, -6.0, 8.0);
	cVertex p55(-8.0, -6.0, 6.0);
	std::vector<cVertex> debug_vector_5;
	debug_vector_5.push_back(p51);
	debug_vector_5.push_back(p52);
	debug_vector_5.push_back(p53);
	debug_vector_5.push_back(p54);
	debug_vector_5.push_back(p55);
	debug_poly5.setVertices(debug_vector_5);
	debug_vector5.push_back(debug_poly5);
	debug_region5.setPolygons(debug_vector5, "Nils", cNormal(0.0, 1.0, 0.0));



	std::vector<Poly> debug_vector;
	std::vector<cVertex> debug_vector_p;
	std::string label = "quatsch";
	debug_vector_p.push_back(p11);
	//debug_vector_p.push_back(p12);
	debug_vector_p.push_back(p13);
	//debug_vector_p.push_back(p14);
	debug_vector_p.push_back(p15);
	//debug_vector_p.push_back(p16);
	debug_vector_p.push_back(p17);
	debug_poly1.setVertices(debug_vector_p);
	debug_vector.push_back(debug_poly1);
	debug_region1.setPolygons(debug_vector, label, cNormal(0.0, 0.0, 1.0));

	debug_vector.clear();
	debug_vector_p.clear();

	cVertex p21(2.0, 2.0, 1.1);
	//cVertex p22(2.0, 3.0, 1.0);
	cVertex p23(2.0, 4.0, 1.1);
	//cVertex p24(3.0, 4.0, 1.0);
	cVertex p25(4.0, 4.0, 0.9);
	//cVertex p26(4.0, 3.0, 1.0);
	cVertex p27(4.0, 2.0, 0.9);
	debug_vector_p.push_back(p21);
	//debug_vector_p.push_back(p22);
	debug_vector_p.push_back(p23);
	//debug_vector_p.push_back(p24);
	debug_vector_p.push_back(p25);
	//debug_vector_p.push_back(p26);
	debug_vector_p.push_back(p27);
	debug_poly2.setVertices(debug_vector_p);
	debug_vector.push_back(debug_poly2);
	debug_region2.setPolygons(debug_vector, label, cNormal(0.0, 0.0, 1.0));

	debug_mesh1.addPolyRegion(debug_region4);
	debug_mesh2.addPolyRegion(debug_region5);
	debug_mesh2.addPolyRegion(debug_region1);
	debug_mesh1.addPolyRegion(debug_region2);
	debug_mesh2.addPolyRegion(debug_region3);
	debug_mesh1.addPolyRegion(debug_region6);

	cout << "Publish Fusion_Mesh_before" << endl;


	lvr_ros::PolygonMesh test2;
	lvr_ros::PolygonMesh test3;
	FusiontoROS(debug_mesh1, test2);
	FusiontoROS(debug_mesh2, test3);
	test2.header.frame_id = "/odom_combined";
	test3.header.frame_id = "/odom_combined";
	polygon_mesh_pub2_.publish(test2);
	polygon_mesh_pub2_.publish(test3);

	cout << "REINSTECKEN UND AUSFUEHREN!!!!" << endl;

	m_Poly_Fusion.addFusionMesh(debug_mesh1);
	m_Poly_Fusion.addFusionMesh(debug_mesh2);

	std::vector<PolyRegion> result;
	m_Poly_Fusion.doFusion(result);

	cout << "Ergebnis liefert Vektor der groesse: " << result.size() << endl;
	cout << "Resultierende Polygone:" << endl;
	std::vector<PolyRegion>::iterator it;
	for(it = result.begin() ; it != result.end() ; ++it)
	{
		std::vector<Poly> points=  it->getPolygons();
		std::vector<Poly>::iterator its;
		cout << "Region hat " << points.size() << " Polygone" << std::endl;
		for(its = points.begin() ; its != points.end() ; ++its)
		{
			std::vector<cVertex> vert = its->getVertices();
			std::vector<cVertex>::iterator it2;
			cout << "Polygon hat " << vert.size() << " Punkte" << std::endl;
			for(it2 = vert.begin() ; it2 != vert.end() ; ++it2)
			{
				cout << "x: " << (*it2).x << "  y: " << (*it2).y << "  z: " << (*it2).z << endl;
			}
		}
	}

	PolyMesh test_mesh;
	test_mesh.addPolyRegions(result);

	lvr_ros::PolygonMesh test;
	FusiontoROS(test_mesh, test);
	test.header.frame_id = "/odom_combined";
	polygon_mesh_pub_.publish(test);
*/

	polymesh->header.stamp = inputCloud.header.stamp;
	polymesh->header.frame_id = inputCloud.header.frame_id;

	ROS_INFO(
	        "Set polymesh header to: %s", inputCloud.header.frame_id.c_str());

	setupReconstructionParameters();

	// Create a model
	int numPoints;
	ModelPtr model = m_lvrRosConverter.convertPointCloud2PtrToModelPtr(
	        boost::make_shared<sensor_msgs::PointCloud2>(inputCloud),
	        numPoints);
	PointBufferPtr pLoader = model->m_pointCloud;
	psSurface::Ptr surface;

	if ( m_dynRecConfig.pcm == "STANN" || m_dynRecConfig.pcm == "FLANN"
	        || m_dynRecConfig.pcm == "NABO"
	        || m_dynRecConfig.pcm == "NANOFLANN"
	        || m_dynRecConfig.pcm == "PCL" )
	{
		// Create point set surface object (AdaptiveKSearchSurface)
		if ( m_dynRecConfig.pcm == "PCL" )
		{
			// Created later as there is no setter for the point normals
		}
		else
		{
			surface.reset(
				        new akSurface(pLoader, m_dynRecConfig.pcm,
				                m_dynRecConfig.kn, m_dynRecConfig.ki,
				                m_dynRecConfig.kd));

			// Set RANSAC flag
			if ( m_dynRecConfig.ransac )
			{
				static_cast<akSurface*>(&(*surface))->useRansac(true);
			}
		}
	}

	else
	{
		ROS_WARN_STREAM(timestamp << "Unable to create PointCloudManager");
		ROS_WARN_STREAM(
			        timestamp << "Unknown option '" << m_dynRecConfig.pcm << "'.");
	}

	surface->setKd(m_dynRecConfig.kd);
	surface->setKi(m_dynRecConfig.ki);
	surface->setKn(m_dynRecConfig.kn);

	calculateNormals(
		        boost::make_shared<sensor_msgs::PointCloud2>(inputCloud),
		        pLoader, surface, numPoints);

	// create an empty mesh
	HalfEdgeMesh<cVertex, cNormal> mesh(surface);

	// set recursion depth for region growing
	if ( m_dynRecConfig.depth )
	{
		mesh.setDepth(m_dynRecConfig.depth);
	}

	float resolution;
	bool useVoxelsize;

	if ( m_dynRecConfig.intersections > 0 )
	{
		resolution = m_dynRecConfig.intersections;
		useVoxelsize = false;
	}

	else
	{
		resolution = m_dynRecConfig.v;
		useVoxelsize = true;
	}

	// Create a new reconstruction object
	FastReconstruction<cVertex, cNormal> reconstruction(surface, resolution,
		        useVoxelsize, m_dynRecConfig.decomposition,
		        !m_dynRecConfig.noExtrusion);

	// Create mesh
	reconstruction.getMesh(mesh);
	optimizeAndFinalize(mesh);

	RegionVector regions = mesh.getRegions();
	mesh.resetUsedFlags();

// get and save Regions
	typename RegionVector::iterator region_iter = regions.begin();
	for ( size_t i = 0; region_iter != regions.end(); ++i, ++region_iter )
	{
		if ( (*region_iter)->size() > 0 )
		{
			// create and prepare polygon region
			lvr_ros::PolygonRegion polyregion;
			polyregion.header.frame_id = inputCloud.header.frame_id;
			polyregion.header.stamp = inputCloud.header.stamp;
			polyregion.normal.x = (*region_iter)->m_normal.x;
			polyregion.normal.y = (*region_iter)->m_normal.y;
			polyregion.normal.z = (*region_iter)->m_normal.z;
			if ( (*region_iter)->hasLabel() )
			{
				polyregion.label = (*region_iter)->getLabel();
			}

			// get all polygons for this region
			VertexVectorGroup vertices = (*region_iter)->getContours(m_dynRecConfig.lft);
			typename VertexVectorGroup::iterator vertex_iter = vertices.begin();

			bool not_empty = false;
			for ( size_t j = 0; vertex_iter != vertices.end(); ++j, ++vertex_iter )
			{
				VertexVector current_poly = (*vertex_iter);
				if( current_poly.size() > 2 )
				{
					lvr_ros::Polygon* polygon = new lvr_ros::Polygon();
					polygon->header.frame_id = inputCloud.header.frame_id;
					polygon->header.stamp = inputCloud.header.stamp;

					typename VertexVector::iterator current_poly_iter = current_poly.begin();
					for ( size_t k = 0; current_poly_iter != current_poly.end(); ++k, ++current_poly_iter)
					{
						geometry_msgs::Point32* p = new geometry_msgs::Point32();
						p->x = (*current_poly_iter).x;
						p->y = (*current_poly_iter).y;
						p->z = (*current_poly_iter).z;
						polygon->points.push_back(*p);
					}
					polyregion.polygons.push_back(*polygon);
					not_empty = true;
				}
			}
			// save polyregion if it is not empty
			if(not_empty)
			{
				polymesh->polyregions.push_back(polyregion);
			}
		}
	}

	polygon_mesh_pub_.publish((*polymesh));

// Test Umwandlung TODO remove
/*	PolyMesh lvr_mesh;
	lvr_ros::PolygonMesh lvrtools_mesh;
	lvrtools_mesh.header = polymesh->header;
	ROS_ERROR("CreatepolyMesh ist durch, beginne Transformation");
	ROStoFusion((*polymesh), lvr_mesh);
	ROS_ERROR("Transformation von ros zu fusion abgeschlossen");
	FusiontoROS(lvr_mesh, lvrtools_mesh);
	ROS_ERROR("Transformation von fusion zu ros abgeschlossen...also wirds das zweite gepublished");
	polygon_mesh_pub_.publish(lvrtools_mesh);
*/
// end get and save regions
	return true;
}

bool LvrReconstructionNode::srvDoFusion(lvr_ros::PolygonFusion::Request& request,  lvr_ros::PolygonFusion::Response& response)
{
	bool res_bool;

	std::cout << " srvDoFusion was called!" << std::endl;

	std::vector<lvr_ros::PolygonMesh> poly_meshes = request.polymeshes;
	typename std::vector<lvr_ros::PolygonMesh>::iterator it;
	for(it = poly_meshes.begin() ; it != poly_meshes.end() ; ++it)
	{
		PolyMesh tmp_mesh;
		ROStoFusion((*it), tmp_mesh);

		m_Poly_Fusion.addFusionMesh(tmp_mesh);
	}

	std::vector<PolyRegion> result;
	if ( m_Poly_Fusion.doFusion(result) )
	{

		PolyMesh output_mesh;
		output_mesh.addPolyRegions(result);

		lvr_ros::PolygonMesh output;
		FusiontoROS(output_mesh, output);
		output.header.frame_id = request.frame;

		response.polymesh = output;

		res_bool = true;
	}
	else
	{
		res_bool = false;
	}

	m_Poly_Fusion.reset();
	return res_bool;
}

void LvrReconstructionNode::ROStoFusion(lvr_ros::PolygonMesh input, PolyMesh &output)
{
	// for all regions in this message
	vector<lvr_ros::PolygonRegion>::iterator region_iter;
	for ( region_iter = input.polyregions.begin(); region_iter != input.polyregions.end(); ++region_iter )
	{
		// set initial label and normal
		PolyRegion region;
		region.setLabel((*region_iter).label.c_str());
		cNormal normal;
		normal.x = (*region_iter).normal.x;
		normal.y = (*region_iter).normal.y;
		normal.z = (*region_iter).normal.z;
		region.setNormal(normal);

		std::vector<Poly> polys;

		// for all polygons in this region
		vector<lvr_ros::Polygon>::iterator poly_iter;
		for ( poly_iter = (*region_iter).polygons.begin(); poly_iter != (*region_iter).polygons.end(); ++poly_iter )
		{
			std::vector<cVertex> new_vertices;
			vector<geometry_msgs::Point32>::iterator point_iter;
			for (point_iter = (*poly_iter).points.begin(); point_iter != (*poly_iter).points.end(); ++point_iter)
			{
				cVertex new_vert;
				new_vert.x = (*point_iter).x;
				new_vert.y = (*point_iter).y;
				new_vert.z = (*point_iter).z;
				new_vertices.push_back(new_vert);
			}

			//store the new polygon
			Poly tmp_poly(new_vertices);
			polys.push_back(tmp_poly);
		}
// Aber was ist mit der Zeit und dem Header? unwichtig?
		region.setPolygons(polys, (*region_iter).label.c_str(), normal);

		output.addPolyRegion(region);
	}
}

void LvrReconstructionNode::FusiontoROS(PolyMesh input, lvr_ros::PolygonMesh &output)
{
	//TODO hier noch ein Label setzen wär gut (notwendig)
	// for all regions in this PolygonMesh
	vector<PolyRegion>::iterator region_iter;
	vector<PolyRegion> regions = input.getPolyRegions();
	for ( region_iter = regions.begin(); region_iter != regions.end(); ++region_iter )
	{
		lvr_ros::PolygonRegion polys;
		// for all polygons in this region
		vector<Poly> polygons;
		polygons = (*region_iter).getPolygons();
		vector<Poly>::iterator poly_iter;
		for ( poly_iter = polygons.begin(); poly_iter != polygons.end(); ++poly_iter )
		{
			lvr_ros::Polygon new_polys;
			std::vector<cVertex> old_polys;
			old_polys = (*poly_iter).getVertices();
			std::vector<cVertex>::iterator vert_iter;

			for(vert_iter = old_polys.begin(); vert_iter != old_polys.end(); ++vert_iter)
			{
				geometry_msgs::Point32 tmp_point;
				tmp_point.x = (*vert_iter).x;
				tmp_point.y = (*vert_iter).y;
				tmp_point.z = (*vert_iter).z;

				new_polys.points.push_back(tmp_point);
			}
			polys.polygons.push_back(new_polys);
		}

		output.polyregions.push_back(polys);
	}

}


bool LvrReconstructionNode::srvCreateMesh(lvr_ros::CreateMesh::Request& request,  lvr_ros::CreateMesh::Response& response)
{
    ROS_INFO("CreateMesh SRV was called...");
    return createMesh(request.cloud, &response.mesh, &response.normals);
}

bool LvrReconstructionNode::srvCreatePolygonMesh(lvr_ros::CreatePolygonMesh::Request& request,  lvr_ros::CreatePolygonMesh::Response& response)
{
    ROS_INFO("CreatePolygonMesh SRV was called...");
    createPolygonMesh(request.cloud, &response.polymesh);
    vector<lvr_ros::PolygonRegion>::iterator poly_iter;
    for ( poly_iter = response.polymesh.polyregions.begin(); poly_iter != response.polymesh.polyregions.end(); ++poly_iter )
    {
    	//ROS_ERROR_STREAM("createPolygonMesh: in response.polymesh.polyregions we see " << poly_iter->polygons.size() << " polygons!");
    	if ( (*poly_iter).label != "unknown" )
    	{
    		ROS_WARN_STREAM("createPolygonMesh gave the label " << (*poly_iter).label);
    	}
    }
    ROS_INFO_STREAM("createPolygonMesh iterated over " << response.polymesh.polyregions.size() << " polygon regions!");
    return true;
}

/*
    @brief callback for dynamic reconfiguration of parameters
*/
void LvrReconstructionNode::cfgcallback(lvr_ros::lvrConfig &nConf, uint32_t level)
{
    m_dynRecConfig = nConf;
}

void LvrReconstructionNode::calculateNormals(const sensor_msgs::PointCloud2::ConstPtr& pc,
                                             PointBufferPtr pLoader,
                                             psSurface::Ptr surface,
                                             int numPoints)
{
    if (m_dynRecConfig.pcm != "PCL")
    {
        surface->calculateSurfaceNormals();
    }

    else 
    {
        ROS_INFO("IntegralImageNormalEstimation");
        // ROS to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc, *cloud);

        // output cloud:
        pcl::PointCloud<pcl::PointNormal> normals;

        if (cloud->isOrganized())
        {
            // method:
            pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal> ine;
            ine.setNormalEstimationMethod(pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal>::COVARIANCE_MATRIX);
            ine.setNormalSmoothingSize(m_dynRecConfig.pclSmoothing);
            ine.setDepthDependentSmoothing(true);
            ine.setInputCloud(cloud);
            ine.compute(normals);
            pcl::concatenateFields(*cloud, normals, normals);
        }

        else 
        {
            ROS_WARN("Pointcloud is not organized, cannot use IntegralImageNE");
            pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
            ne.setInputCloud(cloud);
            ne.setSearchMethod( pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
            ne.setKSearch(m_dynRecConfig.kn);
            ne.compute(normals);
        }

        // create a floatArr with normals for all _valid_ points
        float* normalsFloat = new float[numPoints * 3];
        for (int i = 0, j = 0; i < numPoints; i++)
        {
            float x, y, z;
            x = normals[i].x;
            y = normals[i].y;
            z = normals[i].z;
            
            if (!(std::isnan(x) || std::isnan(y) || std::isnan(z)))
            {
                normalsFloat[j * 3 + 0] = normals[i].normal_x;
                normalsFloat[j * 3 + 1] = normals[i].normal_y;
                normalsFloat[j * 3 + 2] = normals[i].normal_z;
                j++;
            }
        }

        floatArr pointNormals = floatArr(normalsFloat);
        pLoader->setPointNormalArray(pointNormals, numPoints);

        ROS_INFO( "... done." );

        surface.reset(new akSurface(pLoader,
                                    "FLANN",
                                     m_dynRecConfig.kn,
                                     m_dynRecConfig.ki,
                                     m_dynRecConfig.kd));
    }
}

void LvrReconstructionNode::optimizeAndFinalize(HalfEdgeMesh<cVertex, cNormal>& mesh)
{
    // optimize
    if (m_dynRecConfig.rda > 0)
    {
        mesh.removeDanglingArtifacts(m_dynRecConfig.rda);
    }

    mesh.cleanContours(m_dynRecConfig.cleanContours);
    // TODO m_dynRecConfig korrekt verwenden!!
    //mesh.setClassifier(m_dynRecConfig.classifier);
    mesh.setClassifier("NormalClassifier");

    if (m_dynRecConfig.optimizePlanes)
    {
        mesh.optimizePlanes(m_dynRecConfig.planeIterations,
                            m_dynRecConfig.pnt,
                            m_dynRecConfig.mp,
                            m_dynRecConfig.smallRegionThreshold,
                            true);
        mesh.fillHoles(m_dynRecConfig.fillHoles);
        mesh.optimizePlaneIntersections();
        mesh.restorePlanes(m_dynRecConfig.mp);

        if (m_dynRecConfig.ecc > 0)
        {
            QuadricVertexCosts<cVertex, cNormal> c = QuadricVertexCosts<cVertex, cNormal>(true);
            mesh.reduceMeshByCollapse(m_dynRecConfig.ecc, c);
        }
    }
    else if (m_dynRecConfig.clusterPlanes)
    {
        mesh.clusterRegions(m_dynRecConfig.pnt, m_dynRecConfig.mp);
        mesh.fillHoles(m_dynRecConfig.fillHoles);
    }

    // finalize
    if (m_dynRecConfig.optimizePlanes && m_dynRecConfig.retesselate)
    {
        mesh.finalizeAndRetesselate(m_dynRecConfig.generateTextures, m_dynRecConfig.lft);
    }
    else
    {
        mesh.finalize();
    }

    // Write classification to file
    if (m_dynRecConfig.writeClassificationResult)
    {
        mesh.writeClassificationResult();
    }
}

void LvrReconstructionNode::setupReconstructionParameters()
{
    // Set number of threads to use
    omp_set_num_threads(m_dynRecConfig.threads);

    if (m_dynRecConfig.texelSize)
    {
        Texture::m_texelSize = m_dynRecConfig.texelSize;
    }

    if (m_dynRecConfig.tp != "")
    {
        Texturizer<cVertex, cNormal>::m_filename = m_dynRecConfig.tp;
        // copied from options.cpp -->
        float* sc = new float[14];
        std::ifstream in (m_dynRecConfig.tp.c_str());    // ?! confusing merge of options.cpp with dynamic reconf
    
        if (in.good())
        {
            for (int i = 0; i < 14; i++)
            {
                in >> sc[i];
            }
            in.close();
        }

        else
        {
            for (int i = 0; i < 14; i++)
            {
                sc[i] = 0.5;
            }
        }

        for (int i = 0; i < 14; i++)
        {
            Statistics::m_coeffs[i] = sc[i];
        }

        delete[] sc;

        if (m_dynRecConfig.nsc)
        {
            Texturizer<cVertex, cNormal>::m_numStatsColors = m_dynRecConfig.nsc;
        }
        
        if (m_dynRecConfig.nccv)
        {
            Texturizer<cVertex, cNormal>::m_numCCVColors = m_dynRecConfig.nccv;
        }

        if (m_dynRecConfig.ct)
        {
            Texturizer<cVertex, cNormal>::m_coherenceThreshold = m_dynRecConfig.ct;
        }

        if (m_dynRecConfig.colt)
        {
            Texturizer<cVertex, cNormal>::m_colorThreshold = m_dynRecConfig.colt;
        }

        if (m_dynRecConfig.stat)
        {
            Texturizer<cVertex, cNormal>::m_statsThreshold = m_dynRecConfig.stat;
        }

        if (m_dynRecConfig.cro)
        {
            Texturizer<cVertex, cNormal>::m_useCrossCorr = m_dynRecConfig.cro;
        }

        if (m_dynRecConfig.feat)
        {
            Texturizer<cVertex, cNormal>::m_featureThreshold = m_dynRecConfig.feat;
        }

        if (m_dynRecConfig.patt)
        {
            Texturizer<cVertex, cNormal>::m_patternThreshold = m_dynRecConfig.patt;
        }

        if (m_dynRecConfig.textureAnalysis)
        {
            Texturizer<cVertex, cNormal>::m_doAnalysis = true;
        }

        if (m_dynRecConfig.mtv)
        {
            Transform::m_minimumVotes = m_dynRecConfig.mtv;
        }
    }

    if (m_dynRecConfig.sft)
    {
        SharpBox<cVertex, cNormal>::m_theta_sharp = m_dynRecConfig.sft;
    }

    if (m_dynRecConfig.sct)
    {
        SharpBox<cVertex, cNormal>::m_phi_corner = m_dynRecConfig.sct;
    }
}
} 
// Let the magic happen...

/*
void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
   ROS_INFO("Reconfigure Request: %d %f %s %s %d",
             config.int_param, config.double_param,
             config.str_param.c_str(),
             config.bool_param?"True":"False",
             config.size);
}
*/

int main(int argc, char** args)
{
    ros::init(argc, args, "lvr_reconstruction_node");
    ros::NodeHandle nh;

    /*
    dynamic_reconfigure::Server<lvr_ros::lvrConfig>::CallbackType m_dynRecCallback;
    dynamic_reconfigure::Server<lvr_ros::lvrConfig> m_dynRecServer;

    m_dynRecCallback = boost::bind(&LvrReconstructionNode::cfgcallback, _1, _2);
    m_dynRecServer.setCallback(m_dynRecCallback);
*/

    lvr_reconstruction_node::LvrReconstructionNode node(nh);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
