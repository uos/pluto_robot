/*
 * lvr_reconstruction_node.h
 *
 *  Created on: 30.04.2014
 */

#ifndef LVR_RECONSTRUCTION_NODE_H_
#define LVR_RECONSTRUCTION_NODE_H_

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <vector>
#include <algorithm>  
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <pcl-1.7/pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/ColorRGBA.h>
#include <lvr_ros/Face.h>
#include <lvr_ros/MaterialGroup.h>
#include <lvr_ros/Textures.h>
#include <lvr_ros/LabeledFaces.h>
#include <lvr_ros/TriangleMeshGeometry.h>
#include <lvr_ros/Mesh.h>
#include <lvr_ros/PolygonMesh.h>
#include <lvr_ros/DetectRooms.h>
#include <lvr_ros/CreateMesh.h>
#include <lvr_ros/CreatePolygonMesh.h>
#include <lvr_ros/PolygonFusion.h>

#include <dynamic_reconfigure/server.h>
#include <lvr_ros/lvrConfig.h>

#include "io/Timestamp.hpp"
#include "io/Model.hpp"
#include "io/PointBuffer.hpp"
#include "io/MeshBuffer.hpp"
#include "texture/Statistics.hpp"
#include "texture/Texture.hpp"
#include "texture/Transform.hpp"
#include "texture/Texturizer.hpp"
#include "reconstruction/AdaptiveKSearchSurface.hpp"
#include "reconstruction/FastReconstruction.hpp"
#include "reconstruction/SharpBox.hpp"
#include "geometry/HalfEdgeFace.hpp"
#include "geometry/HalfEdgeMesh.hpp"
#include "geometry/Matrix4.hpp"
#include "geometry/QuadricVertexCosts.hpp"
#include "geometry/PolygonFusion.hpp"

#ifdef _USE_PCL_
#include "reconstruction/PCLKSurface.hpp"
#include <pcl/features/integral_image_normal.h>
#include <pcl_conversions/pcl_conversions.h>
#endif

#include "lvr_ros_converter.h"

using namespace lvr;

namespace lvr_reconstruction_node
{

template<typename VertexT, typename NormalT>
class HalfEdgeFace;

typedef ColorVertex<float, unsigned char>        cVertex;
typedef Normal<float>                            cNormal;
typedef PointsetSurface<cVertex>                 psSurface;
typedef AdaptiveKSearchSurface<cVertex, cNormal> akSurface;
typedef vector<Region<cVertex, cNormal>* > RegionVector;
typedef vector<cVertex> VertexVector;
typedef vector<VertexVector> VertexVectorGroup;

typedef PolygonFusion<cVertex, cNormal> PolyFusion;
typedef PolygonMesh<cVertex, cNormal>   PolyMesh;
typedef PolygonRegion<cVertex, cNormal> PolyRegion;
typedef Polygon<cVertex, cNormal>       Poly;

#ifdef _USE_PCL_
typedef PCLKSurface<cVertex, cNormal>            pclSurface;
#endif

/**
 * @brief Class to reconstruct triangle mesh.
 */

class LvrReconstructionNode
{

public:

    /**
     * @brief Constructor
     *
     * @param nh  Ros-NodeHandler
     */
    LvrReconstructionNode(ros::NodeHandle nh);

    /**
     * @brief Destructor
     */
    ~LvrReconstructionNode(){};

    /**
     * @brief Callback for incoming pointCloud
     *
     * @param pc  Incoming PointCloud2
     */
    void callback(sensor_msgs::PointCloud2 pc);

    /**
     * @brief Callback for incoming dynamic_reconfigure_changes
     *
     * @param nConf  dynamic_reconfigure_configs
     * @parem level  
     */
    void cfgcallback(lvr_ros::lvrConfig &nConf, uint32_t level);

    /**
     * @brief Service for creating Mesh
     *
     * @param request  Service request
     * @parem response Service response
     *
     * @return true, if succeded 
     */
    bool srvCreateMesh(lvr_ros::CreateMesh::Request& request, lvr_ros::CreateMesh::Response& response);

    /**
     * @brief Service for creating Polygonmesh
     *
     * @param request  Service request
     * @param response Service response
     *
     * @return true, if succeded 
     */
    bool srvCreatePolygonMesh(lvr_ros::CreatePolygonMesh::Request& request, lvr_ros::CreatePolygonMesh::Response& response);

    /**
     * @brief Service for a PolygonFusion
     *
     * @param request Service request
     * @param response Service response
     *
     * @return true, if succeded 
     */
    bool srvDoFusion(lvr_ros::PolygonFusion::Request& request,  lvr_ros::PolygonFusion::Response& response);

private:

/**
     * @brief Calculates normals of a given PointCloud for creating Polygonmesh
     *
     * @param pc         Given pointCloud
     * @parem pLoader    Given pointBufferPtr
     * @parem surface    Given surfacePointer
     * @parem numPoints  Number of Points
     */
    void calculateNormals(const sensor_msgs::PointCloud2::ConstPtr& pc,
                          PointBufferPtr pLoader,
                          psSurface::Ptr surface,
                          int numPoints);
    /**
     * @brief Optimizes and finalizes a given mesh
     *
     * @param mesh       Given mesh
     */
    void optimizeAndFinalize(HalfEdgeMesh<cVertex, cNormal>& mesh);

    /**
     * @brief Setup the reconstruction Parameters
     */
    void setupReconstructionParameters();

    /**
     * @brief Generates from a given pointCloud a triangle mesh and a message with normals
     *
     * @param inputCloud     Given pointCloud
     * @param outputMesh     Output-Message with mesh
     * @param outputNormals  Output-Message with normals
     */
    bool createMesh(sensor_msgs::PointCloud2& inputCloud,
                    lvr_ros::Mesh *outputMesh,
                    lvr_ros::PointNormals *outputNormals);

    /**
     * @brief This Method processes a Pointcloud to a PolygonMesh
     *
     * @param inputCloud The Pointcloud, which will be processed to a PolygonMesh
     * @param polymesh The Output-Mesh
     *
     * @return false, if some bad bad stuff happend
     */
    bool createPolygonMesh(sensor_msgs::PointCloud2& inputCloud, lvr_ros::PolygonMesh *polymesh);

/**
 * @brief Transforms a lvr PolygonMesh into a ros PolygonMesh Message
 *
 * @param input the lvr PolygonMesh
 * @param output the resulting ros PolygonMesh Message
 */
void FusiontoROS(PolyMesh input, lvr_ros::PolygonMesh &output);

/**
 * @brief Transforms a ros PolygonMesh Message into a lvr PolygonMesh
 *
 * @param input the ros PolygonMesh Message
 * @param output the resulting lvr PolygonMesh
 */
void ROStoFusion(lvr_ros::PolygonMesh input, PolyMesh &output);

ros::NodeHandle m_nodeHandler;
ros::Publisher m_meshPublisher;
ros::Publisher m_pointNormalsPublisher;
ros::Subscriber m_pointCloudSubscriber;

//debug
ros::Publisher polygon_mesh_pub_;
ros::Publisher polygon_mesh_pub2_;
// end debug
PolyFusion m_Poly_Fusion;

lvr_ros_converter::LvrRosConverter m_lvrRosConverter;

//Service-Server
ros::ServiceServer m_createMeshService;
ros::ServiceServer m_createPolygonMeshService;
ros::ServiceServer m_doPolygonFusionService;

//Parameters manipulated by dynamic_reconfigure
lvr_ros::lvrConfig m_dynRecConfig;

dynamic_reconfigure::Server<lvr_ros::lvrConfig> m_dynRecServer;
dynamic_reconfigure::Server<lvr_ros::lvrConfig>::CallbackType m_dynRecCallback;

};
}

#endif /* LVR_RECONSTRUCTION_NODE_H_ */
