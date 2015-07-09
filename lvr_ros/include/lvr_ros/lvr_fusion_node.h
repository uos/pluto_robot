#ifndef LVR_FUSION_NODE_H_
#define LVR_FUSION_NODE_H_

#include <ros/ros.h>
#include <ros/console.h>

#include "io/PLYIO.hpp"
#include "io/Timestamp.hpp"
#include "io/Progress.hpp"
#include "io/DataStruct.hpp"
#include "io/Model.hpp"
#include "io/ModelFactory.hpp"
#include "io/MeshBuffer.hpp"

#include "geometry/ColorVertex.hpp"
#include "geometry/Fusion.hpp"

#include <lvr_ros/Face.h>
#include <lvr_ros/Textures.h>
#include <lvr_ros/TriangleMeshGeometry.h>

#include "lvr_ros/AddToFusionMesh.h"
#include "lvr_ros/ResetFusionMesh.h"
#include "lvr_ros/SaveFusionMesh.h"
#include "lvr_ros/GetFusionMesh.h"
#include "lvr_ros_converter.h"

using namespace std;
using namespace lvr;

typedef ColorVertex<float, unsigned char> fVertex;
typedef Normal<float> fNormal;

namespace lvr_fusion_node
{

/**
 * @brief  MeshFusionNode
 */

class LvrFusionNode
{

private:

	ros::NodeHandle nh;
	
	bool verbose_lvr;
	bool use_callback;
	bool init;

	double distance_threshold;

	string input_mesh_topic;
	string output_mesh_topic;
	string frame;
	string type;

	ros::Subscriber input_mesh_sub;
	ros::Publisher output_mesh_pub;
	ros::ServiceServer get_fusion_mesh_srv;
	ros::ServiceServer save_fusion_mesh_srv;
	ros::ServiceServer reset_fusion_mesh_srv;
	ros::ServiceServer add_to_fusion_mesh_srv;

	lvr_ros_converter::LvrRosConverter converter;
	Fusion<fVertex, fNormal>* fusion;

	/**
	 * TODO comments
	 */
	bool addToFusionMesh(lvr_ros::TriangleMeshGeometry mesh);

	/**
	 * TODO comments
	 */
	bool finalizeFusionMesh(lvr_ros::TriangleMeshGeometry *mesh);

public:

	LvrFusionNode(ros::NodeHandle h);

	~LvrFusionNode()
	{
	}
	;

	/**
	 * TODO comments
	 */
	bool srvGetFusionMesh(lvr_ros::GetFusionMesh::Request& request,
	        lvr_ros::GetFusionMesh::Response& response);

	/**
	 * TODO comments
	 */
	bool srvSaveFusionMesh(lvr_ros::SaveFusionMesh::Request& request,
	        lvr_ros::SaveFusionMesh::Response& response);

	/**
	 * TODO comments
	 */
	bool srvResetFusionMesh(lvr_ros::ResetFusionMesh::Request& request,
	        lvr_ros::ResetFusionMesh::Response& response);

	/**
	 * TODO comments
	 */
	bool srvAddToFusionMesh(lvr_ros::AddToFusionMesh::Request& request,
	        lvr_ros::AddToFusionMesh::Response& response);

	/**
	 * TODO comments
	 */
	void mesh_callback(lvr_ros::Mesh mesh);

};
}

#endif /* LVR_FUSION_NODE_H_ */
