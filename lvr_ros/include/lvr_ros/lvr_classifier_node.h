#ifndef LVR_CLASSIFIER_NODE_H_
#define LVR_CLASSIFIER_NODE_H_

#include "ros/ros.h"
#include "ros/console.h"

#include "io/MeshBuffer.hpp"

#include "lvr_ros/Face.h"
#include "lvr_ros/Textures.h"
#include "lvr_ros/TriangleMeshGeometry.h"
#include "lvr_ros/Mesh.h"

#include <geometry_msgs/Point32.h>

#include "lvr_ros/DetectRooms.h"

#include "lvr_ros_converter.h"

using namespace std;
using namespace lvr;

namespace lvr_classifier_node
{

	class LvrClassifierNode
	{

		private:

			ros::NodeHandle n;
			ros::ServiceServer detect_rooms_srv;

			lvr_ros::Mesh mesh;

			lvr_ros_converter::LvrRosConverter converter;

		public:

			LvrClassifierNode(ros::NodeHandle n);

			~LvrClassifierNode()
			{};

			bool detectRoomsSrvCallback(lvr_ros::DetectRooms::Request& request, lvr_ros::DetectRooms::Response& response);

	};

}

#endif /* LVR_CLASSIFIER_NODE_H_ */
