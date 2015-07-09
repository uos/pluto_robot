#ifndef MESHPUBLISHER_H_
#define MESHPUBLISHER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <lvr_ros_converter.h>
#include <lvr_ros/Face.h>
#include <lvr_ros/Mesh.h>
#include <lvr_ros/TriangleMeshGeometry.h>

using namespace std;
using namespace lvr;

namespace mesh_publisher_node
{

/**
 * @brief  MeshPublisherNode
 */

class MeshPublisherNode
{

private:

ros::NodeHandle nh;

bool saved;
double rate;
int n;
string to_file_path;
string frame;
string from_file_path;
string to_file_topic;
string from_file_topic;
ros::Publisher mesh_pub;
ros::Subscriber mesh_sub;

lvr_ros_converter::LvrRosConverter converter;

public:

MeshPublisherNode(ros::NodeHandle h);

~MeshPublisherNode() {};
void mesh_callback(const lvr_ros::TriangleMeshGeometry::ConstPtr& mesh);
void readAndPublish();

double getRate();
};

}

#endif /* MESHPUBLISHER_H_ */
