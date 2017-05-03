#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <sstream>

using namespace sensor_msgs;
using namespace message_filters;
using namespace tf;

ros::Publisher pub;
int vector_one_index;
int vector_two_index;
std::string joint_state_name;

void callback(const Imu::ConstPtr &msg_one, const Imu::ConstPtr &msg_two) {
    ROS_DEBUG("IMU one Seq number: [%d]", msg_one->header.seq);
    ROS_DEBUG("IMU one Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg_one->orientation.x, msg_one->orientation.y,
              msg_one->orientation.z, msg_one->orientation.w);

    ROS_DEBUG("IMU two Seq number: [%d]", msg_two->header.seq);
    ROS_DEBUG("IMU two Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg_two->orientation.x, msg_two->orientation.y,
              msg_two->orientation.z, msg_two->orientation.w);

    // convert imu one orientation to vector
    Quaternion q1(msg_one->orientation.x, msg_one->orientation.y, msg_one->orientation.z, msg_one->orientation.w);
    Matrix3x3 m1(q1);
    Vector3 v1 = m1.getColumn(vector_one_index);

    // convert imu two orientation to vector
    Quaternion q2(msg_two->orientation.x, msg_two->orientation.y, msg_two->orientation.z, msg_two->orientation.w);
    Matrix3x3 m2(q2);
    Vector3 v2 = m2.getColumn(vector_two_index);

    ROS_DEBUG("x1 -> x: [%f], y: [%f], z: [%f]", v1.x(), v1.y(), v1.z());
    ROS_DEBUG("x2 -> x: [%f], y: [%f], z: [%f]", v2.x(), v2.y(), v2.z());
    ROS_DEBUG("Angle between v1 and v2: [%f]", v1.angle(v2));

    // publish joint state with angle between imus
    JointState msg;
    msg.name.push_back(joint_state_name);
    msg.position.push_back(v1.angle(v2));
    pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lever");
    ros::NodeHandle n_private("~");
    ros::NodeHandle n;
    pub = n.advertise<JointState>("lever", 10000);

    // get params
    n_private.param("vector_one_column", vector_one_index, 0);
    n_private.param("vector_two_column", vector_two_index, 0);
    if (!n_private.getParam("joint_state_name", joint_state_name)) {
        ROS_FATAL("Parameter \"joint_state_name\" missing!");
        ros::shutdown(); // shutdown without joint state name
    }

    // create subscriber
    Subscriber <Imu> imu_one(n, "imu/one", 1000);
    Subscriber <Imu> imu_two(n, "imu/two", 1000);

    // sync imus
    typedef sync_policies::ApproximateTime <Imu, Imu> SyncPolicy;
    Synchronizer <SyncPolicy> sync(SyncPolicy(1000), imu_one, imu_two);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
