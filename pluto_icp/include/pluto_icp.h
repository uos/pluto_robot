#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/filters/filter.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pluto_icp/IcpSrv.h>

#include <stack>

class PlutoICP{

  public:
    PlutoICP(ros::NodeHandle &nh);
    
    void sendMapToOdomCombined();
  private:
    ros::ServiceServer service;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;

    tf::TransformBroadcaster tf_broadcaster;

    void icpUpdateMapToOdomCombined(const sensor_msgs::PointCloud2::ConstPtr &cloud);

    std::stack<tf::StampedTransform>map_to_odom;
    tf::StampedTransform last_sent;

    std::stack<sensor_msgs::PointCloud2> clouds;
    std::stack<geometry_msgs::PoseStamped> poses;
    
    bool registerCloudsSrv( pluto_icp::IcpSrv::Request &req,
                            pluto_icp::IcpSrv::Response &res);
  
    bool registerClouds(
      const sensor_msgs::PointCloud2 &target,
      const geometry_msgs::PoseStamped &target_pose,
      const sensor_msgs::PointCloud2 &cloud,
      const geometry_msgs::PoseStamped &cloud_pose,
      geometry_msgs::PoseStamped &result_pose,
      geometry_msgs::Transform &delta_transform);

    void tfToEigen( const tf::Transform &transform_tf,
                    Eigen::Matrix4f &transform_eigen);
    
    void eigenToTf( const Eigen::Matrix4f &transform_eigen,
                    tf::Transform &transform_tf);

    bool getPointCloudPose(
      const sensor_msgs::PointCloud2 &cloud,
      const std::string &fixed_frame,
      geometry_msgs::PoseStamped &pose);

};
