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
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pluto_icp/IcpSrv.h>

class RosPclIcp{

  public:
    RosPclIcp(ros::NodeHandle &nh);
    
    bool registerClouds(
      const sensor_msgs::PointCloud2 &target,
      const geometry_msgs::PoseStamped &target_pose,
      const sensor_msgs::PointCloud2 &cloud,
      const geometry_msgs::PoseStamped &cloud_pose,
      geometry_msgs::PoseStamped &result_pose,
      geometry_msgs::Transform &delta_transform);

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer service;

    bool registerCloudsSrv( pluto_icp::IcpSrv::Request &req,
                            pluto_icp::IcpSrv::Response &res);
  

    void tfToEigen( const tf::Transform &transform_tf,
                    Eigen::Matrix4f &transform_eigen);
    
    void eigenToTf( const Eigen::Matrix4f &transform_eigen,
                    tf::Transform &transform_tf);

};
