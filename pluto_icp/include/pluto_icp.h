#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pluto_icp/IcpSrv.h>

class PlutoICP{

  public:
    PlutoICP(ros::NodeHandle &nh);
    
  private:
    ros::ServiceServer service;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float> icp;
    ros::NodeHandle nh_;

    bool registerCloudsSrv( pluto_icp::IcpSrv::Request &req,
                            pluto_icp::IcpSrv::Response &res);
  
    bool registerClouds(
      const sensor_msgs::PointCloud2 target,
      const geometry_msgs::PoseStamped target_pose,
      const sensor_msgs::PointCloud2 cloud,
      const geometry_msgs::PoseStamped cloud_pose,
      sensor_msgs::PointCloud2 &result,
      geometry_msgs::PoseStamped &result_pose);

    void poseToEigen(const geometry_msgs::Pose target_pose,
                     const geometry_msgs::Pose cloud_pose,
                     Eigen::Matrix4f &transformation);
    
    void eigenToPose(Eigen::Matrix4f &transformation,
                     const geometry_msgs::Pose target_pose,
                     geometry_msgs::Pose &pose);

    bool registerClouds(
      pcl::PointCloud<pcl::PointXYZ>::Ptr &target, 
      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &guess_transform,
      pcl::PointCloud<pcl::PointXYZ> &result,
      pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &final_transform);

};
