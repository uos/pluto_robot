#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ros_pcl_icp.h>
#include <stack>
#include <boost/thread/mutex.hpp>

class MapOdomICP{

  public:
    MapOdomICP(ros::NodeHandle &nh);
    void sendMapToOdomCombined();
  private:
    RosPclIcp icp_;

    class Sync{
      public:
        static void setUpdated();
        static bool hasUpdated();
      private:
        static boost::mutex mtx_;
        static bool updated_;
    };

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::stack<tf::StampedTransform>map_to_odom_;
    std::stack<sensor_msgs::PointCloud2> clouds_;
    std::stack<geometry_msgs::PoseStamped> poses_;

    void icpUpdateMapToOdomCombined(const sensor_msgs::PointCloud2::ConstPtr &cloud);

    bool getPointCloudPose(
      const sensor_msgs::PointCloud2 &cloud,
      const std::string &fixed_frame,
      geometry_msgs::PoseStamped &pose);

};
