#ifndef BOXFILTER_H
#define BOXFILTER_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// TF
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//Filters
#include "filters/filter_base.h"

namespace laser_filters
{

class PointCloudBoxFilter : public filters::FilterBase<sensor_msgs::PointCloud2>
{
  public:
    PointCloudBoxFilter();
    bool configure();

    bool update(
      const sensor_msgs::PointCloud2& input_scan,
      sensor_msgs::PointCloud2& filtered_scan);

  private:

    std::string box_frame_;
    //ros::NodeHandle nh_;
    //ros::Subscriber cloud_sub_;
    //ros::Publisher cloud_pub_;
    
    //tf::TransformListener tf_listener_;


};

}


#endif /* box_filter.h */
