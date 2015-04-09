#ifndef BOXFILTER_H
#define BOXFILTER_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "laser_geometry/laser_geometry.h"

// TF
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//Filters
#include "filters/filter_base.h"

namespace laser_filters
{

class LaserScanBoxFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    LaserScanBoxFilter();
    bool configure();

    bool update(
      const sensor_msgs::LaserScan& input_scan,
      sensor_msgs::LaserScan& filtered_scan);

  private:
    bool inBox(tf::Point &point);
    std::string box_frame_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_;
    tf::Point min_, max_;
    bool up_and_running_;
};

}


#endif /* box_filter.h */
