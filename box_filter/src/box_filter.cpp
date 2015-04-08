#include <box_filter/box_filter.h>
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(laser_filterss, PointCloudBoxFilter, laser_filters::PointCloudBoxFilter, filters::FilterBase<sensor_msgs::PointCloud2>)

laser_filters::PointCloudBoxFilter::PointCloudBoxFilter(){

}

bool laser_filters::PointCloudBoxFilter::configure(){
  bool success = true;
  if(getParam("boxFrame", box_frame_)){
    success = false;
  }

  return success;

}

bool laser_filters::PointCloudBoxFilter::update(
  const sensor_msgs::PointCloud2& input_cloud,
  sensor_msgs::PointCloud2 &output_cloud)
{
/*
  std::string error_msg;
  bool success = tf_listener_.waitForTransform(box_frame,
  input_cloud.header.frame_id, input_cloud.header.stamp,
      ros::Duration(3.0), ros::Duration(0.01), &error_msg);
  if (!success)
  {
    ROS_WARN("Could not get transform! %s", error_msg.c_str());
    return false;
  }

  tf::StampedTransform transform;
  ROS_INFO("Lookup transform from %s to %s at %f sec",  
    fixed_frame.c_str(), cloud.header.frame_id.c_str(), cloud.header.stamp.toSec());
  try
  {
    tf_listener_.lookupTransform(
      box_frame,
      input_cloud.header.frame_id,
      input_cloud.header.stamp,
      transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
*/
  return true;
}
