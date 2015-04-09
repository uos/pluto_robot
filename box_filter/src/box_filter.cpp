#include <box_filter/box_filter.h>
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(laser_filters, LaserScanBoxFilter, laser_filters::LaserScanBoxFilter, filters::FilterBase<sensor_msgs::LaserScan>)

laser_filters::LaserScanBoxFilter::LaserScanBoxFilter(){

}

bool laser_filters::LaserScanBoxFilter::configure(){
  up_and_running_ = true;
  double min_x, min_y, min_z, max_x, max_y, max_z;
  bool box_frame_set = getParam("box_frame", box_frame_);
  bool x_max_set = getParam("max_x", max_x);
  bool y_max_set = getParam("max_y", max_y);
  bool z_max_set = getParam("max_z", max_z);
  bool x_min_set = getParam("min_x", min_x);
  bool y_min_set = getParam("min_y", min_y);
  bool z_min_set = getParam("min_z", min_z);
  
  max_.setX(max_x);
  max_.setY(max_y);
  max_.setZ(max_z);
  min_.setX(min_x);
  min_.setY(min_y);
  min_.setZ(min_z);
  
  if(!box_frame_set){
    ROS_ERROR("box_frame is not set!");
  }
  if(!x_max_set){
    ROS_ERROR("max_x is not set!");
  }
  if(!y_max_set){
    ROS_ERROR("max_y is not set!");
  }
  if(!z_max_set){
    ROS_ERROR("max_z is not set!");
  }
  if(!x_min_set){
    ROS_ERROR("min_x is not set!");
  }
  if(!y_min_set){
    ROS_ERROR("min_y is not set!");
  }
  if(!z_min_set){
    ROS_ERROR("min_z is not set!");
  }

  return box_frame_set && x_max_set && y_max_set && z_max_set &&
    x_min_set && y_min_set && z_min_set;

}

bool laser_filters::LaserScanBoxFilter::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  output_scan = input_scan;
  sensor_msgs::PointCloud2 laser_cloud;
  
  std::string error_msg;

  bool success = tf_.waitForTransform(
    box_frame_,
    input_scan.header.frame_id,
    input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size()*input_scan.time_increment),
    ros::Duration(1.0),
    ros::Duration(0.01),
    &error_msg
  );
  if(!success){
    ROS_WARN("Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
    return false;
  }

  try{
    projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, tf_);
  }
  catch(tf::TransformException& ex){
    if(up_and_running_){
      ROS_WARN_THROTTLE(1, "Dropping Scan: Tansform unavailable %s", ex.what());
      return true;
    }
    else
    {
      ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");
    }
    return false;
  }
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
  const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
  const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
  const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

  const int pstep = laser_cloud.point_step;
  const long int pcount = laser_cloud.width * laser_cloud.height;
  const long int limit = pstep * pcount;

  int i_idx, x_idx, y_idx, z_idx;  
  for(
    i_idx = i_idx_offset,
    x_idx = x_idx_offset,
    y_idx = y_idx_offset,
    z_idx = z_idx_offset;

    x_idx < limit;

    i_idx += pstep,
    x_idx += pstep,
    y_idx += pstep,
    z_idx += pstep)
  {
    float x = *((float*)(&laser_cloud.data[x_idx]));
    float y = *((float*)(&laser_cloud.data[y_idx]));
    float z = *((float*)(&laser_cloud.data[z_idx]));
    int index = *((int*)(&laser_cloud.data[i_idx]));

    tf::Point point(x, y, z);

    if(inBox(point)){
      output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  up_and_running_ = true;
  return true;
}

bool laser_filters::LaserScanBoxFilter::inBox(tf::Point &point){
  return point.x() < max_.x() && point.x() > min_.x() && 
    point.y() < max_.y() && point.y() > min_.y() &&
    point.z() < max_.z() && point.z() > min_.z();
}

