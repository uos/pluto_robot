#include <map_odom_icp.h>

MapOdomICP::MapOdomICP(ros::NodeHandle &nh)
 :  nh_(nh),
    icp_(nh)
{
  tf::Transform null_transform;
  null_transform.setIdentity();
  map_to_odom_.push(tf::StampedTransform(null_transform, ros::Time::now(), "map", "odom_combined"));
  cloud_sub_ = nh_.subscribe("assembled_cloud", 20, &MapOdomICP::icpUpdateMapToOdomCombined, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("registered_cloud", 1);
}


bool MapOdomICP::getPointCloudPose( const sensor_msgs::PointCloud2 &cloud,
                                    const std::string &fixed_frame,
                                    geometry_msgs::PoseStamped &pose){

  std::string error_msg;
  bool success = tf_listener_.waitForTransform(fixed_frame, cloud.header.frame_id, cloud.header.stamp,
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
    tf_listener_.lookupTransform(fixed_frame, cloud.header.frame_id,  cloud.header.stamp, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  tf::Stamped<tf::Transform> transform_pose(transform, transform.stamp_, transform.frame_id_);
  tf::poseStampedTFToMsg(transform_pose, pose);

  return true;
}

void MapOdomICP::icpUpdateMapToOdomCombined(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  bool succeeded = true;
  ROS_INFO("ros_icp received new point cloud");
  
  geometry_msgs::PoseStamped cloud_pose;
  getPointCloudPose(*cloud, "map", cloud_pose);
  
  if(!clouds_.empty()){

    geometry_msgs::PoseStamped target_pose = poses_.top();
    geometry_msgs::PoseStamped result_pose;
    geometry_msgs::Transform delta_transform;

    // register point clouds with icp
    ROS_INFO("register point clouds...");
    succeeded = icp_.registerClouds(
      clouds_.top(),
      target_pose,
      *cloud,
      cloud_pose,
      result_pose,
      delta_transform
    );

    if(succeeded){
      ROS_INFO("... register point clouds done.");
      

      // convert to tf
      tf::Transform delta_transform_tf;
      tf::transformMsgToTF(delta_transform, delta_transform_tf);

      // calculate the new map to odom_combined transformation
      tf::Transform update_transform_tf = map_to_odom_.top() * delta_transform_tf;
      tf::StampedTransform update_transform_stamped_tf(
        update_transform_tf, cloud->header.stamp, "map", "odom_combined");
      map_to_odom_.push(update_transform_stamped_tf);
      
      // update the cloud pose
      cloud_pose = result_pose;
    }
    else{
      ROS_INFO("register point clouds faild.");
    }
  }

  if(succeeded){
    // save cloud and pose for the next iteration
    poses_.push(cloud_pose);
    clouds_.push(*cloud);
    Sync::setUpdated();
  }
}

bool MapOdomICP::Sync::updated_ = false;
boost::mutex MapOdomICP::Sync::mtx_;

void MapOdomICP::Sync::setUpdated(){
  mtx_.lock();
  updated_ = true;
  mtx_.unlock();
}

bool MapOdomICP::Sync::hasUpdated(){
  bool ret = false;
  mtx_.lock();
  if(updated_){
    updated_ = false;
    ret = true;
  }
  mtx_.unlock();
  return ret;
}

void MapOdomICP::sendMapToOdomCombined(){

  ros::Time now = ros::Time::now();

  tf_broadcaster_.sendTransform(
    tf::StampedTransform(
      map_to_odom_.top(),
      now,
      "map",
      "odom_combined"
    )
  );


  if (Sync::hasUpdated()){
    tf::Transform transform = map_to_odom_.top();
    tf::Vector3 origin = transform.getOrigin();
    tf::Matrix3x3 basis = transform.getBasis();
    tfScalar roll, pitch, yaw;
    basis.getRPY(roll, pitch, yaw);

    ROS_INFO("Updated Transform from map to odom: The new transformation is \n (x:%f y:%f z:%f)[meter] (roll:%f pitch:%f yaw:%f)[degree]",
      origin.getX(),
      origin.getY(),
      origin.getY(),
      roll * 180 / M_PI,
      pitch * 180 / M_PI,
      yaw * 180 / M_PI);

    if(!clouds_.empty()){
      sensor_msgs::PointCloud2 cloud = clouds_.top();
      cloud.header.stamp = now;
      cloud_pub_.publish(cloud);
    }
  }
}

int main(int args, char** argv){
  ros::init(args, argv, "pluto_icp");
  ros::NodeHandle nh;

  MapOdomICP pluto_icp(nh);
  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  ros::Rate rate(10.0);
  while(nh.ok()){
    pluto_icp.sendMapToOdomCombined();
    rate.sleep();
  }
}
