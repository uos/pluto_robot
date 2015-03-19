#include <pluto_icp.h>

PlutoICP::PlutoICP(ros::NodeHandle &nh)
 :  nh_(nh),
    tf_listener(ros::Duration(60.0))
{
  service = nh_.advertiseService("pluto_icp", &PlutoICP::registerCloudsSrv, this);    
}


bool PlutoICP::registerCloudsSrv( pluto_icp::IcpSrv::Request &req,
                                  pluto_icp::IcpSrv::Response &res){                                  
  return registerClouds(req.target, req.target_pose, req.cloud, req.target_pose, res.result, res.result_pose);
}

bool PlutoICP::registerClouds(
  const sensor_msgs::PointCloud2 target,
  const geometry_msgs::PoseStamped target_pose,
  const sensor_msgs::PointCloud2 cloud,
  const geometry_msgs::PoseStamped cloud_pose,
  sensor_msgs::PointCloud2 &result,
  geometry_msgs::PoseStamped &result_pose
  )
{

  // calculate transformation between target_pose and cloud_pose
  Eigen::Matrix4f guess_transform;
  poseToEigen(target_pose.pose, cloud_pose.pose, guess_transform);
  std::cout << guess_transform << std::endl;

  // convert point clouds ro pcl
  pcl::PCLPointCloud2 pcl_target, pcl_cloud;
  pcl_conversions::toPCL(target, pcl_target);
  pcl_conversions::toPCL(cloud, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ> target_xyz;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::fromPCLPointCloud2(pcl_target, target_xyz);
  pcl::fromPCLPointCloud2(pcl_cloud, cloud_xyz);
  
  // create shared pointers
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz_ptr(&target_xyz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(&cloud_xyz);

  // do icp
  Eigen::Matrix4f final_transform;
  pcl::PointCloud<pcl::PointXYZ> result_xyz;
  
  if(!registerClouds(target_xyz_ptr, cloud_xyz_ptr, guess_transform, result_xyz, final_transform)){
    return false;
  }
  pcl::PCLPointCloud2 pcl_result;
  pcl::toPCLPointCloud2(result_xyz, pcl_result);
  pcl_conversions::fromPCL(pcl_result, result);
  
  // convert transformation to pose
  geometry_msgs::PoseStamped pose;
  eigenToPose(final_transform, target_pose.pose, pose.pose);
  return true;
}

bool PlutoICP::getPointCloudPose( const sensor_msgs::PointCloud2 &cloud,
                                  const std::string &fixed_frame,
                                  geometry_msgs::PoseStamped &pose){

  std::string error_msg;
  bool success = tf_listener.waitForTransform(fixed_frame, cloud.header.frame_id, cloud.header.stamp,
      ros::Duration(3.0), ros::Duration(0.01), &error_msg);
  if (!success)
  {
    ROS_WARN("Could not get transform! %s", error_msg.c_str());
    return false;
  }

  tf::StampedTransform transform;
  try
  {
    ROS_INFO("Lookup transform from %s to %s at %f sec",  
        fixed_frame.c_str(), cloud.header.frame_id.c_str(), cloud.header.stamp.toSec());
    tf_listener.lookupTransform(fixed_frame, cloud.header.frame_id,  cloud.header.stamp, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  tf::Stamped<tf::Transform> transform_pose(transform, transform.stamp_, transform.frame_id_);
  tf::poseStampedTFToMsg(transform_pose, pose);

  ROS_INFO("Looked Up Transform from %s to  %s is \n %s - (%f %f %f) (%f %f %f %f)",
      fixed_frame.c_str(),
      cloud.header.frame_id.c_str(),
      pose.header.frame_id.c_str(),
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z,
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w);

  return true;
}
void PlutoICP::eigenToPose(
    Eigen::Matrix4f &transformation,
    const geometry_msgs::Pose target_pose,
    geometry_msgs::Pose &pose){
  
  //create an affine transformation as helper object
  Eigen::Affine3f affine(transformation);
  Eigen::Quaternionf q = (Eigen::Quaternionf)affine.linear();
  
  //create tf transform object
  tf::Vector3 position;
  position.setX(affine.translation()[0]);
  position.setY(affine.translation()[1]);
  position.setZ(affine.translation()[2]);
  tf::Quaternion orientation;
  if(q.w() > 0){
    orientation.setX(q.x());
    orientation.setY(q.y());
    orientation.setZ(q.z());
    orientation.setW(q.w());
  }else{
    orientation.setX(-q.x());
    orientation.setY(-q.y());
    orientation.setZ(-q.z());
    orientation.setW(-q.w()); 
  }
  tf::Transform pose_tf(orientation, position);

  // convert target pose to tf
  tf::Pose target_pose_tf;
  tf::poseMsgToTF(target_pose, target_pose_tf);

  // combine the target_pose and the icp transformation 
  // between the clouds to get the global pose
  tf::Transform result_pose = target_pose_tf * pose_tf;
  // convert to pose msg
  tf::poseTFToMsg(result_pose, pose);
}

void PlutoICP::poseToEigen(
  const geometry_msgs::Pose target_pose,
  const geometry_msgs::Pose cloud_pose,
  Eigen::Matrix4f &transformation)
{
  tf::Pose target_pose_tf;
  tf::Pose cloud_pose_tf;

  tf::poseMsgToTF(target_pose, target_pose_tf);
  tf::poseMsgToTF(cloud_pose, cloud_pose_tf);

  tf::Transform transform_tf = target_pose_tf.inverseTimes(cloud_pose_tf);

  tf::Vector3 translation = transform_tf.getOrigin();
  tf::Quaternion rotation = transform_tf.getRotation();

  Eigen::Affine3f affine =
    Eigen::Translation3f( translation.getX(),
                          translation.getY(),
                          translation.getZ()) *
    Eigen::Quaternionf( rotation.getW(),
                        rotation.getX(),
                        rotation.getY(),
                        rotation.getZ()); 
  transformation = affine.matrix();
}

bool PlutoICP::registerClouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    Eigen::Matrix4f &guess_transform,
    pcl::PointCloud<pcl::PointXYZ> &result,
    Eigen::Matrix4f &final_transform){

  icp.setInputSource(cloud);
  icp.setInputTarget(target);
  icp.align(result, guess_transform);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;

  std::cout << icp.getFinalTransformation() << std::endl;

  final_transform = icp.getFinalTransformation();
  return icp.hasConverged();
}

int main(int args, char** argv){
  ros::init(args, argv, "pluto_icp");
  ros::NodeHandle nh;

  PlutoICP pluto_icp(nh);

  while(ros::ok()){
    ros::spinOnce();
  }
}
