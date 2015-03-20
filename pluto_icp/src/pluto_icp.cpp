#include <pluto_icp.h>

PlutoICP::PlutoICP(ros::NodeHandle &nh)
 :  nh_(nh),
    tf_listener(ros::Duration(60.0))
{
  tf::Transform null_transform;
  map_to_odom.push(tf::StampedTransform(null_transform, ros::Time::now(), "map", "odom_combined"));
  service = nh_.advertiseService("pluto_icp", &PlutoICP::registerCloudsSrv, this); 
}


bool PlutoICP::registerCloudsSrv( pluto_icp::IcpSrv::Request &req,
                                  pluto_icp::IcpSrv::Response &res){

  return registerClouds(req.target,
                        req.target_pose,
                        req.cloud,
                        req.cloud_pose,
                        res.result_pose,
                        res.delta_transform);
}

bool PlutoICP::registerClouds(
  const sensor_msgs::PointCloud2 &target,
  const geometry_msgs::PoseStamped &target_pose,
  const sensor_msgs::PointCloud2 &cloud,
  const geometry_msgs::PoseStamped &cloud_pose,
  geometry_msgs::PoseStamped &result_pose,
  geometry_msgs::Transform &delta_transform)
{

  // calculate transformation between target_pose and cloud_pose
  
  tf::Stamped<tf::Pose> target_pose_tf;
  tf::Stamped<tf::Pose> cloud_pose_tf;

  tf::poseStampedMsgToTF(target_pose, target_pose_tf);
  tf::poseStampedMsgToTF(cloud_pose, cloud_pose_tf);

  tf::Transform guess_transform_tf = target_pose_tf.inverseTimes(cloud_pose_tf);
  Eigen::Matrix4f guess_transform;
  tfToEigen(guess_transform_tf, guess_transform);

  // poseToEigen(target_pose.pose, cloud_pose.pose, guess_transform);
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
  if(!registerClouds(target_xyz_ptr, cloud_xyz_ptr, guess_transform, final_transform)){
    return false;
  }

  tf::Transform final_transform_tf;
  eigenToTf(final_transform, final_transform_tf);

  // combine the target_pose and the icp transformation 
  // between the clouds to get the global pose
  tf::Transform result_pose_tf = target_pose_tf * final_transform_tf;
  tf::Stamped<tf::Pose> result_pose_stamped_tf(result_pose_tf, ros::Time::now(), cloud_pose.header.frame_id);
  // convert to pose msg
  tf::poseStampedTFToMsg(result_pose_stamped_tf, result_pose);
  
  tf::Transform delta_transform_tf = guess_transform_tf.inverseTimes(final_transform_tf);
  tf::transformTFToMsg(delta_transform_tf, delta_transform);

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
void PlutoICP::eigenToTf(
    const Eigen::Matrix4f &transform_eigen,
    tf::Transform &transform_tf){
  
  //create an affine transformation as helper object
  Eigen::Affine3f affine(transform_eigen);
  Eigen::Quaternionf q = (Eigen::Quaternionf)affine.linear();
  
  //create tf transform object
  tf::Vector3 origin;
  origin.setX(affine.translation()[0]);
  origin.setY(affine.translation()[1]);
  origin.setZ(affine.translation()[2]);
  tf::Quaternion rotation;
  if(q.w() > 0){
    rotation.setX(q.x());
    rotation.setY(q.y());
    rotation.setZ(q.z());
    rotation.setW(q.w());
  }else{
    rotation.setX(-q.x());
    rotation.setY(-q.y());
    rotation.setZ(-q.z());
    rotation.setW(-q.w()); 
  }
  transform_tf.setOrigin(origin);
  transform_tf.setRotation(rotation);
}

void PlutoICP::tfToEigen(
  const tf::Transform &transform_tf,
  Eigen::Matrix4f &transform_eigen)
{
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
  transform_eigen = affine.matrix(); // TODO check 
}



void PlutoICP::icpUpdateMapToOdomCombined(sensor_msgs::PointCloud2 &cloud){
  if(!clouds.empty()){

    geometry_msgs::PoseStamped target_pose;
    getPointCloudPose(clouds.top(), "map", target_pose);

    geometry_msgs::PoseStamped cloud_pose;
    getPointCloudPose(cloud, "map", cloud_pose);

    geometry_msgs::PoseStamped result_pose;
    geometry_msgs::Transform delta_transform;

    registerClouds(clouds.top(),
                   target_pose,
                   cloud,
                   cloud_pose,
                   result_pose,
                   delta_transform);

    tf::Transform delta_transform_tf;
    tf::transformMsgToTF(delta_transform, delta_transform_tf);
    tf::Transform update_transform_tf = map_to_odom.top() * delta_transform_tf;
    tf::StampedTransform(update_transform_tf, cloud.header.stamp, "map", "odom_combined");
  }
  clouds.push(cloud);
}

void PlutoICP::sendMapToOdomCombined(){
  tf_broadcaster.sendTransform(tf::StampedTransform(map_to_odom.top(), ros::Time::now(), "map", "odom_combined"));
  if (last_sent.stamp_ != map_to_odom.top().stamp_){
    ROS_INFO("updated transform for map to odom_combined.");
  }
  last_sent = map_to_odom.top();
}


bool PlutoICP::registerClouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    Eigen::Matrix4f &guess_transform,
    Eigen::Matrix4f &final_transform){

  icp.setInputSource(cloud);
  icp.setInputTarget(target);
  
  pcl::PointCloud<pcl::PointXYZ> result;
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

  ros::Rate rate(10.0);
  while(nh.ok()){
    pluto_icp.sendMapToOdomCombined();
    rate.sleep();
  }
}
