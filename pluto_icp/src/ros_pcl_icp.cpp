#include <ros_pcl_icp.h>

RosPclIcp::RosPclIcp(ros::NodeHandle &nh)
 :  nh_(nh)
{
  service = nh_.advertiseService("icp", &RosPclIcp::registerCloudsSrv, this);
}

bool RosPclIcp::registerCloudsSrv( pluto_icp::IcpSrv::Request &req,
                                  pluto_icp::IcpSrv::Response &res){

  return registerClouds(req.target,
                        req.target_pose,
                        req.cloud,
                        req.cloud_pose,
                        res.result_pose,
                        res.delta_transform);
}

bool RosPclIcp::registerClouds(
  const sensor_msgs::PointCloud2 &target,
  const geometry_msgs::PoseStamped &target_pose,
  const sensor_msgs::PointCloud2 &cloud,
  const geometry_msgs::PoseStamped &cloud_pose,
  geometry_msgs::PoseStamped &result_pose,
  geometry_msgs::Transform &delta_transform)
{

  // create shared pointers
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  
  
  tf::Stamped<tf::Pose> target_pose_tf;
  tf::Stamped<tf::Pose> cloud_pose_tf;

  tf::poseStampedMsgToTF(target_pose, target_pose_tf);
  tf::poseStampedMsgToTF(cloud_pose, cloud_pose_tf);

  // calculate transformation between target_pose and cloud_pose
  tf::Transform guess_transform_tf = target_pose_tf.inverseTimes(cloud_pose_tf);
  Eigen::Matrix4f guess_transform;
  tfToEigen(guess_transform_tf, guess_transform);

  //std::cout << guess_transform << std::endl;

  // convert point clouds ro pcl
  pcl::PCLPointCloud2 pcl_target, pcl_cloud;
  pcl_conversions::toPCL(target, pcl_target);
  pcl_conversions::toPCL(cloud, pcl_cloud);
  pcl::fromPCLPointCloud2(pcl_target, *target_xyz_ptr);
  pcl::fromPCLPointCloud2(pcl_cloud, *cloud_xyz_ptr);

  // remove nan values from point clouds
  std::vector<int> indices_target;
  std::vector<int> indices_cloud;
  pcl::removeNaNFromPointCloud(*target_xyz_ptr,*target_xyz_ptr, indices_target);
  pcl::removeNaNFromPointCloud(*cloud_xyz_ptr,*cloud_xyz_ptr, indices_cloud);

  // do icp
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float> icp;

  icp.setInputSource(cloud_xyz_ptr);
  icp.setInputTarget(target_xyz_ptr);
  
  pcl::PointCloud<pcl::PointXYZ> result;
  icp.align(result, guess_transform);

  ROS_INFO("ICP has converged: %s, score: %f", icp.hasConverged()? "true":
  "false", icp.getFitnessScore());

  if(! icp.hasConverged()){
    return false;
  }
  //std::cout << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f final_transform = icp.getFinalTransformation();

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

void RosPclIcp::eigenToTf(
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

void RosPclIcp::tfToEigen(
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
  transform_eigen = affine.matrix();
}

