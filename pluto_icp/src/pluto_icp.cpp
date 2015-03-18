#include <pluto_icp.h>

PlutoICP::PlutoICP(ros::NodeHandle &nh)
 : nh_(nh)
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
  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 guess_transform;
  poseToEigen(target_pose.pose, cloud_pose.pose, guess_transform);
  
  pcl::PCLPointCloud2 pcl_target, pcl_cloud;
  pcl_conversions::toPCL(target, pcl_target);
  pcl_conversions::toPCL(cloud, pcl_cloud);
  
  pcl::PointCloud<pcl::PointXYZ> target_xyz;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;

  pcl::fromPCLPointCloud2(pcl_target, target_xyz);
  pcl::fromPCLPointCloud2(pcl_cloud, cloud_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz_ptr(&target_xyz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(&cloud_xyz);

  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 final_transform;
  pcl::PointCloud<pcl::PointXYZ> result_xyz;
  if(!registerClouds(target_xyz_ptr, cloud_xyz_ptr, guess_transform, result_xyz, final_transform)){
    return false;
  }
  pcl::PCLPointCloud2 pcl_result;
  pcl::toPCLPointCloud2(result_xyz, pcl_result);
  pcl_conversions::fromPCL(pcl_result, result);

  geometry_msgs::PoseStamped pose;
  eigenToPose(final_transform, target_pose.pose, pose.pose);
  return true;
}

void PlutoICP::eigenToPose(
    Eigen::Matrix4f &transformation,
    const geometry_msgs::Pose target_pose,
    geometry_msgs::Pose &pose){

  Eigen::Affine3f affine(transformation);
  Eigen::Quaternionf q = (Eigen::Quaternionf)affine.linear();
  
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
  tf::Pose target_pose_tf;
  tf::poseMsgToTF(target_pose, target_pose_tf);

  tf::Transform result_pose = target_pose_tf.inverseTimes(pose_tf);
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

  tf::Transform transform_tf = cloud_pose_tf.inverseTimes(target_pose_tf);

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
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &guess_transform,
    pcl::PointCloud<pcl::PointXYZ> &result,
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &final_transform){
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
