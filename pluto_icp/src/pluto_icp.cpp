#include <pluto_icp.h>

PlutoICP::PlutoICP(ros::NodeHandle &nh)
 :  nh_(nh),
    tf_listener(ros::Duration(60.0))
{
  tf::Transform null_transform;
  null_transform.setIdentity();
  map_to_odom.push(tf::StampedTransform(null_transform, ros::Time::now(), "map", "odom_combined"));
  service = nh_.advertiseService("pluto_icp", &PlutoICP::registerCloudsSrv, this);
  cloud_sub = nh_.subscribe("assembled_cloud", 20, &PlutoICP::icpUpdateMapToOdomCombined, this);
  cloud_pub = nh_.advertise<sensor_msgs::PointCloud2> ("registered_cloud", 1);
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

void PlutoICP::icpUpdateMapToOdomCombined(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  ROS_INFO("ros_icp received new point cloud");
  
  geometry_msgs::PoseStamped cloud_pose;
  getPointCloudPose(*cloud, "map", cloud_pose);
  
  if(!clouds.empty()){

    geometry_msgs::PoseStamped target_pose = poses.top();
    geometry_msgs::PoseStamped result_pose;
    geometry_msgs::Transform delta_transform;

    // register point clouds with icp
    ROS_INFO("register point clouds...");
    registerClouds(clouds.top(),
                   target_pose,
                   *cloud,
                   cloud_pose,
                   result_pose,
                   delta_transform);
    ROS_INFO("... register point clouds done.");

    // convert to tf
    tf::Transform delta_transform_tf;
    tf::transformMsgToTF(delta_transform, delta_transform_tf);

    // calculate the new map to odom_combined transformation
    tf::Transform update_transform_tf = map_to_odom.top() * delta_transform_tf;
    tf::StampedTransform update_transform_stamped_tf(
      update_transform_tf, cloud->header.stamp, "map", "odom_combined");
    map_to_odom.push(update_transform_stamped_tf);
    
    // update the cloud pose
    cloud_pose = result_pose;
  }

  // save cloud and pose for the next iteration
  poses.push(cloud_pose);
  clouds.push(*cloud);
}

void PlutoICP::sendMapToOdomCombined(){

  ros::Time now = ros::Time::now();

  tf_broadcaster.sendTransform(
    tf::StampedTransform(
      map_to_odom.top(),
      now,
      "map",
      "odom_combined"
    )
  );

  if (last_sent.stamp_ != map_to_odom.top().stamp_){
    tf::Transform transform = map_to_odom.top();
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

    sensor_msgs::PointCloud2 cloud = clouds.top();
    cloud.header.stamp = now;
    cloud_pub.publish(cloud);
  }
  last_sent = map_to_odom.top();
}

int main(int args, char** argv){
  ros::init(args, argv, "pluto_icp");
  ros::NodeHandle nh;

  PlutoICP pluto_icp(nh);
  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  ros::Rate rate(10.0);
  while(nh.ok()){
    pluto_icp.sendMapToOdomCombined();
    rate.sleep();
  }
}
