#include "laser_ps3_control.h"

LaserPS3Control::LaserPS3Control(ros::NodeHandle &nh)
  : nh_(nh),
    ac("scan360", true),
    vel_(0),
    scan_vel_(0.6)
{

  ROS_INFO("Waiting for snapshotter360 action server to start.");
  ac.waitForServer(); 

  ROS_INFO("Connected to server.");
  nh_.param("acc", acc_, 0.01);           // max velocity
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("rot_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 15,  &LaserPS3Control::PS3Callback, this);
  rot_vel_client_ = nh_.serviceClient<rotunit::RotVelSrv>("rotunit_velocity");
}

LaserPS3Control::~LaserPS3Control(){

}

void LaserPS3Control::PS3Callback(const sensor_msgs::Joy::ConstPtr &joy){
  rotunit::RotVelSrv rotvel;
  if(joy->buttons[PS3_BUTTON_REAR_RIGHT_2])
    vel_ += acc_;
  if(joy->buttons[PS3_BUTTON_REAR_RIGHT_1])
    vel_ -= acc_;
  
  if(vel_ > 1.3) vel_ = 1.3;
  if(vel_ < -1.3) vel_ = -1.3;

  rotvel.request.twist.angular.z = vel_;
  rot_vel_client_.call(rotvel);

  //vel_pub_.publish(msg);
  
  if(joy->buttons[PS3_BUTTON_ACTION_CIRCLE]){
    int ret;
    ret = system("source ~/ros/muffin-dry/setup.sh");
    ROS_INFO("System call \"source\" returned %d", ret);
    ros::Duration(1.0).sleep();
  }

  rotunit_snapshotter::Scan360Goal goal;
  if(joy->buttons[PS3_BUTTON_ACTION_TRIANGLE]){
    rotvel.request.twist.angular.z = scan_vel_;
    rot_vel_client_.call(rotvel);
    ros::Duration(3.0).sleep();
    ROS_INFO("start scanning.");
    goal.number_rounds = 1;
    // send a goal to the action
    ac.sendGoal(goal);
    bool finished_before_timeout =
      ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state =
        ac.getState();
      ROS_INFO("Action finished:%s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
 
    rotvel.request.twist.angular.z = vel_;
    rot_vel_client_.call(rotvel);
  }
}

int main(int args, char**argv){
  ros::init(args, argv, "laser_ps3_control");
  
  ros::NodeHandle nh;
  
  LaserPS3Control laser_ps3_crontrol(nh);

  while(ros::ok){
    ros::spinOnce();
  }
  return EXIT_SUCCESS;
}
