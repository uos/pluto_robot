#include "rotunit.h"

Rotunit::Rotunit(ros::NodeHandle &nh)
  : nh_(nh)
{
  pub_ = nh_.advertise<sensor_msgs::JointState> ("joint_states", 1);
  sub_ = nh_.subscribe("rot_vel", 10, &Rotunit::rotunitCallback, this);

  double rotunit_speed;
  nh_.param("rotunit_speed", rotunit_speed, M_PI/6.0);
  can_rotunit_send(rotunit_speed);
}

Rotunit::~Rotunit()
{
  can_rotunit_send(0.0);
}

void Rotunit::can_rotunit_send(double speed)
{
  int ticks =  (int)(speed / (2.0 * M_PI) * 10240 / 20);
  can_frame frame;
  frame.can_id = CAN_SETROTUNT;
  frame.can_dlc = 8;
  frame.data[0] = (ticks >> 8); //high
  frame.data[1] = (ticks & 0xFF); //low
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  if(!can_.send_frame(&frame))
    ROS_ERROR("can_rotunit_send: Error sending rotunit speed");
}

double Rotunit::normalize2PI (double angle)
{
  while (angle > 2.0 * M_PI)
    angle -= 2.0 * M_PI;
  while (angle < 0)
    angle += 2.0 * M_PI;
  return angle;
}

void Rotunit::rotunitCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  can_rotunit_send(msg->angular.z);
}

void Rotunit::can_rotunit(const can_frame &frame)
{
  int rot = (frame.data[1] << 8) + frame.data[2];
  double rot2 = rot * 2 * M_PI / 10240;
  rot2 = rot2 - 6 * M_PI / 180.0; // 6 degree angle correction
  rot2 = normalize2PI(rot2);      // normalize to element of [0 2PI]

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(1);
  joint_state.position.resize(1);
  joint_state.name[0] = "laser_rot_joint";
  joint_state.position[0] = rot2;
  
  pub_.publish(joint_state);
}

int Rotunit::can_read_fifo()
{
  can_frame frame;
  if(!can_.receive_frame(&frame))
    return -1;

  switch (frame.can_id) {
    case CAN_GETROTUNIT:
      can_rotunit(frame);
      break;
  }
  return frame.can_id;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotunit");
  ros::NodeHandle nh;
  Rotunit rotunit(nh);

  while (ros::ok())
  {
    rotunit.can_read_fifo();
    ros::spinOnce();
  }
  return 0;
}

