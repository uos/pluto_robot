#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can/raw.h>

#include <ros/console.h>

#include "can.h"

CAN::CAN()
{
  sockaddr_can addr;
  ifreq ifr;
  char caninterface[] = "can0"; //TODO automatic searching for caninterface

  cansocket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (cansocket_ < 0) {
    ROS_ERROR("can_init: Error opening socket (%s)", strerror(errno));
    exit(1);
  }

  addr.can_family = AF_CAN;

  strcpy(ifr.ifr_name, caninterface);
  if (ioctl(cansocket_, SIOCGIFINDEX, &ifr) < 0) {
    ROS_ERROR("can_init: Error setting SIOCGIFINDEX for interace %s (%s)", caninterface, strerror(errno));
    exit(1);
  }

  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(cansocket_, (sockaddr *)&addr, sizeof(addr)) < 0) {
    ROS_ERROR("can_init: Error binding socket (%s)", strerror(errno));
    exit(1);
  }

  ROS_INFO("CAN interface init done");
}

CAN::~CAN()
{
  if (close(cansocket_) != 0)
    ROS_ERROR("can_close: Error closing can socket (%s)", strerror(errno));
}

bool CAN::send_frame(const can_frame *frame)
{
  if (write(cansocket_, frame, sizeof(*frame)) != sizeof(*frame))
  {
    ROS_ERROR("send_frame: Error writing socket (%s)", strerror(errno));
    return false;
  }
  return true;
}

bool CAN::receive_frame(can_frame *frame)
{
  fd_set rfds;

  FD_ZERO(&rfds);
  FD_SET(cansocket_, &rfds);

  int rc = 1;
  timeval timeout;

  timeout.tv_sec = 5;
  timeout.tv_usec = 0;

  rc = select(cansocket_ + 1, &rfds, NULL, NULL, &timeout);

  if (rc == 0)
  {
    ROS_ERROR("recive_frame: Receiving frame timed out");
    return false;
  }
  else if (rc == -1)
  {
    ROS_WARN("recive_frame: Error receiving frame (%s)", strerror(errno));
    return false;
  }

  //TODO read time stamp
  if (read(cansocket_, frame, sizeof(*frame)) != sizeof(*frame))
  {
    ROS_WARN("receive_frame: Error reading socket (%s)", strerror(errno));
    return false;
  }
  return true;
}
