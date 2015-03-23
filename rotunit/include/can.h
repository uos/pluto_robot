#ifndef _CAN_H_
#define _CAN_H_

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>

class CAN
{
  public:
    CAN();
    ~CAN();

    bool send_frame(const can_frame *frame);
    bool receive_frame(can_frame *frame);

  private:
    int cansocket_;
};

#endif
