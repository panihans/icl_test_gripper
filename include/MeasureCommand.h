#ifndef _ROS_am_soft_grip_msgs_MeasureCommand_h
#define _ROS_am_soft_grip_msgs_MeasureCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace am_soft_grip_msgs
{

  class MeasureCommand : public ros::Msg
  {
    public:
      typedef ros::Time _sent_type;
      _sent_type sent;

    MeasureCommand():
      sent()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sent.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sent.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sent.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sent.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sent.sec);
      *(outbuffer + offset + 0) = (this->sent.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sent.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sent.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sent.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sent.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->sent.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->sent.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sent.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sent.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sent.sec);
      this->sent.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->sent.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sent.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sent.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sent.nsec);
     return offset;
    }

    virtual const char * getType() override { return "am_soft_grip_msgs/MeasureCommand"; };
    virtual const char * getMD5() override { return "0fd60630ead3e0cd3941d3d853e26a8c"; };

  };

}
#endif
