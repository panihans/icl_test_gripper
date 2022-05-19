#ifndef _ROS_am_soft_grip_msgs_ChargeCommand_h
#define _ROS_am_soft_grip_msgs_ChargeCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace am_soft_grip_msgs
{

  class ChargeCommand : public ros::Msg
  {
    public:
      typedef ros::Time _sent_type;
      _sent_type sent;
      typedef float _icl_target_voltage_type;
      _icl_target_voltage_type icl_target_voltage;
      typedef float _icl_current_limit_type;
      _icl_current_limit_type icl_current_limit;

    ChargeCommand():
      sent(),
      icl_target_voltage(0),
      icl_current_limit(0)
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
      union {
        float real;
        uint32_t base;
      } u_icl_target_voltage;
      u_icl_target_voltage.real = this->icl_target_voltage;
      *(outbuffer + offset + 0) = (u_icl_target_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_icl_target_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_icl_target_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_icl_target_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->icl_target_voltage);
      union {
        float real;
        uint32_t base;
      } u_icl_current_limit;
      u_icl_current_limit.real = this->icl_current_limit;
      *(outbuffer + offset + 0) = (u_icl_current_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_icl_current_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_icl_current_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_icl_current_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->icl_current_limit);
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
      union {
        float real;
        uint32_t base;
      } u_icl_target_voltage;
      u_icl_target_voltage.base = 0;
      u_icl_target_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_icl_target_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_icl_target_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_icl_target_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->icl_target_voltage = u_icl_target_voltage.real;
      offset += sizeof(this->icl_target_voltage);
      union {
        float real;
        uint32_t base;
      } u_icl_current_limit;
      u_icl_current_limit.base = 0;
      u_icl_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_icl_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_icl_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_icl_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->icl_current_limit = u_icl_current_limit.real;
      offset += sizeof(this->icl_current_limit);
     return offset;
    }

    virtual const char * getType() override { return "am_soft_grip_msgs/ChargeCommand"; };
    virtual const char * getMD5() override { return "bb51eeb233458673b8ea9bd6fa833dbd"; };

  };

}
#endif
