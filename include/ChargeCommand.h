#ifndef _ROS_tehislihas_ChargeCommand_h
#define _ROS_tehislihas_ChargeCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tehislihas
{

  class ChargeCommand : public ros::Msg
  {
    public:
      typedef float _setpoint_type;
      _setpoint_type setpoint;
      typedef float _current_limit_A_type;
      _current_limit_A_type current_limit_A;

    ChargeCommand():
      setpoint(0),
      current_limit_A(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_setpoint;
      u_setpoint.real = this->setpoint;
      *(outbuffer + offset + 0) = (u_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->setpoint);
      union {
        float real;
        uint32_t base;
      } u_current_limit_A;
      u_current_limit_A.real = this->current_limit_A;
      *(outbuffer + offset + 0) = (u_current_limit_A.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_limit_A.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_limit_A.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_limit_A.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_limit_A);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_setpoint;
      u_setpoint.base = 0;
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->setpoint = u_setpoint.real;
      offset += sizeof(this->setpoint);
      union {
        float real;
        uint32_t base;
      } u_current_limit_A;
      u_current_limit_A.base = 0;
      u_current_limit_A.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_limit_A.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_limit_A.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_limit_A.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_limit_A = u_current_limit_A.real;
      offset += sizeof(this->current_limit_A);
     return offset;
    }

    virtual const char * getType() override { return "tehislihas/ChargeCommand"; };
    virtual const char * getMD5() override { return "e02505a4e3fce4a68d3dffe12262a74f"; };

  };

}
#endif
