#ifndef _ROS_tehislihas_Move_h
#define _ROS_tehislihas_Move_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tehislihas
{

  class Move : public ros::Msg
  {
    public:
      typedef float _setpoint_type;
      _setpoint_type setpoint;
      typedef float _speed_type;
      _speed_type speed;

    Move():
      setpoint(0),
      speed(0)
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
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
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
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
     return offset;
    }

    virtual const char * getType() override { return "tehislihas/Move"; };
    virtual const char * getMD5() override { return "d1ed0e1ef102ef5a148ead435262720e"; };

  };

}
#endif
