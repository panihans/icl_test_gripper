#ifndef _ROS_tehislihas_ChargerStatus_h
#define _ROS_tehislihas_ChargerStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tehislihas
{

  class ChargerStatus : public ros::Msg
  {
    public:
      typedef uint16_t _charger_status_code_type;
      _charger_status_code_type charger_status_code;
      typedef float _shunt_closed_type;
      _shunt_closed_type shunt_closed;
      typedef float _shunt_open_type;
      _shunt_open_type shunt_open;
      typedef float _load_closed_type;
      _load_closed_type load_closed;
      typedef float _load_open_type;
      _load_open_type load_open;

    ChargerStatus():
      charger_status_code(0),
      shunt_closed(0),
      shunt_open(0),
      load_closed(0),
      load_open(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->charger_status_code >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->charger_status_code >> (8 * 1)) & 0xFF;
      offset += sizeof(this->charger_status_code);
      union {
        float real;
        uint32_t base;
      } u_shunt_closed;
      u_shunt_closed.real = this->shunt_closed;
      *(outbuffer + offset + 0) = (u_shunt_closed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_shunt_closed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_shunt_closed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_shunt_closed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->shunt_closed);
      union {
        float real;
        uint32_t base;
      } u_shunt_open;
      u_shunt_open.real = this->shunt_open;
      *(outbuffer + offset + 0) = (u_shunt_open.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_shunt_open.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_shunt_open.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_shunt_open.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->shunt_open);
      union {
        float real;
        uint32_t base;
      } u_load_closed;
      u_load_closed.real = this->load_closed;
      *(outbuffer + offset + 0) = (u_load_closed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_load_closed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_load_closed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_load_closed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->load_closed);
      union {
        float real;
        uint32_t base;
      } u_load_open;
      u_load_open.real = this->load_open;
      *(outbuffer + offset + 0) = (u_load_open.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_load_open.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_load_open.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_load_open.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->load_open);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->charger_status_code =  ((uint16_t) (*(inbuffer + offset)));
      this->charger_status_code |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->charger_status_code);
      union {
        float real;
        uint32_t base;
      } u_shunt_closed;
      u_shunt_closed.base = 0;
      u_shunt_closed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_shunt_closed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_shunt_closed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_shunt_closed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->shunt_closed = u_shunt_closed.real;
      offset += sizeof(this->shunt_closed);
      union {
        float real;
        uint32_t base;
      } u_shunt_open;
      u_shunt_open.base = 0;
      u_shunt_open.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_shunt_open.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_shunt_open.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_shunt_open.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->shunt_open = u_shunt_open.real;
      offset += sizeof(this->shunt_open);
      union {
        float real;
        uint32_t base;
      } u_load_closed;
      u_load_closed.base = 0;
      u_load_closed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_load_closed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_load_closed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_load_closed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->load_closed = u_load_closed.real;
      offset += sizeof(this->load_closed);
      union {
        float real;
        uint32_t base;
      } u_load_open;
      u_load_open.base = 0;
      u_load_open.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_load_open.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_load_open.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_load_open.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->load_open = u_load_open.real;
      offset += sizeof(this->load_open);
     return offset;
    }

    virtual const char * getType() override { return "tehislihas/ChargerStatus"; };
    virtual const char * getMD5() override { return "41398ad683cb1ca6acde23ee67a77fcb"; };

  };

}
#endif
