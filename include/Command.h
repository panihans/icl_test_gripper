#ifndef _ROS_ieap_Command_h
#define _ROS_ieap_Command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ieap
{

  class Command : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;
      typedef float _target_type;
      _target_type target;

    Command():
      command(""),
      target(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      union {
        float real;
        uint32_t base;
      } u_target;
      u_target.real = this->target;
      *(outbuffer + offset + 0) = (u_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      union {
        float real;
        uint32_t base;
      } u_target;
      u_target.base = 0;
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target = u_target.real;
      offset += sizeof(this->target);
     return offset;
    }

    virtual const char * getType() override { return "ieap/Command"; };
    virtual const char * getMD5() override { return "5fd22fe7e0c35fe191b0a7d47832a088"; };

  };

}
#endif