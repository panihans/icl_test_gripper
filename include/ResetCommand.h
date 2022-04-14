#ifndef _ROS_tehislihas_ResetCommand_h
#define _ROS_tehislihas_ResetCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tehislihas
{

  class ResetCommand : public ros::Msg
  {
    public:

    ResetCommand()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return "tehislihas/ResetCommand"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
