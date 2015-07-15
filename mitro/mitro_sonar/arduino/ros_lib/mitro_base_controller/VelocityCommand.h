#ifndef _ROS_mitro_base_controller_VelocityCommand_h
#define _ROS_mitro_base_controller_VelocityCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mitro_base_controller
{

  class VelocityCommand : public ros::Msg
  {
    public:
      float velocity_right;
      float velocity_left;

    VelocityCommand():
      velocity_right(0),
      velocity_left(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_velocity_right;
      u_velocity_right.real = this->velocity_right;
      *(outbuffer + offset + 0) = (u_velocity_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_right);
      union {
        float real;
        uint32_t base;
      } u_velocity_left;
      u_velocity_left.real = this->velocity_left;
      *(outbuffer + offset + 0) = (u_velocity_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_left);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_velocity_right;
      u_velocity_right.base = 0;
      u_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_right = u_velocity_right.real;
      offset += sizeof(this->velocity_right);
      union {
        float real;
        uint32_t base;
      } u_velocity_left;
      u_velocity_left.base = 0;
      u_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_left = u_velocity_left.real;
      offset += sizeof(this->velocity_left);
     return offset;
    }

    const char * getType(){ return "mitro_base_controller/VelocityCommand"; };
    const char * getMD5(){ return "fb58ce72c36d928c50040f6da68a7e12"; };

  };

}
#endif