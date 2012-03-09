#ifndef _ROS_mitro_base_controller_JointStates_h
#define _ROS_mitro_base_controller_JointStates_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mitro_base_controller
{

  class JointStates : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int32_t position_right;
      int32_t position_left;
      float velocity_right;
      float velocity_left;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_position_right;
      u_position_right.real = this->position_right;
      *(outbuffer + offset + 0) = (u_position_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_right);
      union {
        int32_t real;
        uint32_t base;
      } u_position_left;
      u_position_left.real = this->position_left;
      *(outbuffer + offset + 0) = (u_position_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_left);
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
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_position_right;
      u_position_right.base = 0;
      u_position_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_right = u_position_right.real;
      offset += sizeof(this->position_right);
      union {
        int32_t real;
        uint32_t base;
      } u_position_left;
      u_position_left.base = 0;
      u_position_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_left = u_position_left.real;
      offset += sizeof(this->position_left);
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

    const char * getType(){ return "mitro_base_controller/JointStates"; };
    const char * getMD5(){ return "0554e8135eb3764b5bdbad97993957b9"; };

  };

}
#endif