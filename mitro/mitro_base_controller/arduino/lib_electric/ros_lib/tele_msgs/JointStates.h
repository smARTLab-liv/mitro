#ifndef ros_JointStates_h
#define ros_JointStates_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"

namespace tele_msgs
{

  class JointStates : public ros::Msg
  {
    public:
      std_msgs::Header header;
      long position_right;
      long position_left;
      float velocity_right;
      float velocity_left;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        long real;
        unsigned long base;
      } u_position_right;
      u_position_right.real = this->position_right;
      *(outbuffer + offset + 0) = (u_position_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_right);
      union {
        long real;
        unsigned long base;
      } u_position_left;
      u_position_left.real = this->position_left;
      *(outbuffer + offset + 0) = (u_position_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_left);
      union {
        float real;
        unsigned long base;
      } u_velocity_right;
      u_velocity_right.real = this->velocity_right;
      *(outbuffer + offset + 0) = (u_velocity_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_right);
      union {
        float real;
        unsigned long base;
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
        long real;
        unsigned long base;
      } u_position_right;
      u_position_right.base = 0;
      u_position_right.base |= ((typeof(u_position_right.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_right.base |= ((typeof(u_position_right.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_right.base |= ((typeof(u_position_right.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_right.base |= ((typeof(u_position_right.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_right = u_position_right.real;
      offset += sizeof(this->position_right);
      union {
        long real;
        unsigned long base;
      } u_position_left;
      u_position_left.base = 0;
      u_position_left.base |= ((typeof(u_position_left.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_left.base |= ((typeof(u_position_left.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_left.base |= ((typeof(u_position_left.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_left.base |= ((typeof(u_position_left.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_left = u_position_left.real;
      offset += sizeof(this->position_left);
      union {
        float real;
        unsigned long base;
      } u_velocity_right;
      u_velocity_right.base = 0;
      u_velocity_right.base |= ((typeof(u_velocity_right.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_right.base |= ((typeof(u_velocity_right.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_right.base |= ((typeof(u_velocity_right.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_right.base |= ((typeof(u_velocity_right.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_right = u_velocity_right.real;
      offset += sizeof(this->velocity_right);
      union {
        float real;
        unsigned long base;
      } u_velocity_left;
      u_velocity_left.base = 0;
      u_velocity_left.base |= ((typeof(u_velocity_left.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_left.base |= ((typeof(u_velocity_left.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_left.base |= ((typeof(u_velocity_left.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_left.base |= ((typeof(u_velocity_left.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_left = u_velocity_left.real;
      offset += sizeof(this->velocity_left);
     return offset;
    }

    const char * getType(){ return "tele_msgs/JointStates"; };

  };

}
#endif