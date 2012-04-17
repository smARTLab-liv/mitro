#ifndef _ROS_mitro_sonar_SonarRaw_h
#define _ROS_mitro_sonar_SonarRaw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mitro_sonar
{

  class SonarRaw : public ros::Msg
  {
    public:
      int16_t s1_dist;
      int16_t s2_dist;
      int16_t s3_dist;
      int16_t s4_dist;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_s1_dist;
      u_s1_dist.real = this->s1_dist;
      *(outbuffer + offset + 0) = (u_s1_dist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_s1_dist.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->s1_dist);
      union {
        int16_t real;
        uint16_t base;
      } u_s2_dist;
      u_s2_dist.real = this->s2_dist;
      *(outbuffer + offset + 0) = (u_s2_dist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_s2_dist.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->s2_dist);
      union {
        int16_t real;
        uint16_t base;
      } u_s3_dist;
      u_s3_dist.real = this->s3_dist;
      *(outbuffer + offset + 0) = (u_s3_dist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_s3_dist.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->s3_dist);
      union {
        int16_t real;
        uint16_t base;
      } u_s4_dist;
      u_s4_dist.real = this->s4_dist;
      *(outbuffer + offset + 0) = (u_s4_dist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_s4_dist.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->s4_dist);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_s1_dist;
      u_s1_dist.base = 0;
      u_s1_dist.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_s1_dist.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->s1_dist = u_s1_dist.real;
      offset += sizeof(this->s1_dist);
      union {
        int16_t real;
        uint16_t base;
      } u_s2_dist;
      u_s2_dist.base = 0;
      u_s2_dist.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_s2_dist.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->s2_dist = u_s2_dist.real;
      offset += sizeof(this->s2_dist);
      union {
        int16_t real;
        uint16_t base;
      } u_s3_dist;
      u_s3_dist.base = 0;
      u_s3_dist.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_s3_dist.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->s3_dist = u_s3_dist.real;
      offset += sizeof(this->s3_dist);
      union {
        int16_t real;
        uint16_t base;
      } u_s4_dist;
      u_s4_dist.base = 0;
      u_s4_dist.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_s4_dist.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->s4_dist = u_s4_dist.real;
      offset += sizeof(this->s4_dist);
     return offset;
    }

    const char * getType(){ return "mitro_sonar/SonarRaw"; };
    const char * getMD5(){ return "ef1304116228f22e51da1a232e0d6808"; };

  };

}
#endif