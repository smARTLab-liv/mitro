#ifndef _ROS_mitro_sonar_SonarScan_h
#define _ROS_mitro_sonar_SonarScan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mitro_sonar
{

  class SonarScan : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float arc_range;
      float min_range;
      float max_range;
      uint8_t ranges_length;
      float st_ranges;
      float * ranges;
      uint8_t angles_length;
      float st_angles;
      float * angles;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_arc_range;
      u_arc_range.real = this->arc_range;
      *(outbuffer + offset + 0) = (u_arc_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arc_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arc_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arc_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arc_range);
      union {
        float real;
        uint32_t base;
      } u_min_range;
      u_min_range.real = this->min_range;
      *(outbuffer + offset + 0) = (u_min_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_range);
      union {
        float real;
        uint32_t base;
      } u_max_range;
      u_max_range.real = this->max_range;
      *(outbuffer + offset + 0) = (u_max_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_range);
      *(outbuffer + offset++) = ranges_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < ranges_length; i++){
      union {
        float real;
        uint32_t base;
      } u_rangesi;
      u_rangesi.real = this->ranges[i];
      *(outbuffer + offset + 0) = (u_rangesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rangesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rangesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rangesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ranges[i]);
      }
      *(outbuffer + offset++) = angles_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < angles_length; i++){
      union {
        float real;
        uint32_t base;
      } u_anglesi;
      u_anglesi.real = this->angles[i];
      *(outbuffer + offset + 0) = (u_anglesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angles[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_arc_range;
      u_arc_range.base = 0;
      u_arc_range.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arc_range.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arc_range.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arc_range.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arc_range = u_arc_range.real;
      offset += sizeof(this->arc_range);
      union {
        float real;
        uint32_t base;
      } u_min_range;
      u_min_range.base = 0;
      u_min_range.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_range.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_range.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_range.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_range = u_min_range.real;
      offset += sizeof(this->min_range);
      union {
        float real;
        uint32_t base;
      } u_max_range;
      u_max_range.base = 0;
      u_max_range.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_range.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_range.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_range.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_range = u_max_range.real;
      offset += sizeof(this->max_range);
      uint8_t ranges_lengthT = *(inbuffer + offset++);
      if(ranges_lengthT > ranges_length)
        this->ranges = (float*)realloc(this->ranges, ranges_lengthT * sizeof(float));
      offset += 3;
      ranges_length = ranges_lengthT;
      for( uint8_t i = 0; i < ranges_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_ranges;
      u_st_ranges.base = 0;
      u_st_ranges.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_ranges.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_ranges.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_ranges.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_ranges = u_st_ranges.real;
      offset += sizeof(this->st_ranges);
        memcpy( &(this->ranges[i]), &(this->st_ranges), sizeof(float));
      }
      uint8_t angles_lengthT = *(inbuffer + offset++);
      if(angles_lengthT > angles_length)
        this->angles = (float*)realloc(this->angles, angles_lengthT * sizeof(float));
      offset += 3;
      angles_length = angles_lengthT;
      for( uint8_t i = 0; i < angles_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_angles;
      u_st_angles.base = 0;
      u_st_angles.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_angles.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_angles.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_angles.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_angles = u_st_angles.real;
      offset += sizeof(this->st_angles);
        memcpy( &(this->angles[i]), &(this->st_angles), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "mitro_sonar/SonarScan"; };
    const char * getMD5(){ return "4a5183f6e8278504cca61d3cde56c072"; };

  };

}
#endif