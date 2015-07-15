#ifndef _ROS_mitro_diagnostics_SystemStatus_h
#define _ROS_mitro_diagnostics_SystemStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mitro_diagnostics
{

  class SystemStatus : public ros::Msg
  {
    public:
      float cpu_usage_average;
      uint8_t cpu_usage_detail_length;
      float st_cpu_usage_detail;
      float * cpu_usage_detail;
      float cpu_temp_average;
      uint8_t cpu_temp_detail_length;
      float st_cpu_temp_detail;
      float * cpu_temp_detail;
      float mem_usage;

    SystemStatus():
      cpu_usage_average(0),
      cpu_usage_detail_length(0), cpu_usage_detail(NULL),
      cpu_temp_average(0),
      cpu_temp_detail_length(0), cpu_temp_detail(NULL),
      mem_usage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_cpu_usage_average;
      u_cpu_usage_average.real = this->cpu_usage_average;
      *(outbuffer + offset + 0) = (u_cpu_usage_average.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_usage_average.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_usage_average.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_usage_average.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_usage_average);
      *(outbuffer + offset++) = cpu_usage_detail_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cpu_usage_detail_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cpu_usage_detaili;
      u_cpu_usage_detaili.real = this->cpu_usage_detail[i];
      *(outbuffer + offset + 0) = (u_cpu_usage_detaili.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_usage_detaili.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_usage_detaili.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_usage_detaili.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_usage_detail[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_cpu_temp_average;
      u_cpu_temp_average.real = this->cpu_temp_average;
      *(outbuffer + offset + 0) = (u_cpu_temp_average.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_temp_average.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_temp_average.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_temp_average.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_temp_average);
      *(outbuffer + offset++) = cpu_temp_detail_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cpu_temp_detail_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cpu_temp_detaili;
      u_cpu_temp_detaili.real = this->cpu_temp_detail[i];
      *(outbuffer + offset + 0) = (u_cpu_temp_detaili.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_temp_detaili.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_temp_detaili.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_temp_detaili.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_temp_detail[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_mem_usage;
      u_mem_usage.real = this->mem_usage;
      *(outbuffer + offset + 0) = (u_mem_usage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mem_usage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mem_usage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mem_usage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mem_usage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_cpu_usage_average;
      u_cpu_usage_average.base = 0;
      u_cpu_usage_average.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_usage_average.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_usage_average.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_usage_average.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_usage_average = u_cpu_usage_average.real;
      offset += sizeof(this->cpu_usage_average);
      uint8_t cpu_usage_detail_lengthT = *(inbuffer + offset++);
      if(cpu_usage_detail_lengthT > cpu_usage_detail_length)
        this->cpu_usage_detail = (float*)realloc(this->cpu_usage_detail, cpu_usage_detail_lengthT * sizeof(float));
      offset += 3;
      cpu_usage_detail_length = cpu_usage_detail_lengthT;
      for( uint8_t i = 0; i < cpu_usage_detail_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cpu_usage_detail;
      u_st_cpu_usage_detail.base = 0;
      u_st_cpu_usage_detail.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cpu_usage_detail.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cpu_usage_detail.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cpu_usage_detail.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cpu_usage_detail = u_st_cpu_usage_detail.real;
      offset += sizeof(this->st_cpu_usage_detail);
        memcpy( &(this->cpu_usage_detail[i]), &(this->st_cpu_usage_detail), sizeof(float));
      }
      union {
        float real;
        uint32_t base;
      } u_cpu_temp_average;
      u_cpu_temp_average.base = 0;
      u_cpu_temp_average.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_temp_average.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_temp_average.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_temp_average.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_temp_average = u_cpu_temp_average.real;
      offset += sizeof(this->cpu_temp_average);
      uint8_t cpu_temp_detail_lengthT = *(inbuffer + offset++);
      if(cpu_temp_detail_lengthT > cpu_temp_detail_length)
        this->cpu_temp_detail = (float*)realloc(this->cpu_temp_detail, cpu_temp_detail_lengthT * sizeof(float));
      offset += 3;
      cpu_temp_detail_length = cpu_temp_detail_lengthT;
      for( uint8_t i = 0; i < cpu_temp_detail_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cpu_temp_detail;
      u_st_cpu_temp_detail.base = 0;
      u_st_cpu_temp_detail.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cpu_temp_detail.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cpu_temp_detail.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cpu_temp_detail.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cpu_temp_detail = u_st_cpu_temp_detail.real;
      offset += sizeof(this->st_cpu_temp_detail);
        memcpy( &(this->cpu_temp_detail[i]), &(this->st_cpu_temp_detail), sizeof(float));
      }
      union {
        float real;
        uint32_t base;
      } u_mem_usage;
      u_mem_usage.base = 0;
      u_mem_usage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mem_usage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mem_usage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mem_usage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mem_usage = u_mem_usage.real;
      offset += sizeof(this->mem_usage);
     return offset;
    }

    const char * getType(){ return "mitro_diagnostics/SystemStatus"; };
    const char * getMD5(){ return "52870c8e59abb1c558aeef6f2ca85302"; };

  };

}
#endif