#ifndef _ROS_mitro_diagnostics_BatteryStatus_h
#define _ROS_mitro_diagnostics_BatteryStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mitro_diagnostics
{

  class BatteryStatus : public ros::Msg
  {
    public:
      float voltage;
      float percent;
      float time;
      bool plugged_in;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_percent;
      u_percent.real = this->percent;
      *(outbuffer + offset + 0) = (u_percent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_percent.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_percent.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_percent.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->percent);
      union {
        float real;
        uint32_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time);
      union {
        bool real;
        uint8_t base;
      } u_plugged_in;
      u_plugged_in.real = this->plugged_in;
      *(outbuffer + offset + 0) = (u_plugged_in.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->plugged_in);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_percent;
      u_percent.base = 0;
      u_percent.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_percent.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_percent.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_percent.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->percent = u_percent.real;
      offset += sizeof(this->percent);
      union {
        float real;
        uint32_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time = u_time.real;
      offset += sizeof(this->time);
      union {
        bool real;
        uint8_t base;
      } u_plugged_in;
      u_plugged_in.base = 0;
      u_plugged_in.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->plugged_in = u_plugged_in.real;
      offset += sizeof(this->plugged_in);
     return offset;
    }

    const char * getType(){ return "mitro_diagnostics/BatteryStatus"; };
    const char * getMD5(){ return "578776034800dd3380cacb386582be74"; };

  };

}
#endif