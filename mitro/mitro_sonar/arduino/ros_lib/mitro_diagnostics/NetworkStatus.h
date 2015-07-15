#ifndef _ROS_mitro_diagnostics_NetworkStatus_h
#define _ROS_mitro_diagnostics_NetworkStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mitro_diagnostics
{

  class NetworkStatus : public ros::Msg
  {
    public:
      float wifi_signallevel;
      bool ethernet_connected;

    NetworkStatus():
      wifi_signallevel(0),
      ethernet_connected(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wifi_signallevel;
      u_wifi_signallevel.real = this->wifi_signallevel;
      *(outbuffer + offset + 0) = (u_wifi_signallevel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wifi_signallevel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wifi_signallevel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wifi_signallevel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wifi_signallevel);
      union {
        bool real;
        uint8_t base;
      } u_ethernet_connected;
      u_ethernet_connected.real = this->ethernet_connected;
      *(outbuffer + offset + 0) = (u_ethernet_connected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ethernet_connected);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wifi_signallevel;
      u_wifi_signallevel.base = 0;
      u_wifi_signallevel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wifi_signallevel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wifi_signallevel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wifi_signallevel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wifi_signallevel = u_wifi_signallevel.real;
      offset += sizeof(this->wifi_signallevel);
      union {
        bool real;
        uint8_t base;
      } u_ethernet_connected;
      u_ethernet_connected.base = 0;
      u_ethernet_connected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ethernet_connected = u_ethernet_connected.real;
      offset += sizeof(this->ethernet_connected);
     return offset;
    }

    const char * getType(){ return "mitro_diagnostics/NetworkStatus"; };
    const char * getMD5(){ return "86243f4c11ccb324213cce13ddb09a93"; };

  };

}
#endif