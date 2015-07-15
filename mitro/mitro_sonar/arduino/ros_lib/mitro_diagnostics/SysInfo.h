#ifndef _ROS_mitro_diagnostics_SysInfo_h
#define _ROS_mitro_diagnostics_SysInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mitro_diagnostics/BatteryStatus.h"
#include "mitro_diagnostics/NetworkStatus.h"
#include "mitro_diagnostics/SystemStatus.h"

namespace mitro_diagnostics
{

  class SysInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* hostname;
      mitro_diagnostics::BatteryStatus battery;
      mitro_diagnostics::NetworkStatus network;
      mitro_diagnostics::SystemStatus system;

    SysInfo():
      header(),
      hostname(""),
      battery(),
      network(),
      system()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_hostname = strlen(this->hostname);
      memcpy(outbuffer + offset, &length_hostname, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->hostname, length_hostname);
      offset += length_hostname;
      offset += this->battery.serialize(outbuffer + offset);
      offset += this->network.serialize(outbuffer + offset);
      offset += this->system.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_hostname;
      memcpy(&length_hostname, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hostname; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hostname-1]=0;
      this->hostname = (char *)(inbuffer + offset-1);
      offset += length_hostname;
      offset += this->battery.deserialize(inbuffer + offset);
      offset += this->network.deserialize(inbuffer + offset);
      offset += this->system.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mitro_diagnostics/SysInfo"; };
    const char * getMD5(){ return "e2e9460ff6766419e291895d9db3a296"; };

  };

}
#endif