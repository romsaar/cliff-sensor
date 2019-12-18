#ifndef _ROS_turtlebot3_msgs_PayloadConfig_h
#define _ROS_turtlebot3_msgs_PayloadConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace turtlebot3_msgs
{

  class PayloadConfig : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _buffer_pwm_type;
      _buffer_pwm_type buffer_pwm;
      typedef uint32_t _pump_pwm_type;
      _pump_pwm_type pump_pwm;
      typedef uint8_t _pump_priming_time_type;
      _pump_priming_time_type pump_priming_time;
      typedef uint8_t _pump_on_time_type;
      _pump_on_time_type pump_on_time;
      typedef uint8_t _pump_off_time_type;
      _pump_off_time_type pump_off_time;
      enum { OFF =  0 };
      enum { MANUAL =  1 };
      enum { AUTO =  2 };

    PayloadConfig():
      header(),
      buffer_pwm(0),
      pump_pwm(0),
      pump_priming_time(0),
      pump_on_time(0),
      pump_off_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->buffer_pwm >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->buffer_pwm >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->buffer_pwm >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->buffer_pwm >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buffer_pwm);
      *(outbuffer + offset + 0) = (this->pump_pwm >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pump_pwm >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pump_pwm >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pump_pwm >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pump_pwm);
      *(outbuffer + offset + 0) = (this->pump_priming_time >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pump_priming_time);
      *(outbuffer + offset + 0) = (this->pump_on_time >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pump_on_time);
      *(outbuffer + offset + 0) = (this->pump_off_time >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pump_off_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->buffer_pwm =  ((uint32_t) (*(inbuffer + offset)));
      this->buffer_pwm |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->buffer_pwm |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->buffer_pwm |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->buffer_pwm);
      this->pump_pwm =  ((uint32_t) (*(inbuffer + offset)));
      this->pump_pwm |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pump_pwm |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pump_pwm |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pump_pwm);
      this->pump_priming_time =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pump_priming_time);
      this->pump_on_time =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pump_on_time);
      this->pump_off_time =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pump_off_time);
     return offset;
    }

    const char * getType(){ return "turtlebot3_msgs/PayloadConfig"; };
    const char * getMD5(){ return "386cde40017379b35641731d58b585c8"; };

  };

}
#endif
