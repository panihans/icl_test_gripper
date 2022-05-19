#ifndef _ROS_am_soft_grip_msgs_ActuatorFeedback_h
#define _ROS_am_soft_grip_msgs_ActuatorFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace am_soft_grip_msgs
{

  class ActuatorFeedback : public ros::Msg
  {
    public:
      typedef int8_t _actuator_state_type;
      _actuator_state_type actuator_state;
      typedef float _icl_charging_voltage_type;
      _icl_charging_voltage_type icl_charging_voltage;
      typedef float _icl_open_circuit_voltage_type;
      _icl_open_circuit_voltage_type icl_open_circuit_voltage;
      typedef float _icl_coulomb_counter_type;
      _icl_coulomb_counter_type icl_coulomb_counter;
      typedef ros::Time _last_command_type;
      _last_command_type last_command;
      enum { CHARGING = 0                         };
      enum { OPEN_CIRCUIT = 1                     };
      enum { SHORT_CIRCUIT = 2                    };
      enum { MEASURING = 3                        };

    ActuatorFeedback():
      actuator_state(0),
      icl_charging_voltage(0),
      icl_open_circuit_voltage(0),
      icl_coulomb_counter(0),
      last_command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_actuator_state;
      u_actuator_state.real = this->actuator_state;
      *(outbuffer + offset + 0) = (u_actuator_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->actuator_state);
      union {
        float real;
        uint32_t base;
      } u_icl_charging_voltage;
      u_icl_charging_voltage.real = this->icl_charging_voltage;
      *(outbuffer + offset + 0) = (u_icl_charging_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_icl_charging_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_icl_charging_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_icl_charging_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->icl_charging_voltage);
      union {
        float real;
        uint32_t base;
      } u_icl_open_circuit_voltage;
      u_icl_open_circuit_voltage.real = this->icl_open_circuit_voltage;
      *(outbuffer + offset + 0) = (u_icl_open_circuit_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_icl_open_circuit_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_icl_open_circuit_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_icl_open_circuit_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->icl_open_circuit_voltage);
      union {
        float real;
        uint32_t base;
      } u_icl_coulomb_counter;
      u_icl_coulomb_counter.real = this->icl_coulomb_counter;
      *(outbuffer + offset + 0) = (u_icl_coulomb_counter.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_icl_coulomb_counter.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_icl_coulomb_counter.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_icl_coulomb_counter.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->icl_coulomb_counter);
      *(outbuffer + offset + 0) = (this->last_command.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_command.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_command.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_command.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_command.sec);
      *(outbuffer + offset + 0) = (this->last_command.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_command.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_command.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_command.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_command.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_actuator_state;
      u_actuator_state.base = 0;
      u_actuator_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->actuator_state = u_actuator_state.real;
      offset += sizeof(this->actuator_state);
      union {
        float real;
        uint32_t base;
      } u_icl_charging_voltage;
      u_icl_charging_voltage.base = 0;
      u_icl_charging_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_icl_charging_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_icl_charging_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_icl_charging_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->icl_charging_voltage = u_icl_charging_voltage.real;
      offset += sizeof(this->icl_charging_voltage);
      union {
        float real;
        uint32_t base;
      } u_icl_open_circuit_voltage;
      u_icl_open_circuit_voltage.base = 0;
      u_icl_open_circuit_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_icl_open_circuit_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_icl_open_circuit_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_icl_open_circuit_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->icl_open_circuit_voltage = u_icl_open_circuit_voltage.real;
      offset += sizeof(this->icl_open_circuit_voltage);
      union {
        float real;
        uint32_t base;
      } u_icl_coulomb_counter;
      u_icl_coulomb_counter.base = 0;
      u_icl_coulomb_counter.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_icl_coulomb_counter.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_icl_coulomb_counter.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_icl_coulomb_counter.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->icl_coulomb_counter = u_icl_coulomb_counter.real;
      offset += sizeof(this->icl_coulomb_counter);
      this->last_command.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->last_command.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_command.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_command.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_command.sec);
      this->last_command.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->last_command.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_command.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_command.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_command.nsec);
     return offset;
    }

    virtual const char * getType() override { return "am_soft_grip_msgs/ActuatorFeedback"; };
    virtual const char * getMD5() override { return "0c3b457aa000480d1676e61e22b9e755"; };

  };

}
#endif
