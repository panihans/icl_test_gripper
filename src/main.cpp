#include <Arduino.h>
#include "adc.h"
#include "timer.h"
#include "ros.h"
#include "Command.h"
#include "Error.h"
#include "Progress.h"

class DUE_Bluetooth : public ArduinoHardware
{
  public:
    DUE_Bluetooth():ArduinoHardware(&Serial1, 57600){};
};
ros::NodeHandle_<DUE_Bluetooth> nh;

// topic publisher
ieap::Progress progress_msg;
ieap::Error error_msg;
ros::Publisher progress("ieap/progress", &progress_msg);
ros::Publisher error("ieap/error", &error_msg);

// topic subscriber
char buffer[20] = {0};
void commandCb(const ieap::Command& msg) {
  if (strcmp(progress_msg.command, "move") == 0) {
    adcSetpoint = VtoADC(fmax(-1.2, fmin(msg.target, 1.2)));
  } else if (strcmp(progress_msg.command, "hold") == 0) {
    adcSetpoint = ADC_SET_HZ;
  }
  strncpy(buffer, ((String)msg.command).c_str(), 20);
  progress_msg.command = buffer;
  progress_msg.target = msg.target;
}
ros::Subscriber<ieap::Command> command("ieap/command", &commandCb);

void setup() {
  // put your setup code here, to run once:
  setup_adc();
  setup_timers(0.5, 1000);

  nh.initNode();
  nh.advertise(progress);
  nh.advertise(error);
  nh.subscribe(command);
  Serial.begin(115200);
  Serial.println("begin");
}

int32_t load_open = -1;
int32_t shunt_open = -1;
int32_t load_high = -1;
int32_t shunt_high = -1;
void ADC_Handler() {
  if (CAN_READ_ADC_4) {
    if (st == open_circuit) {
      load_open = ADC->ADC_CDR[4];
    } else {
      load_high = ADC->ADC_CDR[4];
    }
  }
  if (CAN_READ_ADC_6) {
    if (st == open_circuit) {
      shunt_open = ADC->ADC_CDR[6];
    } else {
      shunt_high = ADC->ADC_CDR[6];
    }
  }
}

void TC0_Handler() {
  int status = TC0->TC_CHANNEL[0].TC_SR;
  if (status & TC_SR_CPAS) {
    COAST();
    TRIGGER_ADC();
  }
  if (status & TC_SR_CPCS) {
    if (adcSetpoint == ADC_SET_HZ) {
      COAST();
    } else if (adcSetpoint == VtoADC(0)) {
      BRAKE();
      TRIGGER_ADC();
    } else {
      if (load_open < adcSetpoint) {
        // ocv less than setpoint
        if (adcSetpoint < 4095/2) {
          // setpoint negative
          if (load_open < adcSetpoint - 50) {
            // ocv less than setpoint - margin
            FORWARD();
            TRIGGER_ADC();
          }
        } else {
          // setpoint positive
          FORWARD();
          TRIGGER_ADC();
        }
      } else if (load_open > adcSetpoint) {
        // ocv more than setpoint
        if (adcSetpoint > 4095/2) {
          // setpoint positive
          if (load_open > adcSetpoint + 50) {
            // ocv more than setpoint + margin
            REVERSE();
            TRIGGER_ADC();
          }
        } else {
          // setpoint negative
          REVERSE();
          TRIGGER_ADC();
        }
      }
    }
  }
}

uint32_t start = 0;
uint32_t i = 0;
void loop() {
  // while(Serial.available()) {
  //   char c = Serial.read();
  //   switch(c) {
  //     case 'f': adcSetpoint = ADC_SET_MAX; break;
  //     case 'r': adcSetpoint = ADC_SET_MIN; break;
  //     case 'b': adcSetpoint = ADC_SET_ZERO; break;
  //     case 'z': adcSetpoint = ADC_SET_HZ; break;
  //     // case 's': start = millis(); TC_Start(TC0, 0); break;
  //     case 'm': TC0_Handler(); break;
  //   }
  // }

  float i_max = 0.8;

  float duty = fminf(i_max * fabs(ADC_MAX/(VOUT * (ADC_MAX - 2*shunt_high))), 0.9);
  update_timers(duty, 1000);

  if (adcSetpoint != ADC_SET_HZ && load_open < adcSetpoint + 50 && load_open > adcSetpoint - 50) {
    adcSetpoint = ADC_SET_HZ;
  }

  // Serial.println((String)"" + duty + "," + ADCtoV(shunt_high) + "," + ADCtoV(load_open));'
  if (adcSetpoint != ADC_SET_HZ && strcmp(progress_msg.command, "move") == 0){
    if (i > 1000) {
      progress_msg.position = ADCtoV(load_open);
      progress.publish(&progress_msg);
      i = 0;
    } else {
      i++;
    }
  }

  nh.spinOnce();
  Serial.print("i=");
  Serial.print(i);
  Serial.print(" ,lo=");
  Serial.println(load_open);
  delay(1);
}