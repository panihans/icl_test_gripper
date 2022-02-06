#include <Arduino.h>
#include "adc.h"
#include "timer.h"

enum charging_state {high, open_circuit};
charging_state st = open_circuit;

void setup() {
  // put your setup code here, to run once:
  setup_adc();
  setup_timers(0.5, 1000);
  Serial.begin(115200);
  Serial.println("begin");
  Serial.println((String)"millisekund, setpoint, laadimispinge, tehislihase avatud ahela pinge, shunt avatud ahela pinge, tehislihas laadimispinge, shunt laadimispinge");
}

#define VOUT 3.3f
#define ADC_MAX 4095.f
#define ADCtoV(adc) ((adc) / ADC_MAX * (VOUT * 2) - VOUT)
#define VtoADC(v) ((int32_t)(((v) / VOUT + 1) * (ADC_MAX / 2)))  

#define ADC_SET_MIN VtoADC(-1.29)
#define ADC_SET_MAX VtoADC(1.29)
#define ADC_SET_ZERO VtoADC(0)
int32_t adcSetpoint = ADC_SET_ZERO;

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
    if (adcSetpoint == -1) {
      COAST();
    } else if (adcSetpoint == VtoADC(0)) {
      BRAKE();
      TRIGGER_ADC();
    } else {
      if (load_open < adcSetpoint) {
        if (adcSetpoint == ADC_SET_MIN) {
          if (load_open < VtoADC(-1.31)) {
            FORWARD();
            TRIGGER_ADC();
          }
        } else {
          FORWARD();
          TRIGGER_ADC();
        }
      } else if (load_open > adcSetpoint) {
        if (adcSetpoint == ADC_SET_MAX) {
          if (load_open > VtoADC(1.31)) {
            REVERSE();
            TRIGGER_ADC();
          }
        } else {
          REVERSE();
          TRIGGER_ADC();
        }
      }
    }
  }
}

uint32_t start = 0;
void loop() {
  while(Serial.available()) {
    char c = Serial.read();
    switch(c) {
      case 'f': adcSetpoint = ADC_SET_MAX; break;
      case 'r': adcSetpoint = ADC_SET_MIN; break;
      case 'b': adcSetpoint = ADC_SET_ZERO; break;
      case 'z': adcSetpoint = -1; break;
      // case 's': start = millis(); TC_Start(TC0, 0); break;
      case 'm': TC0_Handler(); break;
    }
  }

  float i_max = 0.8;
  // float duty = 0.5;
  // float Icharge = fabs(ADCtoV(shunt_high)/1000)*duty;
  // float i_charge_mA = fabs(ADCtoV(shunt_high)) * duty;

  float duty = fminf(i_max * fabs(ADC_MAX/(VOUT * (ADC_MAX - 2*shunt_high))), 0.9);
  update_timers(duty, 1000);
  // if (i_charge_mA > i_max) {
  //   // reduce
  // } else if (Icharge < Imax) {
  //   // increase
  // }
  Serial.println((String)"" + duty + "," + ADCtoV(shunt_high) + "," + ADCtoV(load_open));

  // if (load_open != -1 && shunt_open != -1 && load_high != -1 && shunt_high != -1) {
  //   String highV = ((adcSetpoint == ADC_SET_MAX) ? "3.3" : ((adcSetpoint == ADC_SET_MIN) ? "-3.3" : " "));
  //   Serial.println((String)"" + (millis() - start) + "," + highV +  "," + (adcSetpoint > -1 ? (String)ADCtoV(adcSetpoint) : " ") + "," + ADCtoV(load_open) + "," + ADCtoV(shunt_open) + "," + ADCtoV(load_high) + "," + ADCtoV(shunt_high));
  // }
  delay(1);
}