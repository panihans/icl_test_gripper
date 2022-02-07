#pragma once
#include <Arduino.h>


enum charging_state {high, open_circuit};
extern charging_state st;

#define OUT_A 7
#define OUT_B 6
#define ENABLE_A(mode) pinMode(OUT_A, OUTPUT); digitalWrite(OUT_A, mode);
#define DISABLE_A() pinMode(OUT_A, INPUT);
#define ENABLE_B(mode) pinMode(OUT_B, OUTPUT); digitalWrite(OUT_B, mode);
#define DISABLE_B() pinMode(OUT_B, INPUT);
#define FORWARD() ENABLE_A(HIGH); ENABLE_B(LOW); st = high;
#define REVERSE() ENABLE_A(LOW); ENABLE_B(HIGH); st = high;
#define BRAKE() ENABLE_A(LOW); ENABLE_B(LOW); st = high;
#define COAST() DISABLE_A(); DISABLE_B(); st = open_circuit;


#define TRIGGER_ADC() ADC->ADC_CR = ADC_CR_START;
#define CAN_READ_ADC_4 (ADC->ADC_ISR & ADC_ISR_EOC4)
#define CAN_READ_ADC_6 (ADC->ADC_ISR & ADC_ISR_EOC6)


#define VOUT 3.3f
#define ADC_MAX 4095.f
#define ADCtoV(adc) ((adc) / ADC_MAX * (VOUT * 2) - VOUT)
#define VtoADC(v) ((int32_t)(((v) / VOUT + 1) * (ADC_MAX / 2)))  

#define ADC_SET_MIN VtoADC(-1.2)
#define ADC_SET_MAX VtoADC(1.2)
#define ADC_SET_ZERO VtoADC(0)
#define ADC_SET_HZ -1
extern int32_t adcSetpoint;


void setup_adc();