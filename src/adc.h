#pragma once
#include "Arduino.h"

#define CONVERT_RANGE(v, omin, omax, nmin, nmax) ((((v - omin) * (nmax - nmin)) / (omax - omin)) + nmin)
#define ADC_TO_V(rdg) CONVERT_RANGE(rdg, 0.f, 4095.f, -3.3f, 3.3f)
#define V_TO_ADC(v) CONVERT_RANGE(v, -3.3f, 3.3f, 0.f, 4095.f)
#define TRIGGER_ADC() ADC->ADC_CR = ADC_CR_START;

extern uint32_t load_open;
extern uint32_t shunt_open;
extern uint32_t load_closed;
extern uint32_t shunt_closed;

void setup_differential_adc_ch4_ch6();