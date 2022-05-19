#pragma once
#include "Arduino.h"

#define CONVERT_RANGE(v, omin, omax, nmin, nmax) ((((v - omin) * (nmax - nmin)) / (omax - omin)) + nmin)
#define ADC_TO_V(rdg) CONVERT_RANGE(rdg, 0.f, 4095.f, -3.3f, 3.3f)
#define V_TO_ADC(v) CONVERT_RANGE(v, -3.3f, 3.3f, 0.f, 4095.f)
#define TRIGGER_ADC() ADC->ADC_CR = ADC_CR_START;
#define V_MAX 1.3
#define V_MIN -1.3


struct ADCMeasure {
    volatile uint32_t open;
    volatile uint32_t closed;
};

struct CircuitMeasure {
    ADCMeasure shunt1;
    ADCMeasure shunt2;
    ADCMeasure load;
};
extern CircuitMeasure currentMeasurement;

void setup_differential_adc_ch2_ch4_ch6();