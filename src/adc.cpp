#include "adc.h"
#include "timer.h"

CircuitMeasure currentMeasurement = CircuitMeasure();

void setup_differential_adc_ch2_ch4_ch6() {
    // setup adc pins A0,A1,A2,A3,A4,A5,A6
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);

    PMC->PMC_PCER1 |= PMC_PCER1_PID37;
    ADC->ADC_CR = ADC_CR_SWRST;
    ADC->ADC_MR |= ADC_MR_ANACH_ALLOWED | ADC_MR_LOWRES_BITS_12 |
                   ADC_MR_STARTUP_SUT64; // separate channels + 12bit + startup delay
    ADC->ADC_COR = ADC_COR_DIFF2 | ADC_COR_DIFF3 | ADC_COR_OFF2 | ADC_COR_OFF3 | ADC_COR_DIFF4 | ADC_COR_DIFF5 |
                   ADC_COR_OFF4 | ADC_COR_OFF5 | ADC_COR_DIFF6 | ADC_COR_DIFF7 | ADC_COR_OFF6 |
                   ADC_COR_OFF7; // enable differential for channels and 0.5 offsets
    ADC->ADC_CHER |= ADC_CHER_CH2 | ADC_CHER_CH3 | ADC_CHER_CH4 | ADC_CHER_CH5 | ADC_CHER_CH6 | ADC_CHER_CH7;

    ADC->ADC_IER |= ADC_IER_EOC2 | ADC_IER_EOC4 | ADC_IER_EOC6; // enable interrupts
    NVIC_EnableIRQ(ADC_IRQn);
}