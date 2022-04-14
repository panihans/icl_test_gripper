#include "adc.h"
#include "timer.h"

volatile uint32_t load_open = 0;
volatile uint32_t shunt_open = 0;
volatile uint32_t load_closed = 0;
volatile uint32_t shunt_closed = 0;

void setup_differential_adc_ch4_ch6() {
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);

    PMC->PMC_PCER1 |= PMC_PCER1_PID37;
    ADC->ADC_CR = ADC_CR_SWRST;
    ADC->ADC_MR |= ADC_MR_ANACH_ALLOWED | ADC_MR_LOWRES_BITS_12 |
                   ADC_MR_STARTUP_SUT64; // separate channels + 12bit + startup delay
    ADC->ADC_COR = ADC_COR_DIFF4 | ADC_COR_DIFF5 | ADC_COR_OFF4 | ADC_COR_OFF5 | ADC_COR_DIFF6 | ADC_COR_DIFF7 |
                   ADC_COR_OFF6 | ADC_COR_OFF7; // enable differential for channels and 0.5 offsets
    ADC->ADC_CHER |= ADC_CHER_CH4 | ADC_CHER_CH5 | ADC_CHER_CH6 | ADC_CHER_CH7;

    ADC->ADC_IER |= ADC_IER_EOC4 | ADC_IER_EOC6; // enable interrupts
    NVIC_EnableIRQ(ADC_IRQn);
}