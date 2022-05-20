#include "timer.h"

volatile pwm_status_t pwm_status = pwm_status_t::open_circuit;

void setup_timer(Tc *timer, uint32_t channel, uint32_t compa, uint32_t compc) {
    timer->TC_CHANNEL[channel].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE;
    timer->TC_CHANNEL[channel].TC_RA = compa;
    timer->TC_CHANNEL[channel].TC_RC = compc;
    timer->TC_CHANNEL[channel].TC_IER = TC_IER_CPAS | TC_IER_CPCS;
    timer->TC_CHANNEL[channel].TC_CV = 0;
}

uint32_t current_frequency = 0;
void setup_timer0_ch0(uint32_t frequency, float duty) {
    // setup timer with given frequency and duty
    current_frequency = frequency;
    PMC->PMC_PCER0 |= PMC_PCER0_PID27;
    setup_timer(TC0, 0, RA(duty, RC(frequency)), RC(frequency));
    NVIC_EnableIRQ(TC0_IRQn);
    TC_Start(TC0, 0);
}

void enable_timer0_ch0() {
    TC_Start(TC0, 0);
}

void disable_timer0_ch0() {
    TC_Stop(TC0, 0);
}

void update_timer0_ch0_duty(float duty) {
    // update timer duty
    TC0->TC_CHANNEL[0].TC_RA = RA(duty, RC(current_frequency));
}