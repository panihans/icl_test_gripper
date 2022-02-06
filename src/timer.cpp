#include "timer.h"

void setup_timer(Tc* timer, uint32_t channel, uint32_t compa, uint32_t compc) {
  timer->TC_CHANNEL[channel].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE;
  timer->TC_CHANNEL[channel].TC_RA = compa;
  timer->TC_CHANNEL[channel].TC_RC = compc;
  timer->TC_CHANNEL[channel].TC_IER = TC_IER_CPAS | TC_IER_CPCS;
  timer->TC_CHANNEL[channel].TC_CV = 0;
}

void setup_timers(float duty, int frequency) {
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;
  setup_timer(TC0, 0, RA(duty, RC(frequency)), RC(frequency));
  NVIC_EnableIRQ(TC0_IRQn);
  TC_Start(TC0, 0);
}

void update_timers(float duty, int frequency) {
  // TC_Stop(TC0, 0);
  TC0->TC_CHANNEL[0].TC_RA = RA(duty, RC(frequency));
  TC0->TC_CHANNEL[0].TC_RC = RC(frequency);
  if (TC0->TC_CHANNEL[0].TC_CV > TC0->TC_CHANNEL[0].TC_RC) {
    TC0->TC_CHANNEL[0].TC_CV = 0;
  }
  // TC_Start(TC0, 0);
}