#pragma once
#include <Arduino.h>

#define RA(duty, RC) (duty*RC)
#define RC(freq) (84000000/2/freq)

void setup_timer(Tc* timer, uint32_t channel, uint32_t compa, uint32_t compc);
void setup_timers(float duty, int frequency);
void update_timers(float duty, int frequency);