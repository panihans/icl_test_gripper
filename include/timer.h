#pragma once
#include "Arduino.h"

#define RA(duty, RC) (duty * RC)
#define RC(freq) (84000000 / 2 / freq)

enum class pwm_status_t
{
    closed_circuit,
    open_circuit
};
extern volatile pwm_status_t pwm_status;

void setup_timer0_ch0(uint32_t frequency, float duty);
void enable_timer0_ch0();
void disable_timer0_ch0();
void update_timer0_ch0_duty(float duty);