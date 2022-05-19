#pragma once
#include "Arduino.h"

#define OUT_A 7
#define OUT_B 6
#define OUT_C 5
#define OUT_D 4

#define ENABLE_PIN(pin, mode)                                                                                          \
    pinMode(pin, OUTPUT);                                                                                              \
    digitalWrite(pin, mode);
#define DISABLE_PIN(pin) pinMode(pin, INPUT);

void open_circuit();
void charge_to_forward();
void short_from_forward();
void charge_to_backward();
void short_from_backward();

enum class charger_status_t
{
    charging,
    open_circuit,
    short_circuit,
    measure_only
};
extern volatile charger_status_t charger_status;
extern volatile uint32_t charger_voltage_setpoint;