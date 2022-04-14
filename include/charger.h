#pragma once
#include "Arduino.h"

#define OUT_A 7
#define OUT_B 6
#define ENABLE_A(mode)                                                                                                 \
    pinMode(OUT_A, OUTPUT);                                                                                            \
    digitalWrite(OUT_A, mode);
#define DISABLE_A() pinMode(OUT_A, INPUT);
#define ENABLE_B(mode)                                                                                                 \
    pinMode(OUT_B, OUTPUT);                                                                                            \
    digitalWrite(OUT_B, mode);
#define DISABLE_B() pinMode(OUT_B, INPUT);
#define FORWARD()                                                                                                      \
    ENABLE_A(HIGH);                                                                                                    \
    ENABLE_B(LOW);
#define REVERSE()                                                                                                      \
    ENABLE_A(LOW);                                                                                                     \
    ENABLE_B(HIGH);
#define SHORT_CIRCUIT()                                                                                                \
    ENABLE_A(LOW);                                                                                                     \
    ENABLE_B(LOW);
#define OPEN_CIRCUIT()                                                                                                 \
    DISABLE_A();                                                                                                       \
    DISABLE_B();

enum class charger_status_t
{
    charging,
    open_circuit,
    short_circuit,
    measure_only
};
extern volatile charger_status_t charger_status;

enum class setpoint_type_t
{
    voltage,
    charge
};
extern volatile setpoint_type_t charger_setpoint_type;

extern volatile uint32_t charger_voltage_setpoint;
extern volatile float charger_current_setpoint;