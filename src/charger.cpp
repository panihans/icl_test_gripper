#include "charger.h"

volatile charger_status_t charger_status = charger_status_t::open_circuit;
volatile uint32_t charger_voltage_setpoint = 0;

void open_circuit() {
    // open circuit mode
    DISABLE_PIN(OUT_A);
    DISABLE_PIN(OUT_B);
    DISABLE_PIN(OUT_C);
    DISABLE_PIN(OUT_D);
}

void charge_to_forward() {
    // charge to 'forward' direction
    DISABLE_PIN(OUT_A);
    ENABLE_PIN(OUT_B, HIGH);
    DISABLE_PIN(OUT_C);
    ENABLE_PIN(OUT_D, LOW);
}

void short_from_forward() {
    // short from 'forward' direction
    ENABLE_PIN(OUT_A, LOW);
    DISABLE_PIN(OUT_B);
    ENABLE_PIN(OUT_C, LOW);
    DISABLE_PIN(OUT_D);
}

void charge_to_backward() {
    // charge to 'backward' direction
    ENABLE_PIN(OUT_A, LOW);
    DISABLE_PIN(OUT_B);
    ENABLE_PIN(OUT_C, HIGH);
    DISABLE_PIN(OUT_D);
}

void short_from_backward() {
    // short from 'backward' direction
    DISABLE_PIN(OUT_A);
    ENABLE_PIN(OUT_B, LOW);
    DISABLE_PIN(OUT_C);
    ENABLE_PIN(OUT_D, LOW);
}