#include "charger.h"

volatile charger_status_t charger_status = charger_status_t::open_circuit;
volatile uint32_t charger_voltage_setpoint = 0;

void open_circuit() {
    DISABLE_PIN(OUT_A);
    DISABLE_PIN(OUT_B);
    DISABLE_PIN(OUT_C);
    DISABLE_PIN(OUT_D);
}

void charge_to_forward() {
    DISABLE_PIN(OUT_A);
    ENABLE_PIN(OUT_B, HIGH);
    DISABLE_PIN(OUT_C);
    ENABLE_PIN(OUT_D, LOW);
}

void short_from_forward() {
    ENABLE_PIN(OUT_A, LOW);
    DISABLE_PIN(OUT_B);
    ENABLE_PIN(OUT_C, LOW);
    DISABLE_PIN(OUT_D);
}

void charge_to_backward() {
    ENABLE_PIN(OUT_A, LOW);
    DISABLE_PIN(OUT_B);
    ENABLE_PIN(OUT_C, HIGH);
    DISABLE_PIN(OUT_D);
}

void short_from_backward() {
    DISABLE_PIN(OUT_A);
    ENABLE_PIN(OUT_B, LOW);
    DISABLE_PIN(OUT_C);
    ENABLE_PIN(OUT_D, LOW);
}