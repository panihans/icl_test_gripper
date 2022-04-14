#include "charger.h"

volatile charger_status_t charger_status = charger_status_t::open_circuit;
volatile setpoint_type_t charger_setpoint_type = setpoint_type_t::voltage;
volatile uint32_t charger_voltage_setpoint = 0;
volatile float charger_current_setpoint = 0;