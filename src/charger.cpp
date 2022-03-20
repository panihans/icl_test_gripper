#include "charger.h"

charger_status_t charger_status = charger_status_t::open_circuit;
uint32_t charger_setpoint = 0;