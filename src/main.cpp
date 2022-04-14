#include "Arduino.h"
#include "Charge2Command.h"
#include "ChargeCommand.h"
#include "ChargerStatus.h"
#include "MeasureCommand.h"
#include "ResetCommand.h"
#include "ShortcircuitCommand.h"
#include "StopCommand.h"
#include "adc.h"
#include "charger.h"
#include "math.h"
#include "ros.h"
#include "timer.h"

// ROS
class DUE_Bluetooth : public ArduinoHardware {
public:
    DUE_Bluetooth() : ArduinoHardware(&Serial, 57600){};
};
ros::NodeHandle_<DUE_Bluetooth> nh;

// topic publisher
tehislihas::ChargerStatus charger_status_msg;
ros::Publisher charger_status_publisher("tehislihas/charger", &charger_status_msg);

void charge_command_callback(const tehislihas::ChargeCommand &msg);
void charge2_command_callback(const tehislihas::Charge2Command &msg);
void measure_command_callback(const tehislihas::MeasureCommand &msg);
void shortcircuit_command_callback(const tehislihas::ShortcircuitCommand &msg);
void stop_command_callback(const tehislihas::StopCommand &msg);
void reset_command_callback(const tehislihas::ResetCommand &msg);

ros::Subscriber<tehislihas::ChargeCommand> charge_command_subscriber("tehislihas/charge", &charge_command_callback);
ros::Subscriber<tehislihas::Charge2Command> charge2_command_subscriber("tehislihas/charge2", &charge2_command_callback);
ros::Subscriber<tehislihas::MeasureCommand> measure_command_subscriber("tehislihas/measure", &measure_command_callback);
ros::Subscriber<tehislihas::ShortcircuitCommand> shortcircuit_command_subscriber("tehislihas/short",
                                                                                 &shortcircuit_command_callback);
ros::Subscriber<tehislihas::StopCommand> stop_command_subscriber("tehislihas/stop", &stop_command_callback);
ros::Subscriber<tehislihas::ResetCommand> reset_command_subscriber("tehislihas/reset", &reset_command_callback);

void setup() {
    // put your setup code here, to run once:
    setup_differential_adc_ch4_ch6();
    nh.initNode();

    // publisher
    nh.advertise(charger_status_publisher);
    // subscriber
    nh.subscribe(charge_command_subscriber);
    nh.subscribe(charge2_command_subscriber);
    nh.subscribe(measure_command_subscriber);
    nh.subscribe(shortcircuit_command_subscriber);
    nh.subscribe(stop_command_subscriber);
    nh.subscribe(reset_command_subscriber);

    nh.negotiateTopics();
}

float duty = 0.0f;
#define DUTY_100 1
#define DUTY_MAX 0.9f
#define DUTY_MIN 0.1f

void charger_done() {
    disable_timer0_ch0();
    charger_status = charger_status_t::open_circuit;
}

volatile float charge_accumulator = 0;
void TC0_Handler() {
    // changes charging direction
    // triggers adc
    int status = TC0->TC_CHANNEL[0].TC_SR;
    if (charger_status == charger_status_t::charging) {
        if (status & TC_SR_CPAS) {
            pwm_status = pwm_status_t::open_circuit;
            OPEN_CIRCUIT();
        }
        if (status & TC_SR_CPCS) {
            pwm_status = pwm_status_t::closed_circuit;
            if (charger_setpoint_type == setpoint_type_t::voltage) {
                if (load_open < charger_voltage_setpoint - (V_TO_ADC(0.01) - V_TO_ADC(0))) {
                    FORWARD();
                } else if (load_open > charger_voltage_setpoint + (V_TO_ADC(0.01) - V_TO_ADC(0))) {
                    REVERSE();
                } else {
                    charger_done();
                }
            } else {
                if (charge_accumulator < (charger_current_setpoint - 20)) {
                    FORWARD();
                } else if (charge_accumulator > (charger_current_setpoint + 20)) {
                    REVERSE();
                } else {
                    charger_done();
                }
            }
        }
    } else if (charger_status == charger_status_t::short_circuit) {
        pwm_status = pwm_status_t::closed_circuit;
        SHORT_CIRCUIT();
    } else if (charger_status == charger_status_t::measure_only) {
        pwm_status = pwm_status_t::open_circuit;
        OPEN_CIRCUIT();
    } else if (charger_status == charger_status_t::open_circuit) {
        pwm_status = pwm_status_t::open_circuit;
        OPEN_CIRCUIT();
    }
    TRIGGER_ADC();
}

void ADC_Handler() {
    uint32_t status = ADC->ADC_ISR;
    if (status & ADC_ISR_EOC4) {
        if (pwm_status == pwm_status_t::open_circuit) {
            load_open = ADC->ADC_CDR[4];
        } else {
            load_closed = ADC->ADC_CDR[4];
        }
    }
    if (status & ADC_ISR_EOC6) {
        if (pwm_status == pwm_status_t::open_circuit) {
            shunt_open = ADC->ADC_CDR[6];
        } else {
            shunt_closed = ADC->ADC_CDR[6];
            charge_accumulator += ADC_TO_V(shunt_closed) * duty;
        }
    }
}

#define mmin(a, b) (a < b ? a : b)
#define mmax(a, b) (a > b ? a : b)
#define clamp(l, h, val) mmax(l, mmin(h, val))
float current_limit_A = 0;
void charge_command_callback(const tehislihas::ChargeCommand &msg) {
    charger_done();
    charger_setpoint_type = setpoint_type_t::voltage;
    current_limit_A = msg.current_limit_A;
    charger_status = charger_status_t::charging;
    charger_voltage_setpoint = V_TO_ADC(clamp(V_MIN, V_MAX, msg.setpoint));
    duty = DUTY_MAX;
    setup_timer0_ch0(1000, duty);
    enable_timer0_ch0();
}

void charge2_command_callback(const tehislihas::Charge2Command &msg) {
    charger_done();
    charger_setpoint_type = setpoint_type_t::charge;
    current_limit_A = msg.current_limit_A;
    charger_status = charger_status_t::charging;
    charger_current_setpoint = msg.setpoint;
    duty = DUTY_MAX;
    setup_timer0_ch0(1000, duty);
    enable_timer0_ch0();
}

void measure_command_callback(const tehislihas::MeasureCommand &msg) {
    charger_done();
    charger_status = charger_status_t::measure_only;
    duty = DUTY_100;
    setup_timer0_ch0(1, duty);
    enable_timer0_ch0();
}

void shortcircuit_command_callback(const tehislihas::ShortcircuitCommand &msg) {
    charger_done();
    charger_status = charger_status_t::short_circuit;
    duty = DUTY_100;
    setup_timer0_ch0(1000, duty);
    enable_timer0_ch0();
}

void stop_command_callback(const tehislihas::StopCommand &msg) {
    charger_done();
}

void reset_command_callback(const tehislihas::ResetCommand &msg) {
    charge_accumulator = 0;
}

int i = 0;
void loop() {
    // put your main code here, to run repeatedly:
    if (charger_status == charger_status_t::charging) {
        // calculate pwm duty
        float shunt_resistance = 1000.f;
        float voltage_diff = fabs(ADC_TO_V(load_closed)) + fabs(ADC_TO_V(shunt_closed));
        duty = clamp(DUTY_MIN, DUTY_MAX, current_limit_A * (shunt_resistance / voltage_diff));
        update_timer0_ch0_duty(duty);
    }

    if (i >= 100) {
        i = 0;
        charger_status_msg.shunt_closed = shunt_closed;
        charger_status_msg.shunt_open = shunt_open;
        charger_status_msg.load_closed = load_closed;
        charger_status_msg.load_open = load_open;
        charger_status_msg.charger_status_code = (uint16_t)charger_status;
        charger_status_msg.charge_accumulator = charge_accumulator;
        charger_status_publisher.publish(&charger_status_msg);
    } else {
        i++;
    }

    nh.spinOnce();
    delay(1);
}