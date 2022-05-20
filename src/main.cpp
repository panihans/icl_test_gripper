#include "ActuatorFeedback.h"
#include "Arduino.h"
#include "ChargeCommand.h"
#include "MeasureCommand.h"
#include "ResetCommand.h"
#include "ShortCommand.h"
#include "StopCommand.h"
#include "adc.h"
#include "charger.h"
#include "math.h"
#include "ros.h"
#include "timer.h"

// ROS use Serial1 
class DUE_Bluetooth : public ArduinoHardware {
public:
    DUE_Bluetooth() : ArduinoHardware(&Serial1, 57600){};
};
ros::NodeHandle_<DUE_Bluetooth> nh;

// topic publisher
am_soft_grip_msgs::ActuatorFeedback feedback_msg;
ros::Publisher feedback_publisher("/am_act/feedback", &feedback_msg);

void charge_callback(const am_soft_grip_msgs::ChargeCommand &msg);
void measure_callback(const am_soft_grip_msgs::MeasureCommand &msg);
void short_callback(const am_soft_grip_msgs::ShortCommand &msg);
void stop_callback(const am_soft_grip_msgs::StopCommand &msg);
void reset_callback(const am_soft_grip_msgs::ResetCommand &msg);

ros::Subscriber<am_soft_grip_msgs::ChargeCommand> charge_subscriber("/am_act/charge", &charge_callback);
ros::Subscriber<am_soft_grip_msgs::MeasureCommand> measure_subscriber("/am_act/measure", &measure_callback);
ros::Subscriber<am_soft_grip_msgs::ShortCommand> short_subscriber("/am_act/short", &short_callback);
ros::Subscriber<am_soft_grip_msgs::StopCommand> stop_subscriber("/am_act/stop", &stop_callback);
ros::Subscriber<am_soft_grip_msgs::ResetCommand> reset_subscriber("/am_act/reset", &reset_callback);

void setup() {
    // setup adc
    setup_differential_adc_ch2_ch4_ch6();
    // init ros node
    nh.initNode();

    // publisher
    nh.advertise(feedback_publisher);
    // subscriber
    nh.subscribe(charge_subscriber);
    nh.subscribe(measure_subscriber);
    nh.subscribe(short_subscriber);
    nh.subscribe(stop_subscriber);
    nh.subscribe(reset_subscriber);

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
            open_circuit();
        }
        if (status & TC_SR_CPCS) {
            pwm_status = pwm_status_t::closed_circuit;
            if ((currentMeasurement.load.open) < charger_voltage_setpoint - (V_TO_ADC(0.01) - V_TO_ADC(0))) {
                charge_to_forward();
            } else if ((currentMeasurement.load.open) > charger_voltage_setpoint + (V_TO_ADC(0.01) - V_TO_ADC(0))) {
                charge_to_backward();
            } else {
                charger_done();
            }
        }
    } else if (charger_status == charger_status_t::short_circuit) {
        pwm_status = pwm_status_t::closed_circuit;
        if (ADC_TO_V(currentMeasurement.load.open) > 0) {
            short_from_forward();
        } else {
            short_from_backward();
        }
    } else if (charger_status == charger_status_t::measure_only) {
        pwm_status = pwm_status_t::open_circuit;
        open_circuit();
    } else if (charger_status == charger_status_t::open_circuit) {
        pwm_status = pwm_status_t::open_circuit;
        open_circuit();
    }
    TRIGGER_ADC();
}

void ADC_Handler() {
    uint32_t status = ADC->ADC_ISR;
    if (status & ADC_ISR_EOC2) {
        if (pwm_status == pwm_status_t::open_circuit) {
            currentMeasurement.shunt2.open = ADC->ADC_CDR[2];
        } else {
            currentMeasurement.shunt2.closed = ADC->ADC_CDR[2];
            charge_accumulator += ADC_TO_V(currentMeasurement.shunt2.closed) * duty;
        }
    }
    if (status & ADC_ISR_EOC4) {
        if (pwm_status == pwm_status_t::open_circuit) {
            currentMeasurement.load.open = ADC->ADC_CDR[4];
        } else {
            currentMeasurement.load.closed = ADC->ADC_CDR[4];
        }
    }
    if (status & ADC_ISR_EOC6) {
        if (pwm_status == pwm_status_t::open_circuit) {
            currentMeasurement.shunt1.open = ADC->ADC_CDR[6];
        } else {
            currentMeasurement.shunt1.closed = ADC->ADC_CDR[6];
            charge_accumulator += ADC_TO_V(currentMeasurement.shunt1.closed) * duty;
        }
    }
}

#define mmin(a, b) (a < b ? a : b)
#define mmax(a, b) (a > b ? a : b)
#define clamp(l, h, val) mmax(l, mmin(h, val))
float current_limit_A = 0;
ros::Time last_cmd_received;
void charge_callback(const am_soft_grip_msgs::ChargeCommand &msg) {
    // rosserial charge command callback
    charger_done();
    last_cmd_received = msg.sent;
    current_limit_A = msg.icl_current_limit;
    charger_status = charger_status_t::charging;
    charger_voltage_setpoint = V_TO_ADC(clamp(V_MIN, V_MAX, msg.icl_target_voltage));
    duty = DUTY_MAX;
    setup_timer0_ch0(1000, duty);
    enable_timer0_ch0();
}

void measure_callback(const am_soft_grip_msgs::MeasureCommand &msg) {
    // rosserial measure command callback
    charger_done();
    last_cmd_received = msg.sent;
    charger_status = charger_status_t::measure_only;
    duty = DUTY_100;
    setup_timer0_ch0(1000, duty);
    enable_timer0_ch0();
}

void short_callback(const am_soft_grip_msgs::ShortCommand &msg) {
    // rosserial short circuit command callback
    charger_done();
    last_cmd_received = msg.sent;
    charger_status = charger_status_t::short_circuit;
    duty = DUTY_100;
    setup_timer0_ch0(1000, duty);
    enable_timer0_ch0();
}

void stop_callback(const am_soft_grip_msgs::StopCommand &msg) {
    // rosserial stop command callback
    charger_done();
    last_cmd_received = msg.sent;
}

void reset_callback(const am_soft_grip_msgs::ResetCommand &msg) {
    // rosserial reset command callback
    last_cmd_received = msg.sent;
    charge_accumulator = 0;
}

int i = 0;
void loop() {
    // update pwm when charging
    if (charger_status == charger_status_t::charging) {
        // calculate pwm duty
        float shunt_resistance = 1000.f;
        float shunt_v_abs = fabs(ADC_TO_V(currentMeasurement.shunt1.closed) + ADC_TO_V(currentMeasurement.shunt2.closed));
        duty = clamp(DUTY_MIN, DUTY_MAX, current_limit_A * (shunt_resistance / shunt_v_abs));
        update_timer0_ch0_duty(duty);
    }

    // send feedback about every ~100ms
    if (millis() - i >= 100) {
        i = millis();
        feedback_msg.icl_charging_voltage = ADC_TO_V(currentMeasurement.load.closed);
        feedback_msg.icl_open_circuit_voltage = ADC_TO_V(currentMeasurement.load.open);
        feedback_msg.icl_coulomb_counter = charge_accumulator;
        feedback_msg.actuator_state = (uint8_t)charger_status;
        feedback_msg.last_command = last_cmd_received;
        feedback_publisher.publish(&feedback_msg);
    }

    // update node
    nh.spinOnce();
}