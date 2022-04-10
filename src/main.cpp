#include "Arduino.h"
#include "ChargeCommand.h"
#include "ChargerStatus.h"
#include "MeasureCommand.h"
#include "ShortcircuitCommand.h"
#include "StopCommand.h"
#include "adc.h"
#include "charger.h"
#include "math.h"
#include "ros.h"
#include "timer.h"

// uint32_t charging_finish_ms;
// uint32_t charging_finish_load_open;

// float v_max = 1.3;
// float charge_target = 1;
float charge_speed = 1;

// ROS
class DUE_Bluetooth : public ArduinoHardware {
public:
    DUE_Bluetooth() : ArduinoHardware(&Serial, 57600){};
};
ros::NodeHandle_<DUE_Bluetooth> nh;

// topic publisher
tehislihas::ChargeCommand charge_command_msg;
tehislihas::ChargerStatus charger_status_msg;
tehislihas::MeasureCommand measure_command_msg;
tehislihas::ShortcircuitCommand shortcircuit_command_msg;
tehislihas::StopCommand stop_command_msg;
ros::Publisher charger_status_publisher("tehislihas/charger", &charger_status_msg);

void charge_command_callback(const tehislihas::ChargeCommand &msg);
void measure_command_callback(const tehislihas::MeasureCommand &msg);
void shortcircuit_command_callback(const tehislihas::ShortcircuitCommand &msg);
void stop_command_callback(const tehislihas::StopCommand &msg);

ros::Subscriber<tehislihas::ChargeCommand> charge_command_subscriber("tehislihas/charge",
                                                                     &charge_command_callback);
ros::Subscriber<tehislihas::MeasureCommand> measure_command_subscriber("tehislihas/measure",
                                                                       &measure_command_callback);
ros::Subscriber<tehislihas::ShortcircuitCommand> shortcircuit_command_subscriber("tehislihas/short",
                                                                                 &shortcircuit_command_callback);
ros::Subscriber<tehislihas::StopCommand> stop_command_subscriber("tehislihas/stop", &stop_command_callback);

void setup() {
    // put your setup code here, to run once:
    setup_differential_adc_ch4_ch6();
    nh.initNode();

    nh.advertise(charger_status_publisher);
    nh.subscribe(charge_command_subscriber);
    nh.subscribe(measure_command_subscriber);
    nh.subscribe(shortcircuit_command_subscriber);
    nh.subscribe(stop_command_subscriber);
    // Serial.begin(115200);
    // Serial.println("begin");
}

void begin_charge(uint32_t setpoint) {
    charger_status = charger_status_t::charging;
    charger_setpoint = setpoint;
    setup_timer0_ch0(1000, 0.1f);
    enable_timer0_ch0();
}

void begin_short() {
    charger_status = charger_status_t::short_circuit;
    disable_timer0_ch0();
    pwm_status = pwm_status_t::closed_circuit;
    SHORT_CIRCUIT();
    // setup_timer0_ch0(1000, 0.1f);
    // enable_timer0_ch0();
}

void begin_measure() {
    charger_status = charger_status_t::measure_only;
    setup_timer0_ch0(1, 0.1f);
    enable_timer0_ch0();
}

void charger_done() {
    charger_status = charger_status_t::open_circuit;
    disable_timer0_ch0();
}

void TC0_Handler() {
    // changes charging direction
    // triggers adc
    int status = TC0->TC_CHANNEL[0].TC_SR;
    if (status & TC_SR_CPAS) {
        pwm_status = pwm_status_t::open_circuit;
        OPEN_CIRCUIT();
    }
    if (status & TC_SR_CPCS) {
        pwm_status = pwm_status_t::closed_circuit;
        if (charger_status == charger_status_t::charging) {
            if (load_open < charger_setpoint) {
                if (charger_setpoint > V_TO_ADC(0)) {
                    FORWARD();
                } else {
                    charger_done();
                }
            } else {
                if (charger_setpoint < V_TO_ADC(0)) {
                    REVERSE();
                } else {
                    charger_done();
                }
            }
        } else if (charger_status == charger_status_t::short_circuit) {
            // if (load_open >= V_TO_ADC(0.001) || load_open <= V_TO_ADC(-0.001)) {
            //     SHORT_CIRCUIT();
            // } else {
            //     charger_done();
            // }
        } else if (charger_status == charger_status_t::measure_only) {
            // nothing to do here
        }
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
        }
    }
}

#define mmin(a, b) (a < b ? a : b)
#define mmax(a, b) (a > b ? a : b)
#define clamp(l, h, val) mmax(l, mmin(h, val))
float current_limit_A = 0;
void charge_command_callback(const tehislihas::ChargeCommand &msg) {
    current_limit_A = msg.current_limit_A;
    begin_charge(V_TO_ADC(clamp(V_MIN, V_MAX, msg.setpoint)));
}

void measure_command_callback(const tehislihas::MeasureCommand &msg) {
    begin_measure();
}

void shortcircuit_command_callback(const tehislihas::ShortcircuitCommand &msg) {
    begin_short();
}

void stop_command_callback(const tehislihas::StopCommand &msg) {
    charger_done();
}

void loop() {
    // put your main code here, to run repeatedly:

    // float charge_target = 1;
    //  float v_max = 1.3;
    // while (Serial.available()) {
    //     char c = Serial.read();
    //     switch (c) {
    //     case 'f': {
    //         charge_target = 1;
    //         begin_charge(V_TO_ADC(v_max * charge_target));
    //         break;
    //     }
    //     case 'r': {
    //         charge_target = -1;
    //         begin_charge(V_TO_ADC(v_max * charge_target));
    //         break;
    //     }
    //     case 'b': {
    //         begin_short();
    //         break;
    //     }
    //     case 's': {
    //         charger_done();
    //         break;
    //     }
    //     case 'm': {
    //         begin_measure();
    //         break;
    //     }
    //     }
    // }
    // Serial.print(shunt_closed);
    // Serial.print(",");
    // Serial.print(load_open);
    // Serial.print(",");
    // Serial.print(percent);
    // Serial.print((uint32_t)(percent * 100));
    // Serial.println();

    if (charger_status == charger_status_t::charging) {
        // calculate timer 
        float shunt_resistance = 1000.f;
        float duty_max = 0.9;
        float voltage_diff = fabs(ADC_TO_V(load_closed)) + fabs(ADC_TO_V(shunt_closed));
        float duty = fmin(current_limit_A * (shunt_resistance / voltage_diff), duty_max);
        update_timer0_ch0_duty(duty);
    } else if (charger_status == charger_status_t::short_circuit) {
        // trigger adc without pwm
        TRIGGER_ADC();
    }

    if (charger_status != charger_status_t::open_circuit) {
        // update if not in open circuit mode
        charger_status_msg.shunt_closed = shunt_closed;
        charger_status_msg.shunt_open = shunt_open;
        charger_status_msg.load_closed = load_closed;
        charger_status_msg.load_open = load_open;
        charger_status_msg.charger_status_code = (uint16_t)charger_status;
        charger_status_publisher.publish(&charger_status_msg);
    }

    nh.spinOnce();
    delay(10);
}