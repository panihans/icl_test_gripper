#include "Arduino.h"
#include "Done.h"
#include "Move.h"
#include "Progress.h"
#include "adc.h"
#include "charger.h"
#include "math.h"
#include "ros.h"
#include "timer.h"

uint32_t charging_finish_ms;
uint32_t charging_finish_load_open;

float v_max = 1.3;
float charge_target = 1;
float charge_speed = 1;

enum class charging_percentage_status_t
{
    maximum,
    calculatable,
    unavailable
};
charging_percentage_status_t charging_percentage_status = charging_percentage_status_t::unavailable;

// ROS
class DUE_Bluetooth : public ArduinoHardware {
public:
    DUE_Bluetooth() : ArduinoHardware(&Serial1, 57600){};
};
ros::NodeHandle_<DUE_Bluetooth> nh;

// topic publisher
tehislihas::Progress progress_msg;
tehislihas::Done done_msg;
ros::Publisher progress("tehislihas/progress", &progress_msg);
ros::Publisher done("tehislihas/done", &done_msg);

// topic subscriber
#define mmin(a, b) (a < b ? a : b)
#define mmax(a, b) (a > b ? a : b)
#define clamp(l, h, val) mmax(l, mmin(h, val))
void begin_charge(uint32_t setpoint);
void begin_short();
void moveCb(const tehislihas::Move &msg) {
    charge_target = msg.setpoint;
    charge_speed = clamp(0.0f, 1.0f, msg.speed);
    progress_msg.setpoint = charge_target;
    progress_msg.speed = charge_speed;
    done_msg.setpoint = charge_target;
    done_msg.speed = charge_speed;
    if (charge_target == 0) {
        begin_short();
    } else {
        begin_charge(V_TO_ADC(clamp(-v_max, v_max, v_max * charge_target)));
    }
}
ros::Subscriber<tehislihas::Move> move("tehislihas/move", &moveCb);

void setup() {
    // put your setup code here, to run once:
    setup_differential_adc_ch4_ch6();
    nh.initNode();

    nh.advertise(progress);
    nh.advertise(done);
    nh.subscribe(move);
    // Serial.begin(115200);
    // Serial.println("begin");
}

void begin_charge(uint32_t setpoint) {
    charger_status = charger_status_t::charging;
    charger_setpoint = setpoint;
    charging_percentage_status = charging_percentage_status_t::unavailable;
    setup_timer0_ch0(1000, 0.1f);
    enable_timer0_ch0();
}

void begin_short() {
    charger_status = charger_status_t::short_circuit;
    charging_percentage_status = charging_percentage_status_t::unavailable;
    setup_timer0_ch0(1000, 0.1f);
    enable_timer0_ch0();
}

void begin_measure() {
    charger_status = charger_status_t::measure_only;
    setup_timer0_ch0(1, 0.1f);
    enable_timer0_ch0();
}

void charger_done() {
    charging_finish_ms = millis();
    charging_percentage_status = charging_percentage_status_t::maximum;
    begin_measure();
}

void TC0_Handler() {
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
            if (load_open >= V_TO_ADC(0.001) || load_open <= V_TO_ADC(-0.001)) {
                SHORT_CIRCUIT();
            } else {
                charger_done();
            }
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

int i = 0;
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
    float shunt_resistance = 1000.f;
    float duty_max = 0.9;
    float max_current = 1.5 * pow(10, -3);
    float current_limit_mA = charge_speed * max_current;
    float voltage_diff = fabs(ADC_TO_V(load_closed)) + fabs(ADC_TO_V(shunt_closed));
    float duty = fmin(current_limit_mA * (shunt_resistance / voltage_diff), duty_max);
    update_timer0_ch0_duty(duty);

    float percent = 0;
    if (charger_status == charger_status_t::charging) {
        if (charge_target * v_max != 0) {
            // percent = ADC_TO_V(load_open) / ADC_TO_V(v_max);
            percent = (ADC_TO_V(load_open) / (charge_target * v_max)) * charge_target;
        }
        // progress_msg.position = percent;
    } else {
        if (charging_percentage_status == charging_percentage_status_t::maximum) {
            uint32_t settling_time_ms = 30 * 1000;
            if (millis() >= charging_finish_ms + settling_time_ms) {
                charging_finish_load_open = load_open;
                charging_percentage_status = charging_percentage_status_t::calculatable;
            }
            percent = charge_target;
            // done_msg.position = percent;
            // done.publish(&progress_msg);
        }
        if (charging_percentage_status == charging_percentage_status_t::calculatable) {
            percent = (ADC_TO_V(load_open) / ADC_TO_V(charging_finish_load_open)) * charge_target;
            // progress_msg.position = percent;
        }
    }

    Serial.print(shunt_closed);
    Serial.print(",");
    Serial.print(load_open);
    Serial.print(",");
    Serial.print(percent);
    // Serial.print((uint32_t)(percent * 100));
    Serial.println();

    if (charger_status == charger_status_t::charging) {
        if (i > 1000) {
            // progress.publish(&progress_msg);
            i = 0;
        } else {
            i++;
        }
    }

    nh.spinOnce();
    delay(1);
}