#include <Arduino.h>

#define OUT_A 7
#define OUT_B 6
#define ENABLE_A(mode) pinMode(OUT_A, OUTPUT); digitalWrite(OUT_A, mode);
#define DISABLE_A() pinMode(OUT_A, INPUT);
#define ENABLE_B(mode) pinMode(OUT_B, OUTPUT); digitalWrite(OUT_B, mode);
#define DISABLE_B() pinMode(OUT_B, INPUT);
#define FORWARD() ENABLE_A(HIGH); ENABLE_B(LOW);
#define REVERSE() ENABLE_A(LOW); ENABLE_B(HIGH);
#define BRAKE() ENABLE_A(LOW); ENABLE_B(LOW);
#define COAST() DISABLE_A(); DISABLE_B();

#define TRIGGER_ADC() ADC->ADC_CR = ADC_CR_START;

void setup_adc() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_CR = ADC_CR_SWRST;
  ADC->ADC_MR |= ADC_MR_ANACH_ALLOWED | ADC_MR_LOWRES_BITS_12 | ADC_MR_STARTUP_SUT64; // separate channels + 12bit + delay for startup for enable->disable->enable...
  ADC->ADC_COR = ADC_COR_DIFF4 | ADC_COR_DIFF5 | ADC_COR_OFF4 | ADC_COR_OFF5 | 
                  ADC_COR_DIFF6 | ADC_COR_DIFF7 | ADC_COR_OFF6 | ADC_COR_OFF7; // enable differential for channels and 0.5 offsets
  ADC->ADC_CHER |= ADC_CHER_CH4 | ADC_CHER_CH5 | ADC_CHER_CH6 | ADC_CHER_CH7;

  ADC->ADC_IER |= ADC_IER_EOC4 | ADC_IER_EOC6; // enable interrupts
  NVIC_EnableIRQ(ADC_IRQn);
}

void setup_timer() {
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;

  TC_Configure(TC0, 0, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVSEL_UP | TC_CMR_WAVE);
  TC_SetRC(TC0, 0, 42000); //1ms timer
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
}

void setup() {
  // put your setup code here, to run once:
  setup_adc();
  setup_timer();
  Serial.begin(115200);
  Serial.println("begin");
  Serial.println((String)"millisekund, pinge, tehislihas avatud ahela pinge, shunt avatud ahela pinge, tehislihas laadimispinge, shunt laadimispinge");
}

#define CAN_READ_ADC_4 (ADC->ADC_ISR & ADC_ISR_EOC4)
#define CAN_READ_ADC_6 (ADC->ADC_ISR & ADC_ISR_EOC6)

void get_adc_readings(uint32_t& load, uint32_t& shunt) {
  while(!CAN_READ_ADC_4) asm volatile("NOP");
  load = ADC->ADC_CDR[4];
  while(!CAN_READ_ADC_6) asm volatile("NOP");
  shunt = ADC->ADC_CDR[6];
}

#define VOUT 3.3f
#define ADC_MAX 4095.f
#define ADCtoV(adc) ((adc) / ADC_MAX * (VOUT * 2) - VOUT)
#define VtoADC(v) ((int32_t)(((v) / VOUT + 1) * (ADC_MAX / 2)))

#define ADC_SET_MIN VtoADC(-1.29);
#define ADC_SET_MAX VtoADC(1.29);
#define ADC_SET_ZERO VtoADC(0);
int32_t adcSetpoint = ADC_SET_ZERO;

// void loop() {
//   // put your main code here, to run repeatedly:
//   while(Serial.available()) {
//     char c = Serial.read();
//     switch(c) {
//       case 'f': adcSetpoint = ADC_SET_MAX; break;
//       case 'r': adcSetpoint = ADC_SET_MIN; break;
//       case 'b': adcSetpoint = ADC_SET_ZERO; break;
//     }
//   }
//   COAST();
//   TRIGGER_ADC();
//   uint32_t load_open = -1;
//   uint32_t shunt_open = -1;
//   get_adc_readings(load_open, shunt_open);

//   if (adcSetpoint == VtoADC(0)) {
//     BRAKE();
//   } else if (load_open < adcSetpoint) {
//     FORWARD();
//   } else if (load_open > adcSetpoint) {
//     REVERSE();
//   }
//   TRIGGER_ADC();
//   uint32_t load_high = -1;
//   uint32_t shunt_high = -1;
//   get_adc_readings(load_high, shunt_high);
//   delay(1);
//   COAST();

//   Serial.println((String)"" + load_open + "," + shunt_open + "," + load_high + "," + shunt_high);
//   delay(1000);
// }


enum charging_state {high, open_circuit};
charging_state st = open_circuit;


#define CLEAR_TC0_CH0_ISR() TC0->TC_CHANNEL[0].TC_SR
void TC0_Handler() {
  CLEAR_TC0_CH0_ISR();
  COAST();
  st = open_circuit;
  TRIGGER_ADC();
}

int32_t load_open = -1;
int32_t shunt_open = -1;
int32_t load_high = -1;
int32_t shunt_high = -1;
void ADC_Handler() {
  if (CAN_READ_ADC_4) {
    if (st == open_circuit) {
      load_open = ADC->ADC_CDR[4];
    } else {
      load_high = ADC->ADC_CDR[4];
    }
  }
  if (CAN_READ_ADC_6) {
    if (st == open_circuit) {
      shunt_open = ADC->ADC_CDR[6];
    } else {
      shunt_high = ADC->ADC_CDR[6];
    }
  }
}

void charge_cycle_start() {
  st = high;
  if (adcSetpoint == VtoADC(0)) {
    BRAKE();
  } else if (load_open < adcSetpoint) {
    FORWARD();
  } else if (load_open > adcSetpoint) {
    REVERSE();
  }

  load_open = -1;
  shunt_open = -1;
  load_high = -1;
  shunt_high = -1;

  TC_Start(TC0, 0);
  TRIGGER_ADC();
}

uint32_t start = 0;
void loop() {
  while(Serial.available()) {
    char c = Serial.read();
    switch(c) {
      case 'f': adcSetpoint = ADC_SET_MAX; break;
      case 'r': adcSetpoint = ADC_SET_MIN; break;
      case 'b': adcSetpoint = ADC_SET_ZERO; break;
      case 's': start = millis(); charge_cycle_start(); break;
      case 'm': TC0_Handler(); break;
    }
  }
  if (load_open != -1 && shunt_open != -1 && load_high != -1 && shunt_high != -1) {
    Serial.println((String)"" + (millis() - start) + "," + ADCtoV(adcSetpoint) + "," + ADCtoV(load_open) + "," + ADCtoV(shunt_open) + "," + ADCtoV(load_high) + "," + ADCtoV(shunt_high));
    charge_cycle_start();
  }
}


// /*
//  * rosserial Publisher Example
//  * Prints "hello world!"
//  */

// #include <ros.h>
// #include <std_msgs/String.h>

// class NewHardware : public ArduinoHardware
// {
//   public:
//   NewHardware():ArduinoHardware(&Serial1, 57600){};
// };

// ros::NodeHandle_<NewHardware>  nh;


// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);

// char hello[] = "1234";

// void setup()
// {
//   nh.initNode();
//   nh.advertise(chatter);
// }

// void loop()
// {
//   str_msg.data = hello;
//   chatter.publish( &str_msg );
//   nh.spinOnce();
//   delay(1);
// }

// volatile uint8_t t = 0;
// void TC0_Handler() {
//   t = 1;
//   TC0->TC_CHANNEL[0].TC_SR;
//   TC_Start(TC0, 0);
// }

// void setup_timer() {
//   PMC->PMC_PCER0 |= PMC_PCER0_PID27;

//   TC_Configure(TC0, 0, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVSEL_UP | TC_CMR_WAVE);
//   TC_SetRC(TC0, 0, 42000);
//   TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
//   NVIC_EnableIRQ(TC0_IRQn);
// }

// void setup() {
//   Serial.begin(115200);
// }

// void loop() {
//   if (t) {
//     t = 0;
//     Serial.println((String)"timer: " + millis());
//   }
//   TC_Start(TC0, 0);
//   // Serial.println((String)"loop: " + millis());
//   // delay(1000);
// }