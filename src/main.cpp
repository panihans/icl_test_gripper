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
}

void setup() {
  // put your setup code here, to run once:
  setup_adc();
  Serial.begin(115200);
  Serial.println("begin");
}

#define CAN_READ_ADC_4 (ADC->ADC_ISR & ADC_ISR_EOC4)
#define CAN_READ_ADC_6 (ADC->ADC_ISR & ADC_ISR_EOC6)

void get_adc_readings(uint32_t& load, uint32_t& shunt) {
  while(!CAN_READ_ADC_4) asm volatile("NOP");
  // Serial.println(ADC->ADC_ISR, BIN);
  load = ADC->ADC_CDR[4];
  while(!CAN_READ_ADC_6) asm volatile("NOP");
  // Serial.println(ADC->ADC_ISR, BIN);
  shunt = ADC->ADC_CDR[6];
}

#define VOUT 3.3f
#define ADC_MAX 4095.f
#define ADCtoV(adc) ((adc) / ADC_MAX * (VOUT * 2) - VOUT)
#define VtoADC(v) ((v) / VOUT + 1) * (ADC_MAX / 2)

uint32_t adcSetpoint = VtoADC(3);

void loop() {
  // put your main code here, to run repeatedly:
  COAST();
  TRIGGER_ADC();
  uint32_t load = -1;
  uint32_t shunt = -1;
  get_adc_readings(load, shunt);
  Serial.println((String)"c," + load + "," + shunt);

  if (load < adcSetpoint) {
    FORWARD();
    TRIGGER_ADC();
    load = -1;
    shunt = -1;
    get_adc_readings(load, shunt);
    delay(1);
    Serial.println((String)"f," + load + "," + shunt);
    COAST();
  }

  Serial.println(".");
  delay(1000);
}