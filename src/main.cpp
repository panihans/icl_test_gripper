#include <Arduino.h>

void setupADC() {
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  //https://forum.arduino.cc/t/read-a-differential-signal-using-the-arduino-due-adc/456265/4
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_CR = ADC_CR_SWRST;

  ADC->ADC_MR |= ADC_MR_ANACH_ALLOWED | ADC_MR_LOWRES_BITS_12;
  ADC->ADC_COR = ADC_COR_DIFF4 | ADC_COR_DIFF5 | ADC_COR_OFF4 | ADC_COR_OFF5 | 
                  ADC_COR_DIFF6 | ADC_COR_DIFF7 | ADC_COR_OFF6 | ADC_COR_OFF7;

  ADC->ADC_CHER |= ADC_CHER_CH4 | ADC_CHER_CH5 | ADC_CHER_CH6 | ADC_CHER_CH7;

  // enable interrupts
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_IER |= ADC_IER_EOC4 | ADC_IER_EOC6;
}

void triggerADC() {
  ADC->ADC_CR = ADC_CR_START;
}

int adc = 0;
int adc2 = 0;
int state = 0;
void readADC() {
  if (ADC->ADC_ISR & ADC_ISR_EOC4) {
    if (state) {
      adc = ADC->ADC_CDR[4];
    } else {
      volatile int temp = ADC->ADC_CDR[4];
    }
  }
  if (ADC->ADC_ISR & ADC_ISR_EOC6) {
    if (state) {
      volatile int temp = ADC->ADC_CDR[6];
    } else {
      adc2 = ADC->ADC_CDR[6];
    }
  }
}


#define VREF 3.3f
#define ADC_MAX 4095.f
#define ADCtoV(adc) ((adc) / ADC_MAX * (VREF * 2) - VREF)
#define VtoADC(v) ((v) / VREF + 1) * (ADC_MAX / 2)

int adcL = VtoADC(0);
int adcSetpoint = VtoADC(0);


void setup() {
  // put your setup code here, to run once:
  setupADC();
  Serial.begin(115200);
  Serial.println("begin");
}

void loop() {
  // put your main code here, to run repeatedly:
  char chars[128] = {0};
  int i = 0;
  while(Serial.available()) {
    char c = Serial.read();
    if (c) {
      chars[i++] = c;
    }
  }
  if (chars[0]) {
    String in = String(chars);
    // Serial.println((String)"in: " + in);
    in.replace("=", "");
    in.replace("\r", "");
    in.replace("\n", "");
    float num = in.toFloat();
    adcSetpoint = VtoADC(num);
  }

  state = 0;
  pinMode(A8, OUTPUT);
  pinMode(A9, OUTPUT);
  if (adc > adcSetpoint) {
    digitalWrite(A8, LOW);
    digitalWrite(A9, HIGH);
  } else {
    digitalWrite(A8, HIGH);
    digitalWrite(A9, LOW);
  }
  triggerADC();
  delay(1);

  state = 1;
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  triggerADC();
  delay(1);
  // Serial.println((String)"adc:" + adc + ", adcc:"+ADCtoV(adc));
  Serial.println((String)"" + ADCtoV(adc) + "V, " + ADCtoV(adc2) + "mA");
  Serial.println(ADC->ADC_ISR);
  delay(1000);
}

void ADC_Handler() {
  readADC();
}