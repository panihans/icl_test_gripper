#include <Arduino.h>
#include <math.h>

void setupADC() {
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  //https://forum.arduino.cc/t/read-a-differential-signal-using-the-arduino-due-adc/456265/4
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_CR = ADC_CR_SWRST;

  ADC->ADC_MR |= ADC_MR_ANACH_ALLOWED | ADC_MR_LOWRES_BITS_12 | ADC_MR_STARTUP_SUT24; // separate channels + 12bit + delay for startup for enable->disable->enable...
  ADC->ADC_COR = ADC_COR_DIFF4 | ADC_COR_DIFF5 | ADC_COR_OFF4 | ADC_COR_OFF5 | 
                  ADC_COR_DIFF6 | ADC_COR_DIFF7 | ADC_COR_OFF6 | ADC_COR_OFF7; // enable differential for channels and 0.5 offsets

  // ADC->ADC_CHER |= ADC_CHER_CH4 | ADC_CHER_CH5 | ADC_CHER_CH6 | ADC_CHER_CH7;

  // enable interrupts
  // NVIC_EnableIRQ(ADC_IRQn);
  // ADC->ADC_IER |= ADC_IER_EOC4 | ADC_IER_EOC6;
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

int adcSetpoint = VtoADC(0);


void setup() {
  // put your setup code here, to run once:
  setupADC();
  Serial.begin(115200);
  Serial.println("begin");
}

int ocv_prev = 0;
int ocv = 0;
int cur_prev = 0;
int cur = 0;
int phaseCHARGE() {
  int start = millis();
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  if (ocv > adcSetpoint) {
    digitalWrite(7, LOW);
    digitalWrite(6, HIGH);
  } else {
    digitalWrite(7, HIGH);
    digitalWrite(6, LOW);
  }
  delay(5);

  ADC->ADC_CHER |= ADC_CHER_CH6 | ADC_CHER_CH7;
  triggerADC();
  while(!(ADC->ADC_ISR & ADC_ISR_EOC6)) {
  }
  cur_prev = cur;
  cur = ADC->ADC_CDR[6];
  ADC->ADC_CHDR |= ADC_CHDR_CH6 | ADC_CHDR_CH7;

  pinMode(7, INPUT);
  pinMode(6, INPUT);
  return millis() - start;
}

int phaseOPEN() {
  ADC->ADC_CHER |= ADC_CHER_CH4 | ADC_CHER_CH5;

  triggerADC();
  while(!(ADC->ADC_ISR & ADC_ISR_EOC4)) {
  }

  ocv_prev = ocv;
  ocv = ADC->ADC_CDR[4];

  ADC->ADC_CHDR |= ADC_CHDR_CH4 | ADC_CHDR_CH5;
  return ocv;
}

void measurePARAM() {
  adcSetpoint = VtoADC(1);
  int ocvs = phaseOPEN();
  int dt = phaseCHARGE();
  int vd = cur;
  int ocve = phaseOPEN();

  float vend = 3.3f - ADCtoV(ocve);
  float resistance = vend / ADCtoV(vd) * 1000;

  float x1 = (ADCtoV(ocvs) - ADCtoV(ocve)) / (3.3f - ADCtoV(ocvs));
  float x2 = log(1/(1-x1));
  float x3 = dt / x2;
  float x4 = x3 / resistance * 1000;
  Serial.println((String)"ocvs=" + ocvs + ",dt=" + dt + ",vd=" + vd + ",ocve=" + ocve);
  Serial.println((String)"r=" + resistance + ",x1=" + x1 + ",x2=" + x2 + ",x3=" + x3 + ",x4=" + x4);
  Serial.println("---");
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

  // measurePARAM();
  phaseOPEN();
  phaseCHARGE();
  

  // Serial.println((String)"adc:" + adc + ", adcc:"+ADCtoV(adc));
  // Serial.println((String)"" + ADCtoV(ocv) + "V, " + ADCtoV(cur) + "mA");
  // Serial.println(ADCtoV(adc2));
  delay(1000);
}

void ADC_Handler() {
  readADC();
}