#include <Arduino.h>
#include "esp32-hal-ledc.h"

#define LEDC_CHANNEL 0
#define LEDC_TIMER 0
#define LED_PIN 2

int freq = 10;
int dutyCycle = 50;

void setup()
{
  Serial.begin(115200);
  ledcSetup(LEDC_CHANNEL, freq, 8);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
  ledcWrite(LEDC_CHANNEL, dutyCycle * 255 / 100);
}

void loop()
 {
  
  // читаем новые значения частоты и заполнения с монитора порта
  if (Serial.available() > 0) {
    freq = Serial.parseInt();
    dutyCycle = Serial.parseInt();
    ledcWriteTone(LEDC_TIMER, freq);
    ledcWrite(LEDC_CHANNEL, dutyCycle * 255 / 100);
  }
  delay(100);
}