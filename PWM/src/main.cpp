#include <Arduino.h>
#include "esp32-hal-ledc.h"

int freq = 2;
int ledChannel = 0;
int resolution = 8;
int dutyCycle = 50;
int ledPin = 2;

void setup()
 {
  Serial.begin(9600);
  pinMode(5, INPUT);
  pinMode(15, INPUT);
  pinMode(4, OUTPUT);
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
}

void loop() {
  //ledcWrite(0, dutyCycle * 255 / 100); // задаем скважность в ШИМ-сигнале
  //ledcWrite(ledChannel, dutyCycle);
digitalWrite(4, HIGH);
  Serial.println("Сигнал на пине 18: " + String(analogRead(15)));

delay(250);
}