#include <Arduino.h>
#include "esp32-hal-ledc.h"

#define LEDC_CHANNEL 0
#define LEDC_TIMER 0
#define LED_PIN 2
byte del = 50;
int freq = 0;
unsigned long LastTime_1 = 0;
unsigned long LastTime_2 = 0;
bool State = 0;
int dutyCycle = 0;

void setup()
{
  Serial.begin(115200);
  
  ledcSetup(LEDC_CHANNEL, freq, 8);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
  ledcWrite(LEDC_CHANNEL, dutyCycle * 255 / 100);
  

pinMode(LED_PIN, OUTPUT);

//del = 50;
}

void loop()
 {


digitalWrite(LED_PIN, HIGH);
delay(100);
digitalWrite(LED_PIN, LOW);
delay(100);


/*
if (millis() - LastTime_1 >= 250)
{
  State = !State;
  LastTime_1 = millis();
  digitalWrite(LED_PIN, State);
}

*/
/*
if (millis() - LastTime_2 >= 500)
{
  LastTime_2 = millis();
  State = !State;
  digitalWrite(5, State);
}
*/


  /*
  // читаем новые значения частоты и заполнения с монитора порта
  if (Serial.available() > 0) 
  {
    freq = Serial.parseInt();
    dutyCycle = Serial.parseInt();
    ledcWriteTone(LEDC_TIMER, freq);
    ledcWrite(LEDC_CHANNEL, dutyCycle * 255 / 100);
  }
  delay(100);

  */
/*

  float period = 1.0 / freq; // вычисляем длительность периода ШИМ
  float pulse_width = (dutyCycle / 100.0) * period; // вычисляем длительность импульса ШИМ
  float space_width = period - pulse_width; // вычисляем длительность пробела ШИМ

  Serial.print("Pulse width: ");
  Serial.print(pulse_width);
  Serial.print(" us ");
  Serial.print("Space width: ");
  Serial.print(space_width);
  Serial.println(" us");
  delay(1000);

 */ 
}