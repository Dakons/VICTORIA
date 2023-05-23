#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <iarduino_RTC.h>

Adafruit_INA219 ina219_1;
float Curr_1, Curr_2, Curr_3;
float Med = 0;
//Adafruit_INA219 ina219_2;

void setup() 
{
Wire.begin(21, 22);
Serial.begin(9600);  
ina219_1.setCalibration_32V_2A();
//ina219_2.setCalibration_16V_400mA();
if (! ina219_1.begin()) {       
    Serial.println("Failed to find INA219 chip");  
    while (1) { delay(10); }
  }
//ina219_2.begin();

}

void loop() 

{
/*
Curr_1 = ina219_1.getCurrent_mA();
Curr_2 = ina219_1.getCurrent_mA();
Curr_3 = ina219_1.getCurrent_mA();

Med = Curr_1 + Curr_2 + Curr_3;
Med = Med /3;
*/

Serial.println(ina219_1.getBusVoltage_V());


delay(100);
//Serial.println(ina219_2.getCurrent_mA());
}

