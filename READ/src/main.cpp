#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219_1;

//Adafruit_INA219 ina219_2;

void setup() 
{
Wire.begin(21, 22);
Serial.begin(9600);  
ina219_1.setCalibration_16V_400mA();
//ina219_2.setCalibration_16V_400mA();
if (! ina219_1.begin()) {       
    Serial.println("Failed to find INA219 chip");  
    while (1) { delay(10); }
  }
//ina219_2.begin();

}

void loop() 

{

Serial.println(ina219_1.getBusVoltage_V());

delay(100);
//Serial.println(ina219_2.getCurrent_mA());
}

