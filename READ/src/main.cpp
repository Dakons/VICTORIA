#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>


Adafruit_INA219 ina219_1;

unsigned long sensTimer = 0;


float dist_3[3] = {0.0, 0.0, 0.0};


float Curr_1, Curr_2, Curr_3;
float middle, dist, dist_filtered; // Для фильтрации

float k; //Коэффициент для бегущего среднего
byte i, delta; //счётчики


//Adafruit_INA219 ina219_2;
float middle_of_3(float a, float b, float c);
float ReadAndFilterUS();

float convertToMillimeters(float sensorValue);


void setup() 
{
Wire.begin(21, 22);
Serial.begin(115200);
//Serial.println("clean, median");  
//ina219_1.setCalibration_32V_2A();
ina219_1.setCalibration_16V_400mA();
if (! ina219_1.begin()) {       
    //Serial.println("Failed to find INA219 chip");  
    while (1) { delay(10); }
  }
//ina219_2.begin();

}



void loop() 

{
Serial.println(convertToMillimeters(ReadAndFilterUS()));
//delay(80);
//Curr_1 = ReadAndFilterUS();
//Serial.println(Curr_1);
//Serial.println(ina219_1.getCurrent_mA());

/*
Serial.print(ina219_1.getCurrent_mA());
Serial.print(',');
Curr_1 = ReadAndFilterUS();
Serial.print(Curr_1);
Serial.println();
*/

}


float middle_of_3(float a, float b, float c) 
{
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  }
  else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}


float ReadAndFilterUS()
{
  if (millis() - sensTimer > 1)
  {                          // измерение и вывод каждые 50 мс
    // счётчик от 0 до 2
    // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу
    //if (i > 1) i = 0;
    //else i++;

    //dist_3[i] = ina219_1.getCurrent_mA();                 // получить расстояние в текущую ячейку массива
    //dist = middle_of_3(dist_3[0], dist_3[1], dist_3[2]);    // фильтровать медианным фильтром из 3ёх последних измерений
      dist = ina219_1.getCurrent_mA();
    
    delta = abs(dist_filtered - dist);                      // расчёт изменения с предыдущим
    if (delta > 1)  // если большое - резкий коэффициент
    {
      k = 0.95;
    } 
    else if (delta <= 1 && delta > 0.3)
    {
      k = 0.8;
    }     
    else if (delta <= 0.3 && delta >=0.1)
    {
      k = 0.1;
    }
      else if (delta <= 0.1)
    {
      k = 0.05;
    }
                                   // если маленькое - плавный коэффициент

    dist_filtered += ( dist -dist_filtered) * k;     // фильтр "бегущее среднее"
    
    sensTimer = millis();                                   // сбросить таймер

  }
  return dist_filtered + 0.35; //+0.43
}

float convertToMillimeters(float sensorValue)
 {
  if (sensorValue < 3.7)
  {
    return 0.0;
  }
  else if (sensorValue >= 3.7 && sensorValue < 4.0)
  {
    sensorValue = 4.0;
  }
  
  if (sensorValue > 22.0)
  {
    return 1.0;
  }
  else if (sensorValue >= 20.0 && sensorValue <= 22.0)
  {
    sensorValue = 20.0;
  }


  float minSensorValue = 4.0;  // Минимальное значение с токового датчика (в мА)
  float minMillimeters = 100.0;  // Минимальное значение в миллиметрах
  float maxSensorValue = 20.0;  // Максимальное значение с токового датчика (в мА)
  float maxMillimeters = 2000.0;  // Максимальное значение в миллиметрах

  // Выполняем линейную интерполяцию
  float millimeters = ((sensorValue - minSensorValue) / (maxSensorValue - minSensorValue)) * (maxMillimeters - minMillimeters) + minMillimeters;

  return millimeters;
}