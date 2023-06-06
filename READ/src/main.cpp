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



void setup() 
{
Wire.begin(21, 22);
Serial.begin(9600);
Serial.println("clean, median");  
ina219_1.setCalibration_32V_2A();
//ina219_2.setCalibration_16V_400mA();
if (! ina219_1.begin()) {       
    //Serial.println("Failed to find INA219 chip");  
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


Serial.print(ina219_1.getCurrent_mA());
Serial.print(',');
Curr_1 = ReadAndFilterUS();
Serial.print(Curr_1);
Serial.println();


//delay();
//Serial.println(ina219_2.getCurrent_mA());
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
    if (millis() - sensTimer > 50) {                          // измерение и вывод каждые 50 мс
    // счётчик от 0 до 2
    // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу
    if (i > 1) i = 0;
    else i++;

    dist_3[i] = ina219_1.getCurrent_mA();                 // получить расстояние в текущую ячейку массива
    dist = middle_of_3(dist_3[0], dist_3[1], dist_3[2]);    // фильтровать медианным фильтром из 3ёх последних измерений

    delta = abs(dist_filtered - dist);                      // расчёт изменения с предыдущим
    if (delta > 1) k = 0.99;                                 // если большое - резкий коэффициент
    else k = 0.9;                                           // если маленькое - плавный коэффициент

    dist_filtered += ( dist -dist_filtered) * k;     // фильтр "бегущее среднее"

    sensTimer = millis();                                   // сбросить таймер

    
    
  }
  return dist_filtered;
}